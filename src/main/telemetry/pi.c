/*
 * Configure serial port to parse pi-messages and provide facilities to send
 *
 * Copyright 2023 Till Blaha (Delft University of Technology)
 *
 * This file is part of Indiflight.
 *
 * Indiflight is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Indiflight is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 *
 * If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_TELEMETRY_PI)

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/dshot.h"
#include "drivers/sensor.h"
#include "drivers/time.h"
#include "drivers/light_led.h"

#include "config/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/position.h"
#include "flight/rpm_filter.h"

#include "io/serial.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "telemetry/telemetry.h"
#include "telemetry/pi.h"
#include "pi-protocol.h"
#include "pi-messages.h"

#ifdef USE_CLI_DEBUG_PRINT
#include "cli/cli_debug_print.h"
#endif

#define TELEMETRY_PI_INITIAL_PORT_MODE MODE_RXTX
#define TELEMETRY_PI_MAXRATE 50
#define TELEMETRY_PI_DELAY ((1000 * 1000) / TELEMETRY_PI_MAXRATE)

static serialPort_t *piPort = NULL;
static const serialPortConfig_t *portConfig;

static bool piTelemetryEnabled =  false;
static portSharing_e piPortSharing;

// wrapper for serialWrite
static void serialWriter(uint8_t byte) { serialWrite(piPort, byte); }

void freePiTelemetryPort(void)
{
    closeSerialPort(piPort);
    piPort = NULL;
    piTelemetryEnabled = false;
}

void initPiTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_PI);
    piPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_PI);
}

#define BLINK_ONCE delay(500); LED1_ON; delay(100); LED1_OFF; delay(100)

void configurePiTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        baudRateIndex = BAUD_921600;
    }

    piPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_PI, NULL, NULL, baudRates[baudRateIndex], TELEMETRY_PI_INITIAL_PORT_MODE, SERIAL_NOT_INVERTED);

    if (!piPort) {
        return;
    }

    piTelemetryEnabled = true;
}

void checkPiTelemetryState(void)
{
    if (portConfig && telemetryCheckRxPortShared(portConfig, rxRuntimeState.serialrxProvider)) {
        if (!piTelemetryEnabled && telemetrySharedPort != NULL) {
            piPort = telemetrySharedPort;
            piTelemetryEnabled = true;
        }
    } else {
        bool newTelemetryEnabledValue = telemetryDetermineEnabledState(piPortSharing);

        if (newTelemetryEnabledValue == piTelemetryEnabled) {
            return;
        }

        if (newTelemetryEnabledValue)
            configurePiTelemetryPort();
        else
            freePiTelemetryPort();
    }
}

void piSendIMU(void)
{
    piMsgImuTx.time_us = micros();
    piMsgImuTx.roll = DEGREES_TO_RADIANS(gyro.gyroADCf[0]);
    piMsgImuTx.pitch = DEGREES_TO_RADIANS(gyro.gyroADCf[1]);
    piMsgImuTx.yaw = DEGREES_TO_RADIANS(gyro.gyroADCf[2]);
    piMsgImuTx.x = GRAVITYf * ((float)acc.accADC[0]) / ((float)acc.dev.acc_1G);
    piMsgImuTx.y = GRAVITYf * ((float)acc.accADC[1]) / ((float)acc.dev.acc_1G);
    piMsgImuTx.z = GRAVITYf * ((float)acc.accADC[2]) / ((float)acc.dev.acc_1G);
    
    piSendMsg(&piMsgImuTx, &serialWriter);
}

// dshotErpmf[] is indexed in Betaflight's own mixer output order
// ([RR, FR, RL, FL], see mixer_init.c mixerQuadX[]) - NOT the
// indi_controller/simulator convention used elsewhere in the workspace.
//
// Guarded by USE_DSHOT_TELEMETRY, not USE_INDI: dshotErpmf[] is filtered
// directly from getDshotTelemetry() (drivers/dshot.c dshotErpmFiltering(),
// called every taskFiltering() tick independent of which controller is
// active), so motor telemetry works the same whether INDI or stock PID is
// flying. Deliberately NOT reusing indiRun.omega_fs[]: that only exists
// behind USE_INDI (indi.c's own top-level #ifdef, not just an extern
// declaration - referencing it without USE_INDI fails to link), and its
// filtering is tuned for control-loop noise rejection (15Hz default cutoff)
// rather than preserving telemetry bandwidth.
#ifdef USE_DSHOT_TELEMETRY
void piSendMotor(void)
{
    // Same conversion indiRun.erpmToRads uses (indi_init.c), just applied to
    // the INDI-independent filtered value instead: eRPM -> rad/s directly,
    // without the lossy float->uint16_t->float round trip erpmToRpm() would
    // otherwise force.
    const float erpmToRads = ERPM_PER_LSB / SECONDS_PER_MINUTE /
        (motorConfig()->motorPoleCount / 2.f) * (2.f * M_PIf);

    piMsgMotorTx.time_us = micros();
    piMsgMotorTx.omega0 = erpmToRads * dshotErpmf[0];
    piMsgMotorTx.omega1 = erpmToRads * dshotErpmf[1];
    piMsgMotorTx.omega2 = erpmToRads * dshotErpmf[2];
    piMsgMotorTx.omega3 = erpmToRads * dshotErpmf[3];

    piSendMsg(&piMsgMotorTx, &serialWriter);
}
#endif

void processPiTelemetry(void)
{
    piSendIMU();
#ifdef USE_DSHOT_TELEMETRY
    // Motor telemetry every other tick (1000Hz) alongside IMU every tick
    // (2000Hz), to keep total pi-protocol bandwidth comfortably under the
    // UART's budget.
    static uint32_t tick = 0;
    if ((tick++ & 1) == 0) {
        piSendMotor();
    }
#endif
}

pi_parse_states_t p_telem;

void processPiUplink(void)
{
#ifdef PI_BETAFLIGHT_DEBUG
    static unsigned int i = 0;
    if (++i > 3) {
        i = 0;
        LED1_TOGGLE;
        cliPrintLinef("%10d", piStats[PI_PARSE_INVOKE]);
    }
#endif
    if (piPort) {
        while (serialRxBytesWaiting(piPort)) {
            piParse(&p_telem, serialRead(piPort));
        }
    }
}

void handlePiTelemetry(void)
{
    if (!piTelemetryEnabled) {
        return;
    }

    if (!piPort) {
        return;
    }

    processPiTelemetry();
    processPiUplink();
}

#endif
