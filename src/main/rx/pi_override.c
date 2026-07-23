/*
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

#include "platform.h"

#if defined(USE_RX_PI_OVERRIDE)

#include "rx/pi_override.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"

// RC_OVERRIDE only ever carries these 4 stick channels - unlike MSP_SET_RAW_RC,
// there's no full-channel-count frame to fall back on, so this stays a small
// fixed array indexed by the same ROLL/PITCH/YAW/THROTTLE indices rc_controls.h
// already defines, rather than mirroring mspFrame[MAX_SUPPORTED_RC_CHANNEL_COUNT].
static uint16_t piOverrideFrame[4];

void rxPiOverrideFrameReceive(uint16_t roll, uint16_t pitch, uint16_t yaw, uint16_t throttle)
{
    piOverrideFrame[ROLL] = roll;
    piOverrideFrame[PITCH] = pitch;
    piOverrideFrame[YAW] = yaw;
    piOverrideFrame[THROTTLE] = throttle;
}

uint16_t rxPiOverrideReadRawRc(const rxRuntimeState_t *rxRuntimeState, const rxConfig_t *rxConfig, uint8_t chan)
{
    uint16_t rxSample = (rxRuntimeState->rcReadRawFn)(rxRuntimeState, chan);

    bool override = chan < 4 && ((1 << chan) & rxConfig->pi_override_channels_mask);

    if (IS_RC_MODE_ACTIVE(BOXPIOVERRIDE) && override) {
        return piOverrideFrame[chan];
    } else {
        return rxSample;
    }
}
#endif
