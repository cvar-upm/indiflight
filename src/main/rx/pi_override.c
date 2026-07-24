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
#include "fc/rc_modes.h"

// RC_OVERRIDE only ever carries these 4 stick channels - unlike MSP_SET_RAW_RC,
// there's no full-channel-count frame to fall back on, so this stays a small
// fixed array. It is NOT indexed by fc/rc_controls.h's ROLL/PITCH/YAW/THROTTLE
// enum (0/1/2/3, canonical/internal order) - rxPiOverrideReadRawRc() below is
// called with `chan` = rawChannel = rxConfig()->rcmap[channel] (see rx/rx.c
// readRxChannelsApplyRanges()), i.e. a *wire* position, not a canonical index.
// For the default "AETR1234" rc_map, canonical YAW(2)/THROTTLE(3) map to wire
// positions 3/2 respectively - the opposite order - so indexing this array
// with ROLL/PITCH/YAW/THROTTLE would silently swap the yaw and throttle
// override values on any FC using that default map.
//
// mspFrame[] (rx/msp.c) sidesteps this by being wire-order to begin with: the
// host always sends MSP_SET_RAW_RC as roll/pitch/throttle/yaw, which is
// exactly "AETR" wire order, and rxMspFrameReceive() copies it in verbatim
// with no permutation. This array mirrors that same convention instead of
// rc_controls.h's, since it's read the same way (rawChannel-indexed).
enum {
    PI_OVERRIDE_WIRE_ROLL = 0,
    PI_OVERRIDE_WIRE_PITCH = 1,
    PI_OVERRIDE_WIRE_THROTTLE = 2,
    PI_OVERRIDE_WIRE_YAW = 3,
};

static uint16_t piOverrideFrame[4];

// Latches true on the first RC_OVERRIDE ever received, never cleared again -
// this guards only against engaging BOXPIOVERRIDE before the companion
// computer has sent a single frame (piOverrideFrame[] otherwise reads as its
// zero-initialized default, an invalid pulse Betaflight's own RX-failure
// detection then reacts to). It is NOT a staleness/timeout check: once the
// first frame lands, this stays true even if RC_OVERRIDE later stops
// arriving - matches msp_override.c's own lack of a freshness guard there,
// just closing the specific "activated before the platform ever started"
// gap instead.
static bool piOverrideFrameValid = false;

void rxPiOverrideFrameReceive(uint16_t roll, uint16_t pitch, uint16_t yaw, uint16_t throttle)
{
    piOverrideFrame[PI_OVERRIDE_WIRE_ROLL] = roll;
    piOverrideFrame[PI_OVERRIDE_WIRE_PITCH] = pitch;
    piOverrideFrame[PI_OVERRIDE_WIRE_THROTTLE] = throttle;
    piOverrideFrame[PI_OVERRIDE_WIRE_YAW] = yaw;
    piOverrideFrameValid = true;
}

uint16_t rxPiOverrideReadRawRc(const rxRuntimeState_t *rxRuntimeState, const rxConfig_t *rxConfig, uint8_t chan)
{
    uint16_t rxSample = (rxRuntimeState->rcReadRawFn)(rxRuntimeState, chan);

    bool override = chan < 4 && ((1 << chan) & rxConfig->pi_override_channels_mask);

    if (IS_RC_MODE_ACTIVE(BOXPIOVERRIDE) && override && piOverrideFrameValid) {
        return piOverrideFrame[chan];
    } else {
        return rxSample;
    }
}
#endif
