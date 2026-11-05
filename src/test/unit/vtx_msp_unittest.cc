/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

extern "C" {
    #include "platform.h"

    #include "drivers/vtx_common.h"
    #include "drivers/vtx_table.h"
    #include "fc/runtime_config.h"
    #include "io/serial.h"
    #include "io/vtx.h"
    #include "io/vtx_msp.h"
    #include "pg/vtx_table.h"

    static uint8_t mspFrame[15];
    extern mspVtxStatus_e mspVtxStatus;

    static bool vtxOnModeMapped = false;
    static bool vtxOnModeActive = false;
    static bool failsafeActive = false;
    static serialPortConfig_t *findSerialPortConfig_stub_retval;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

//make clean test_vtx_msp_unittest
TEST(VtxMspUnitTest, TestVtxMspPacket)
{
    vtxSettingsConfigMutable()->band = 1;
    vtxSettingsConfigMutable()->channel = 2;
    vtxSettingsConfigMutable()->power = 3;
    vtxSettingsConfigMutable()->freq = 5800;
    vtxSettingsConfigMutable()->pitModeFreq = 5300;
    vtxSettingsConfigMutable()->lowPowerDisarm = VTX_LOW_POWER_DISARM_ALWAYS;

    vtxTableConfigMutable()->bands = 4;
    vtxTableConfigMutable()->channels = 5;
    vtxTableConfigMutable()->powerLevels = 6;

    uint8_t expectedFrame[15] = {5, 1, 2, 1, 0, 168, 22, 0, 1, 180, 20, 1, 4, 5, 6};

    prepareMspFrame(mspFrame);

    for (int i = 0; i < 15; i++) {
        EXPECT_EQ(expectedFrame[i], mspFrame[i]);
    }
}

TEST(VtxMspUnitTest, TestVtxMspReady)
{
    mspVtxStatus = MSP_VTX_STATUS_OFFLINE;
    vtxTableConfigMutable()->bands = 0;
    vtxTableConfigMutable()->channels = 0;
    vtxTableConfigMutable()->powerLevels = 0;

    setMspVtxDeviceStatusReady(0);

    EXPECT_EQ(MSP_VTX_STATUS_OFFLINE, mspVtxStatus);

    vtxTableConfigMutable()->bands = 1;
    vtxTableConfigMutable()->channels = 1;
    vtxTableConfigMutable()->powerLevels = 1;

    setMspVtxDeviceStatusReady(1); // testing wrong descriptor

    EXPECT_EQ(MSP_VTX_STATUS_OFFLINE, mspVtxStatus);

    setMspVtxDeviceStatusReady(0);

    EXPECT_EQ(MSP_VTX_STATUS_READY, mspVtxStatus);
}

static void setArmed(bool armed)
{
    armingFlags = armed ? ARMED : static_cast<armingFlag_e>(0);
}

class VtxMspSwitchTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        vtxSettingsConfigMutable()->power = 1;
        vtxSettingsConfigMutable()->lowPowerDisarm = VTX_LOW_POWER_DISARM_OFF;
        vtxTablePowerLevels = 3;
        vtxTablePowerValues[0] = 14;
        vtxTablePowerValues[1] = 23;
        vtxTablePowerValues[2] = 0;
        vtxOnModeMapped = true;
        vtxOnModeActive = false;
        failsafeActive = false;
        setArmed(false);
    }

    void TearDown() override
    {
        vtxOnModeMapped = false;
        vtxOnModeActive = false;
        failsafeActive = false;
        setArmed(false);
    }
};

TEST_F(VtxMspSwitchTest, UnmappedModeLeavesPowerAlone)
{
    vtxOnModeMapped = false;
    EXPECT_FALSE(vtxMspSwitchControlled());
    EXPECT_FALSE(vtxMspForcedOff());
    EXPECT_EQ(1, vtxMspPowerIndexForVtx());
}

TEST_F(VtxMspSwitchTest, SwitchLowDisarmedIsOff)
{
    EXPECT_TRUE(vtxMspForcedOff());
    EXPECT_EQ(3, vtxMspPowerIndexForVtx());

    prepareMspFrame(mspFrame);
    EXPECT_EQ(3, mspFrame[3]);
    // the configured power stays, so a save cannot store the 0mW level
    EXPECT_EQ(1, vtxSettingsConfig()->power);
}

TEST_F(VtxMspSwitchTest, SwitchHighIsOn)
{
    vtxOnModeActive = true;
    EXPECT_FALSE(vtxMspForcedOff());
    EXPECT_EQ(1, vtxMspPowerIndexForVtx());
}

TEST_F(VtxMspSwitchTest, ArmedIsOnWithSwitchLow)
{
    setArmed(true);
    EXPECT_FALSE(vtxMspForcedOff());
    EXPECT_EQ(1, vtxMspPowerIndexForVtx());
}

TEST_F(VtxMspSwitchTest, FailsafeIsOnWithSwitchLow)
{
    failsafeActive = true;
    EXPECT_FALSE(vtxMspForcedOff());
    EXPECT_EQ(1, vtxMspPowerIndexForVtx());
}

TEST_F(VtxMspSwitchTest, TableWithoutOffLevelIsNeverForcedOff)
{
    vtxTablePowerValues[2] = 27;
    EXPECT_FALSE(vtxMspSwitchControlled());
    EXPECT_FALSE(vtxMspForcedOff());
    EXPECT_EQ(1, vtxMspPowerIndexForVtx());
}

TEST_F(VtxMspSwitchTest, ArmingIsPendingUntilProcessed)
{
    serialPortConfig_t portConfig = {};
    findSerialPortConfig_stub_retval = &portConfig;
    ASSERT_TRUE(vtxMspInit());
    ASSERT_NE(nullptr, vtxCommonDevice());
    mspVtxStatus = MSP_VTX_STATUS_READY;

    vtxCommonProcess(vtxCommonDevice(), 0);
    EXPECT_FALSE(vtxMspOffStatePending());

    setArmed(true);
    EXPECT_TRUE(vtxMspOffStatePending());

    vtxCommonProcess(vtxCommonDevice(), 0);
    EXPECT_FALSE(vtxMspOffStatePending());

    setArmed(false);
    EXPECT_TRUE(vtxMspOffStatePending());
}

// STUBS

extern "C" {

    #include "build/debug.h"
    #include "fc/rc_modes.h"
    #include "rx/rx.h"
    #include "io/serial.h"
    #include "msp/msp.h"

    uint8_t debugMode = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];

    uint8_t cliMode = 0;
    rxRuntimeState_t rxRuntimeState = rxRuntimeState_t();

    bool IS_RC_MODE_ACTIVE(boxId_e boxId) {return boxId == BOXVTXON && vtxOnModeActive;}
    bool isModeActivationConditionPresent(boxId_e boxId) {return boxId == BOXVTXON && vtxOnModeMapped;}
    void beeperConfirmationBeeps(uint8_t) {}
    void saveConfigAndNotify(void) {}
    void crsfRxSendTelemetryData(void) {}
    void crsfRxWriteTelemetryData(const void *, int) {}
    bool failsafeIsActive(void) {return failsafeActive;}
    const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e) {return findSerialPortConfig_stub_retval;}
    bool isRangeActive(uint8_t , const channelRange_t *) {return true;}
    int mspSerialPush(serialPortIdentifier_e, uint8_t, uint8_t *, int, mspDirection_e, mspVersion_e) {return 0;}
    void tfp_sprintf(char *, char*, ...) {}

    mspDescriptor_t getMspSerialPortDescriptor(const uint8_t ) {return 0;}
    mspDescriptor_t getMspTelemetryDescriptor(void) {return 0;}

    void mspCmsUpdateStatusString(void) {}
}
