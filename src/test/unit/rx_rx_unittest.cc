/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * rx.c channel pipeline: frame check -> read channels (rcmap) -> per-channel
 * validation, hold and Stage 1 failsafe values -> rcInput. A fake receiver
 * is plugged into rxRuntimeState after rxInit().
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern "C" {
#include "platform.h"

#include "common/bitarray.h"

#include "pg/pg.h"
#include "pg/rx.h"
#include "pg/modes.h"

#include "fc/rc.h"
#include "fc/rc_modes.h"

#include "rx/rx.h"

#include "build/debug.h"
#include "fc/runtime_config.h"
#include "io/serial.h"

bool isPulseValid(uint16_t pulseDuration);
uint16_t updateLinkQualitySamples(uint16_t value);
}

#include "gtest/gtest.h"

// Fake receiver
static float rxChannels[MAX_SUPPORTED_RC_CHANNEL_COUNT];
static uint8_t rxStatus;

static float fakeReadRawRC(const rxRuntimeState_t *, uint8_t channel) { return rxChannels[channel]; }
static uint8_t fakeFrameStatus(rxRuntimeState_t *) { return rxStatus; }
static bool fakeProcessFrame(const rxRuntimeState_t *) { return true; }

static timeUs_t nowUs;
static int validDataCount;
static int failedDataCount;

extern "C" {
timeUs_t micros(void) { return nowUs; }
timeMs_t millis(void) { return nowUs / 1000; }

uint16_t getFailsafeThrottle(void) { return 1000; }
uint16_t getFailsafeCenter(void) { return 1500; }

void failsafeOnValidDataReceived(void) { validDataCount++; }
void failsafeOnValidDataFailed(void) { failedDataCount++; }
void failsafeOnRxSuspend(uint32_t) {}
void failsafeOnRxResume(void) {}

bool taskUpdateRxMainInProgress(void) { return false; }

uint8_t armingFlags;
uint8_t debugMode;
int32_t debug[DEBUG_VALUE_COUNT];
float rcCommand[MAX_SUPPORTED_RC_CHANNEL_COUNT];

// Serial receivers are not under test
bool crsfRxInit(const rxConfig_t *, rxRuntimeState_t *) { return false; }
bool ibusInit(const rxConfig_t *, rxRuntimeState_t *) { return false; }
bool ibus2Init(const rxConfig_t *, rxRuntimeState_t *) { return false; }
bool jetiExBusInit(const rxConfig_t *, rxRuntimeState_t *) { return false; }
bool sbusInit(const rxConfig_t *, rxRuntimeState_t *) { return false; }
bool spektrumInit(const rxConfig_t *, rxRuntimeState_t *) { return false; }
bool sumdInit(const rxConfig_t *, rxRuntimeState_t *) { return false; }
bool sumhInit(const rxConfig_t *, rxRuntimeState_t *) { return false; }
bool xBusInit(const rxConfig_t *, rxRuntimeState_t *) { return false; }
void rxMspInit(const rxConfig_t *, rxRuntimeState_t *) {}
const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e) { return NULL; }
serialPortUsage_t *findSerialPortUsageByIdentifier(serialPortIdentifier_e) { return NULL; }
void closeSerialPort(serialPort_t *) {}
}

class RxTest : public ::testing::Test {
  protected:
    void SetUp() override
    {
        pgResetAll();
        // Identity channel map, so rxChannels[] lines up with rcInput[]
        parseRcChannels("AERCT123", rxConfigMutable());
        nowUs = 1000 * 1000;
        validDataCount = failedDataCount = 0;
        boxBitmask_t none;
        memset(&none, 0, sizeof(none));
        rcModeUpdate(&none);
        for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++)
            rxChannels[i] = 1500;
    }

    void init()
    {
        rxInit();
        rxRuntimeState.rxProvider = RX_PROVIDER_SERIAL;
        rxRuntimeState.rcReadRawFn = fakeReadRawRC;
        rxRuntimeState.rcFrameStatusFn = fakeFrameStatus;
        rxRuntimeState.rcProcessFrameFn = fakeProcessFrame;
    }

    // One RX task cycle, 10 ms later
    void cycle(uint8_t status = RX_FRAME_COMPLETE)
    {
        nowUs += 10 * 1000;
        rxStatus = status;
        rxFrameCheck(nowUs, 10 * 1000);
        calculateRxChannelsAndUpdateFailsafe(nowUs);
    }

    void cycles(int n, uint8_t status = RX_FRAME_COMPLETE)
    {
        for (int i = 0; i < n; i++)
            cycle(status);
    }

    static void setFailsafeSwitch(bool on)
    {
        boxBitmask_t mask;
        memset(&mask, 0, sizeof(mask));
        if (on)
            bitArraySet(&mask, BOXFAILSAFE);
        rcModeUpdate(&mask);
    }
};

TEST_F(RxTest, PulseValidity)
{
    rxConfigMutable()->rx_pulse_min = 885;
    rxConfigMutable()->rx_pulse_max = 2115;

    EXPECT_FALSE(isPulseValid(884));
    EXPECT_TRUE(isPulseValid(885));
    EXPECT_TRUE(isPulseValid(1500));
    EXPECT_TRUE(isPulseValid(2115));
    EXPECT_FALSE(isPulseValid(2116));
}

TEST_F(RxTest, ParseRcChannels)
{
    rxConfig_t config;
    parseRcChannels("AETRC1234", &config);
    // AERCT order: Aileron, Elevator, Rudder, Collective, Throttle
    EXPECT_EQ(0, config.rcmap[0]);   // A
    EXPECT_EQ(1, config.rcmap[1]);   // E
    EXPECT_EQ(3, config.rcmap[2]);   // R
    EXPECT_EQ(4, config.rcmap[3]);   // C
    EXPECT_EQ(2, config.rcmap[4]);   // T
    EXPECT_EQ(5, config.rcmap[5]);   // 1
    EXPECT_EQ(7, config.rcmap[7]);   // 3 (only the first 8 channels are mappable)

    // Letters not given keep their own position
    parseRcChannels("TAE", &config);
    EXPECT_EQ(1, config.rcmap[0]);
    EXPECT_EQ(2, config.rcmap[1]);
    EXPECT_EQ(2, config.rcmap[2]);
    EXPECT_EQ(0, config.rcmap[4]);
}

TEST_F(RxTest, InitialValues)
{
    init();

    EXPECT_FALSE(rxIsReceivingSignal());
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        if (i == THROTTLE)
            EXPECT_EQ(RX_PWM_PULSE_MIN, rcInput[i]);
        else
            EXPECT_EQ(rcControlsConfig()->rc_center, rcInput[i]) << "channel " << i;
    }
}

TEST_F(RxTest, ArmSwitchStartsOff)
{
    // ARM on aux 1 high: initialised one step below the range
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->auxChannelIndex = 1;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);
    init();
    EXPECT_EQ(1695, rcInput[CONTROL_CHANNEL_COUNT + 1]);

    // ARM on aux 1 low: one step above the range
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(1300);
    init();
    EXPECT_EQ(1305, rcInput[CONTROL_CHANNEL_COUNT + 1]);
}

TEST_F(RxTest, GoodFramesReachRcInput)
{
    init();
    rxChannels[ROLL] = 1600;
    rxChannels[THROTTLE] = 1200;
    rxChannels[CONTROL_CHANNEL_COUNT] = 1900;

    cycle();

    EXPECT_TRUE(rxIsReceivingSignal());
    EXPECT_TRUE(rxAreFlightChannelsValid());
    EXPECT_EQ(1600, rcInput[ROLL]);
    EXPECT_EQ(1200, rcInput[THROTTLE]);
    EXPECT_EQ(1900, rcInput[CONTROL_CHANNEL_COUNT]);
    EXPECT_EQ(1, validDataCount);
}

TEST_F(RxTest, DefaultRcmap)
{
    // Default channel order is AETRC: throttle on receiver channel 3
    pgResetAll();
    EXPECT_EQ(2, rxConfig()->rcmap[THROTTLE]);
    EXPECT_EQ(3, rxConfig()->rcmap[YAW]);
    EXPECT_EQ(4, rxConfig()->rcmap[COLLECTIVE]);
}

TEST_F(RxTest, RcmapIsApplied)
{
    // Receiver sends throttle on its channel 2
    parseRcChannels("AETRC", rxConfigMutable());
    init();
    rxChannels[2] = 1234;
    rxChannels[3] = 1555;

    cycle();

    EXPECT_EQ(1234, rcInput[THROTTLE]);
    EXPECT_EQ(1555, rcInput[YAW]);
}

TEST_F(RxTest, BadChannelHeldThenFailsafe)
{
    init();
    rxChannels[ROLL] = 1600;
    rxChannels[THROTTLE] = 1700;
    cycles(5);

    // Out-of-range pulses: last good value held for 300 ms
    rxChannels[ROLL] = 500;
    rxChannels[THROTTLE] = 500;
    cycles(29);
    EXPECT_EQ(1600, rcInput[ROLL]);
    EXPECT_EQ(1700, rcInput[THROTTLE]);
    EXPECT_TRUE(rxAreFlightChannelsValid());

    // ...then Stage 1 values: sticks centre, throttle off
    cycles(2);
    EXPECT_EQ(1500, rcInput[ROLL]);
    EXPECT_EQ(1000, rcInput[THROTTLE]);
    EXPECT_FALSE(rxAreFlightChannelsValid());
    EXPECT_GT(failedDataCount, 0);
}

TEST_F(RxTest, AuxFailsafeModes)
{
    // aux 0: HOLD (default for aux), aux 1: SET to 1800
    rxFailsafeChannelConfigsMutable(CONTROL_CHANNEL_COUNT + 1)->mode = RX_FAILSAFE_MODE_SET;
    rxFailsafeChannelConfigsMutable(CONTROL_CHANNEL_COUNT + 1)->step = CHANNEL_VALUE_TO_RXFAIL_STEP(1800);
    init();

    rxChannels[CONTROL_CHANNEL_COUNT] = 1300;
    rxChannels[CONTROL_CHANNEL_COUNT + 1] = 1300;
    cycles(5);

    rxChannels[CONTROL_CHANNEL_COUNT] = 3000;
    rxChannels[CONTROL_CHANNEL_COUNT + 1] = 3000;
    cycles(40);

    EXPECT_EQ(1300, rcInput[CONTROL_CHANNEL_COUNT]);
    EXPECT_EQ(1800, rcInput[CONTROL_CHANNEL_COUNT + 1]);
    // Bad aux channels don't invalidate the flight channels
    EXPECT_TRUE(rxAreFlightChannelsValid());
}

TEST_F(RxTest, SignalLoss)
{
    init();
    rxChannels[PITCH] = 1400;
    cycles(5);
    ASSERT_TRUE(rxIsReceivingSignal());

    // No frames: the signal is declared lost after 100 ms
    cycles(9, RX_FRAME_PENDING);
    EXPECT_TRUE(rxIsReceivingSignal());
    cycles(2, RX_FRAME_PENDING);
    EXPECT_FALSE(rxIsReceivingSignal());

    // Frames flagged failsafe by the receiver don't count as signal
    cycles(5);
    ASSERT_TRUE(rxIsReceivingSignal());
    cycles(11, RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE);
    EXPECT_FALSE(rxIsReceivingSignal());

    // Lost signal is processed every 100 ms: values go to Stage 1 after 300 ms
    cycles(40, RX_FRAME_PENDING);
    EXPECT_EQ(1500, rcInput[PITCH]);
    EXPECT_FALSE(rxAreFlightChannelsValid());
}

TEST_F(RxTest, FailsafeSwitch)
{
    init();
    rxChannels[ROLL] = 1600;
    rxChannels[CONTROL_CHANNEL_COUNT] = 1900;
    cycles(5);

    // The switch invalidates the control channels only
    setFailsafeSwitch(true);
    cycle();
    EXPECT_FALSE(rxAreFlightChannelsValid());
    EXPECT_EQ(1600, rcInput[ROLL]);      // held for now

    cycles(31);
    EXPECT_EQ(1500, rcInput[ROLL]);
    EXPECT_EQ(1900, rcInput[CONTROL_CHANNEL_COUNT]);

    setFailsafeSwitch(false);
    cycle();
    EXPECT_TRUE(rxAreFlightChannelsValid());
    EXPECT_EQ(1600, rcInput[ROLL]);
}

TEST_F(RxTest, ValuesAreClamped)
{
    // Wider pulse window than the accepted output range
    rxConfigMutable()->rx_pulse_min = 500;
    rxConfigMutable()->rx_pulse_max = 2500;
    init();

    rxChannels[ROLL] = 600;
    rxChannels[PITCH] = 2400;
    cycle();

    EXPECT_EQ(PWM_PULSE_MIN, rcInput[ROLL]);
    EXPECT_EQ(PWM_PULSE_MAX, rcInput[PITCH]);
}

TEST_F(RxTest, LinkQualityAverage)
{
    // 16-sample moving average
    uint16_t lq = 0;
    for (int i = 0; i < 16; i++)
        lq = updateLinkQualitySamples(LINK_QUALITY_MAX_VALUE);
    EXPECT_EQ(LINK_QUALITY_MAX_VALUE, lq);

    for (int i = 0; i < 4; i++)
        lq = updateLinkQualitySamples(0);
    EXPECT_EQ(LINK_QUALITY_MAX_VALUE * 12 / 16, lq);
}
