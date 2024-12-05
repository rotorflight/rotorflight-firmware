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

#include <stdint.h>
#include <string.h>

extern "C" {
#include "drivers/sbus_output.h"
#include "flight/mixer.h"
#include "io/serial.h"
#include "pg/sbus_output.h"

extern serialPort_t *sbusOutPort;
extern void sbusOutPrepareSbusFrame(sbusOutFrame_t *frame, uint16_t *channels);
extern void pgResetFn_sbusOutConfig(sbusOutConfig_t *config);

extern float sbusOutGetChannelValue(uint8_t channel);
extern uint16_t sbusOutConvertToSbus(uint8_t channel, float pwm);
}

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "unittest_macros.h"

// In order to mock free functions, we have to replace them in linking stage
// (rather than normal polymorphism).
// Approach:
//   1. We create a mock interface class for all need-to-mock free functions.
//   2. We create stub free functions to call the virtual functions in the mock
//   object.
// Because we create and desctruct mock object in each test:
//   3. we save it (globally) for those free functions to use.
class MockInterface {
  public:
    MOCK_METHOD(const serialPortConfig_t *, findSerialPortConfig,
                (serialPortFunction_e function), ());
    MOCK_METHOD(serialPort_t *, openSerialPort,
                (serialPortIdentifier_e identifier,
                 serialPortFunction_e function,
                 serialReceiveCallbackPtr rxCallback, void *rxCallbackData,
                 uint32_t baudRate, portMode_e mode, portOptions_e options),
                ());
    MOCK_METHOD(uint32_t, serialTxBytesFree, (const serialPort_t *instance),
                ());
    MOCK_METHOD(void, serialWriteBuf,
                (serialPort_t * instance, const uint8_t *data, int count), ());
    MOCK_METHOD(float, mixerGetOutput, (uint8_t index), ());
    MOCK_METHOD(uint16_t, getServoOutput, (uint8_t index), ());
    MOCK_METHOD(int16_t, getMotorOutput, (uint8_t index), ());
} *g_mock = nullptr;

extern "C" {
const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function) {
    return g_mock->findSerialPortConfig(function);
}

serialPort_t *openSerialPort(serialPortIdentifier_e identifier,
                             serialPortFunction_e function,
                             serialReceiveCallbackPtr rxCallback,
                             void *rxCallbackData, uint32_t baudRate,
                             portMode_e mode, portOptions_e options) {
    return g_mock->openSerialPort(identifier, function, rxCallback,
                                  rxCallbackData, baudRate, mode, options);
}

uint32_t serialTxBytesFree(const serialPort_t *instance) {
    return g_mock->serialTxBytesFree(instance);
}

void serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count) {
    g_mock->serialWriteBuf(instance, data, count);
}

float rcChannel[18];
float mixerGetOutput(uint8_t index) { return g_mock->mixerGetOutput(index); }
uint16_t getServoOutput(uint8_t index) { return g_mock->getServoOutput(index); }
int16_t getMotorOutput(uint8_t motor) { return g_mock->getMotorOutput(motor); }
}

using ::testing::_;
using ::testing::Invoke;
using ::testing::Return;
using ::testing::StrictMock;

void pgReset(void) {
    // We will use the real config variable
    extern const pgRegistry_t _sbusOutConfigRegistry;
    pgResetFn_sbusOutConfig(sbusOutConfigMutable());
}

// Start testing
TEST(SBusOutInit, ConfigReset) {
    // Reset config for testing
    pgReset();

    // Source Type (All SBUS_OUT_SOURCE_RX)
    for (int i = 0; i < 18; i++) {
        EXPECT_EQ(sbusOutConfig()->sourceType[i], SBUS_OUT_SOURCE_RX);
    }
    // Source Channel
    for (int i = 0; i < 18; i++) {
        EXPECT_EQ(sbusOutConfig()->sourceIndex[i], i);
    }
    // Low High
    for (int i = 0; i < 18; i++) {
        EXPECT_EQ(sbusOutConfig()->sourceRangeLow[i], 1000);
        EXPECT_EQ(sbusOutConfig()->sourceRangeHigh[i], 2000);
    }

    EXPECT_EQ(sbusOutConfig()->frameRate, 50);
}

TEST(SBusOutInit, GoodPath) {
    StrictMock<MockInterface> mock;
    g_mock = &mock;
    constexpr serialPortIdentifier_e fake_identifier =
        (serialPortIdentifier_e)0x15;
    serialPortConfig_t fake_port_config = {};
    fake_port_config.identifier = fake_identifier;
    serialPort_t fake_port = {};
    EXPECT_CALL(mock, findSerialPortConfig).WillOnce(Return(&fake_port_config));
    EXPECT_CALL(mock, openSerialPort(fake_identifier, _, _, _, _, _, _))
        .WillOnce(Return(&fake_port));

    sbusOutInit();

    EXPECT_EQ(sbusOutPort, &fake_port);

    EXPECT_TRUE(sbusOutIsEnabled());
}

TEST(SBusOutInit, NoPortConfig) {
    StrictMock<MockInterface> mock;
    g_mock = &mock;

    EXPECT_CALL(mock, findSerialPortConfig).WillOnce(Return(nullptr));
    // Expect not to call openSerialPort
    EXPECT_CALL(mock, openSerialPort).Times(0);

    sbusOutInit();

    // Update should be a no-op
    sbusOutUpdate(0);

    EXPECT_FALSE(sbusOutIsEnabled());
}

class SBusOutTestBase : public ::testing::Test {
  public:
    void SetUp() override {
        // Reset config for testing
        pgReset();

        // Reset rcChannel
        memset(rcChannel, 0, sizeof(rcChannel));

        // Call Init()
        g_mock = &mock_;
        EXPECT_CALL(mock_, findSerialPortConfig)
            .WillOnce(Return(&fake_port_config_));
        EXPECT_CALL(mock_, openSerialPort).WillOnce(Return(&fake_port_));

        sbusOutInit();
    }

    // Internal storage to help testing
    uint16_t channels_[SBUS_OUT_CHANNELS];

    // Helper function to extract bits.
    uint16_t GetBits(uint8_t *buffer, size_t begin_bit, size_t length) {
        // Read rounded byte (with extra bits)
        size_t begin_byte = begin_bit / 8;
        size_t ending_byte = (begin_bit + length) / 8;

        // We will store it to a uint32_t, which is sufficient for SBus (11
        // bits).
        EXPECT_LE(ending_byte - begin_byte + 1, sizeof(uint32_t));

        uint32_t value = 0;
        for (size_t i = ending_byte; i >= begin_byte; i--) {
            value <<= 8;
            value += buffer[i];
        }

        // Remove extra bits
        value >>= (begin_bit - begin_byte * 8);
        value &= (1 << length) - 1;

        return (uint16_t)value;
    }
    serialPortConfig_t fake_port_config_ = {};
    serialPort_t fake_port_ = {};
    StrictMock<MockInterface> mock_;
};

// Test param:
// <int, int> -> sbus_channel, source_index
// Different source_type is expanded in different TEST_P
class SBusOutSourceMapping
    : public SBusOutTestBase,
      public testing::WithParamInterface<std::tuple<int, int>> {};

TEST_P(SBusOutSourceMapping, SourceRX) {
    uint8_t sbus_channel = std::get<0>(GetParam());
    uint8_t source_index = std::get<1>(GetParam());
    sbusOutConfigMutable()->sourceType[sbus_channel] = SBUS_OUT_SOURCE_RX;
    sbusOutConfigMutable()->sourceIndex[sbus_channel] = source_index;

    constexpr float kFakeValue = 1234.56f;
    rcChannel[source_index] = kFakeValue;

    EXPECT_EQ(sbusOutGetChannelValue(sbus_channel), kFakeValue);
}

TEST_P(SBusOutSourceMapping, SourceMixer) {
    uint8_t sbus_channel = std::get<0>(GetParam());
    uint8_t source_index = std::get<1>(GetParam());
    sbusOutConfigMutable()->sourceType[sbus_channel] = SBUS_OUT_SOURCE_MIXER;
    sbusOutConfigMutable()->sourceIndex[sbus_channel] = source_index;
    sbusOutConfigMutable()->sourceRangeLow[sbus_channel] = -1000;
    sbusOutConfigMutable()->sourceRangeHigh[sbus_channel] = 1000;

    constexpr float kFakeValue = -0.1234f;

    if (source_index < MIXER_OUTPUT_COUNT) {
        EXPECT_CALL(mock_, mixerGetOutput(source_index))
            .WillOnce(Return(kFakeValue));
        EXPECT_EQ(sbusOutGetChannelValue(sbus_channel), 1000 * kFakeValue);
    } else {
        EXPECT_EQ(sbusOutGetChannelValue(sbus_channel), 0);
    }
}

TEST_P(SBusOutSourceMapping, SourceServo) {
    uint8_t sbus_channel = std::get<0>(GetParam());
    uint8_t source_index = std::get<1>(GetParam());
    sbusOutConfigMutable()->sourceType[sbus_channel] = SBUS_OUT_SOURCE_SERVO;
    sbusOutConfigMutable()->sourceIndex[sbus_channel] = source_index;

    constexpr uint16_t kFakeValue = 14285;

    if (source_index < MAX_SUPPORTED_SERVOS) {
        EXPECT_CALL(mock_, getServoOutput(source_index))
            .WillOnce(Return(kFakeValue));
        EXPECT_EQ(sbusOutGetChannelValue(sbus_channel), kFakeValue);
    } else {
        EXPECT_EQ(sbusOutGetChannelValue(sbus_channel), 0);
    }
}

TEST_P(SBusOutSourceMapping, SourceMotor) {
    uint8_t sbus_channel = std::get<0>(GetParam());
    uint8_t source_index = std::get<1>(GetParam());
    sbusOutConfigMutable()->sourceType[sbus_channel] = SBUS_OUT_SOURCE_MOTOR;
    sbusOutConfigMutable()->sourceIndex[sbus_channel] = source_index;

    constexpr uint16_t kFakeValue = 14285;

    if (source_index < MAX_SUPPORTED_MOTORS) {
        EXPECT_CALL(mock_, getMotorOutput(source_index))
            .WillOnce(Return(kFakeValue));
        EXPECT_EQ(sbusOutGetChannelValue(sbus_channel), kFakeValue);
    } else {
        EXPECT_EQ(sbusOutGetChannelValue(sbus_channel), 0);
    }
}

// Test 18x18 combinations. This will also include out of bound indexes.
INSTANTIATE_TEST_SUITE_P(SourceMappingSweep, SBusOutSourceMapping,
                         testing::Combine(testing::Range(0, 18),
                                          testing::Range(0, 18)));

// Test all channels in the frame construct
// Test param is: <channel, value>
class SBusOutFrameSweep
    : public SBusOutTestBase,
      public testing::WithParamInterface<std::tuple<int, int>> {};

TEST_P(SBusOutFrameSweep, FrameConstruct) {
    const uint8_t channel = std::get<0>(GetParam());
    const uint16_t value = std::get<1>(GetParam());

    // Set value through API.
    channels_[channel] = value;

    // Generate frame
    union {
        sbusOutFrame_t frame;
        uint8_t bytes[25];
    } buffer;
    sbusOutPrepareSbusFrame(&buffer.frame, channels_);

    // Expectations:
    // Check the value in the frame
    size_t offset;
    size_t length;
    if (channel < 16) {
        // Full resolution channels:
        offset = channel * 11 + 8;
        length = 11;
    } else {
        // On-off channels:
        //  channel 16 = bit 23*8
        //  channel 17 = bit 23*8+1
        EXPECT_GE(channel, 16);
        EXPECT_LE(channel, 17);
        offset = channel == 16 ? 23 * 8 : 23 * 8 + 1;
        length = 1;
    }

    uint16_t value_in_frame = GetBits(buffer.bytes, offset, length);
    EXPECT_EQ(value, value_in_frame);
}

// Test channel 0 with all possible values.
INSTANTIATE_TEST_SUITE_P(ChannelOneValueSweep, SBusOutFrameSweep,
                         testing::Combine(testing::Values(0),
                                          testing::Range(0, (1 << 11) - 1)));

// Test all channels with a few extreme values (to save time).
INSTANTIATE_TEST_SUITE_P(FullScale, SBusOutFrameSweep,
                         testing::Combine(testing::Range(0, 16),
                                          testing::Values(0, 4,
                                                          (1 << 11) - 1)));

// Test all on-off channels.
INSTANTIATE_TEST_SUITE_P(OnOff, SBusOutFrameSweep,
                         testing::Combine(testing::Values(16, 17),
                                          testing::Values(0, 1)));

class SBusOutPWMToSBusTest : public SBusOutTestBase {};

TEST_F(SBusOutPWMToSBusTest, OutOfBoundValues) {
    uint16_t sbus;
    sbus = sbusOutConvertToSbus(0, -1);
    EXPECT_EQ(sbus, 0);
    sbus = sbusOutConvertToSbus(0, 4000);
    EXPECT_EQ(sbus, (1 << 11) - 1);
    sbus = sbusOutConvertToSbus(16, -1);
    EXPECT_EQ(sbus, 0);
    sbus = sbusOutConvertToSbus(16, 4000);
    EXPECT_EQ(sbus, 1);
}

// When config low/high is reversed (low>high)
TEST_F(SBusOutPWMToSBusTest, ReversedOutOfBoundValues) {
    // Reversed low/high
    sbusOutConfigMutable()->sourceRangeLow[4] = 500;
    sbusOutConfigMutable()->sourceRangeHigh[4] = -500;
    sbusOutConfigMutable()->sourceRangeLow[16] = 500;
    sbusOutConfigMutable()->sourceRangeHigh[16] = -500;

    uint16_t sbus;
    sbus = sbusOutConvertToSbus(4, 4000);
    EXPECT_EQ(sbus, 0);
    sbus = sbusOutConvertToSbus(4, -4000);
    EXPECT_EQ(sbus, (1 << 11) - 1);
    sbus = sbusOutConvertToSbus(16, 4000);
    EXPECT_EQ(sbus, 0);
    sbus = sbusOutConvertToSbus(16, -4000);
    EXPECT_EQ(sbus, 1);
}

// Test PWM [1000, 2000] convert to SBus value
// Test param = PWM value
class SBusOutPWMToSBusSweep : public SBusOutPWMToSBusTest,
                              public testing::WithParamInterface<int> {
  public:
    // Second source of values
    // Convert value to a float between [0, 1]
    static float ValueMinMaxToFloat(int value, int min, int max) {
        return static_cast<float>(value - min) / (max - min);
    }
    static float SBusToFloat(int sbus) {
        return ValueMinMaxToFloat(sbus, 192, 1792);
    }
    static float Resolution(int min, int max) {
        return 1.0f / (max - min);
    }
};

TEST_P(SBusOutPWMToSBusSweep, FullScaleChannel) {
    const uint16_t pwm = GetParam();
    uint16_t sbus = sbusOutConvertToSbus(3, pwm);

    // Validate:
    float sbusf = SBusToFloat(sbus);
    // pwm: [1000, 2000]
    float pwmf = ValueMinMaxToFloat(pwm, 1000, 2000);
    const float resolution = Resolution(1000, 2000);
    EXPECT_NEAR(sbusf, pwmf, resolution);
}

TEST_P(SBusOutPWMToSBusSweep, OnOffConversion) {
    const uint16_t pwm = GetParam();
    uint16_t sbus = sbusOutConvertToSbus(17, pwm);
    // Skip 1500 (mid point)
    if (pwm == 1500)
        GTEST_SKIP();

    EXPECT_EQ(sbus, pwm > 1500 ? 1 : 0);
}

INSTANTIATE_TEST_SUITE_P(PWMConversionSweep, SBusOutPWMToSBusSweep,
                         testing::Range(1000, 2001));

// Test Narrow PWM [500, 1000] convert to SBus value
// Test param = PWM value
class SBusOutNarrowbandPWMToSBusSweep : public SBusOutPWMToSBusSweep {};

TEST_P(SBusOutNarrowbandPWMToSBusSweep, FullScaleChannel) {
    sbusOutConfigMutable()->sourceRangeLow[4] = 500;
    sbusOutConfigMutable()->sourceRangeHigh[4] = 1000;

    const uint16_t pwm = GetParam();
    uint16_t sbus = sbusOutConvertToSbus(4, pwm);

    // Validate:
    float sbusf = SBusToFloat(sbus);
    // pwm: [500, 1000]
    float pwmf = ValueMinMaxToFloat(pwm, 500, 1000);
    const float resolution = Resolution(500, 1000);
    EXPECT_NEAR(sbusf, pwmf, resolution);
}

TEST_P(SBusOutNarrowbandPWMToSBusSweep, OnOffConversion) {
    sbusOutConfigMutable()->sourceRangeLow[16] = 500;
    sbusOutConfigMutable()->sourceRangeHigh[16] = 1000;

    const uint16_t pwm = GetParam();
    uint16_t sbus = sbusOutConvertToSbus(16, pwm);
    // Skip 750 (mid point)
    if (pwm == 750)
        GTEST_SKIP();

    EXPECT_EQ(sbus, pwm > 750 ? 1 : 0);
}

INSTANTIATE_TEST_SUITE_P(NarrowbandPWMConversionSweep,
                         SBusOutNarrowbandPWMToSBusSweep,
                         testing::Range(500, 1001));

// Test Mixer value [-1000, 1000] convert to SBus value
// Test param = Mixer rule value
class SBusOutMixerValueToSBusSweep : public SBusOutPWMToSBusSweep {};

TEST_P(SBusOutMixerValueToSBusSweep, FullScaleChannel) {
    sbusOutConfigMutable()->sourceRangeLow[4] = -1000;
    sbusOutConfigMutable()->sourceRangeHigh[4] = 1000;

    const int16_t value = GetParam();
    uint16_t sbus = sbusOutConvertToSbus(4, value);

    // Validate:
    float sbusf = SBusToFloat(sbus);
    // value: [-1000, 1000]
    float valuef = ValueMinMaxToFloat(value, -1000, 1000);
    const float resolution = Resolution(-1000, 1000);
    EXPECT_NEAR(sbusf, valuef, resolution);
}

TEST_P(SBusOutMixerValueToSBusSweep, OnOffConversion) {
    sbusOutConfigMutable()->sourceRangeLow[16] = -1000;
    sbusOutConfigMutable()->sourceRangeHigh[16] = 1000;

    const int16_t value = GetParam();
    uint16_t sbus = sbusOutConvertToSbus(16, value);

    EXPECT_EQ(sbus, value > 0 ? 1 : 0);
}

INSTANTIATE_TEST_SUITE_P(MixerValueConversionSweep,
                         SBusOutMixerValueToSBusSweep,
                         testing::Range(-1000, 1001));

// Test SBus conversion when low/high is reversed in the config
// Test param = value between [-500, 1000] (more generic)
class SBusOutToSBusReversedSweep : public SBusOutPWMToSBusSweep {};

TEST_P(SBusOutToSBusReversedSweep, FullScaleChannel) {
    // Reversed low/high
    sbusOutConfigMutable()->sourceRangeLow[4] = 1000;
    sbusOutConfigMutable()->sourceRangeHigh[4] = -500;

    const int16_t value = GetParam();
    uint16_t sbus = sbusOutConvertToSbus(4, value);

    float sbusf = SBusToFloat(sbus);
    // value: [1000, -500]
    float valuef = ValueMinMaxToFloat(value, 1000, -500);
    const float resolution = Resolution(-500, 1000);
    EXPECT_NEAR(sbusf, valuef, resolution);
}

TEST_P(SBusOutToSBusReversedSweep, OnOffConversion) {
    sbusOutConfigMutable()->sourceRangeLow[16] = 1000;
    sbusOutConfigMutable()->sourceRangeHigh[16] = -500;

    const int16_t value = GetParam();
    uint16_t sbus = sbusOutConvertToSbus(16, value);

    const int16_t midpoint = (1000 + -500)/2;
    if ((1000 + -500)%2 == 0 && value == midpoint)
        GTEST_SKIP();

    float valuef = ValueMinMaxToFloat(value, 1000, -500);
    EXPECT_EQ(sbus, valuef > 0.5 ? 1 : 0);
}

INSTANTIATE_TEST_SUITE_P(ReversedMinMaxConversionSweep,
                         SBusOutToSBusReversedSweep,
                         testing::Range(-500, 1001));

class SBusOutSerialTest : public SBusOutTestBase {};

TEST_F(SBusOutSerialTest, NoFreeBuffer) {
    EXPECT_CALL(mock_, serialTxBytesFree(&fake_port_)).WillOnce(Return(25));
    // Expect no-op
    EXPECT_CALL(mock_, serialWriteBuf).Times(0);

    sbusOutUpdate(1000000);
}

TEST_F(SBusOutSerialTest, GoodUpdate) {
    EXPECT_CALL(mock_, serialTxBytesFree(&fake_port_))
        .Times(2)
        .WillRepeatedly(Return(50));
    EXPECT_CALL(mock_, serialWriteBuf(&fake_port_, _, _)).Times(2);

    sbusOutUpdate(1000000);

    sbusOutUpdate(2000000);
}

TEST_F(SBusOutSerialTest, SerialFormat) {
    for (int i = 0; i < 16; ++i) {
        rcChannel[i] = 1000 + i * 2;
    }
    rcChannel[16] = 1000;
    rcChannel[17] = 2000;
    // This is a matching buffer verified externally
    static const uint8_t expected_data[25] = {
        0x0f, 0xc0, 0x18, 0x86, 0x31, 0x94, 0xd1, 0x0c, 0x68,
        0x4c, 0xc3, 0x1a, 0xda, 0xe8, 0x06, 0x38, 0xc6, 0x61,
        0x0e, 0x75, 0xb4, 0x03, 0x1e, 0x02, 0x00,
    };

    EXPECT_CALL(mock_, serialTxBytesFree(&fake_port_)).WillOnce(Return(50));
    EXPECT_CALL(mock_, serialWriteBuf(&fake_port_, _, _))
        .WillOnce(Invoke([&](serialPort_t *, const uint8_t *data, int count) {
            EXPECT_EQ(count, 25);
            EXPECT_EQ(memcmp((const void *)data, (const void *)expected_data,
                             (size_t)count),
                      0);
        }));

    sbusOutUpdate(0);
}