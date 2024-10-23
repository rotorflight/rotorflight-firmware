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

extern "C" {
#include "drivers/sbus_output.h"
#include "io/serial.h"

extern uint16_t sbusOutChannel[SBUS_OUT_CHANNELS];
extern serialPort_t *sbusOutPort;
extern timeUs_t sbusOutLastTxTimeUs;
void sbusOutPrepareSbusFrame(sbusOutFrame_t *frame);
}

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "unittest_macros.h"

// In order to mock free functions, we have to replace them in linking stage
// (rather than normal polymorphism).
// Approach: 
//   1. We create a mock interface class for all need-to-mock free functions.
//   2. We create stub free functions to call the virtual functions in the mock object. 
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
} *g_mock = nullptr;

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

using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;

// Start testing
TEST(SBusOutInit, Memset) {
    MockInterface mock;
    g_mock = &mock;

    for (int i = 0; i < SBUS_OUT_CHANNELS; i++) {
        sbusOutChannel[i] = 0xff;
    }

    sbusOutInit();

    // Expectations:
    for (int i = 0; i < SBUS_OUT_CHANNELS; i++) {
        EXPECT_EQ(sbusOutChannel[i], 0);
    }
}
TEST(SBusOutInit, GoodPath) {
    StrictMock<MockInterface> mock;
    g_mock = &mock;
    constexpr serialPortIdentifier_e fake_identifier =
        (serialPortIdentifier_e)0x15;
    serialPortConfig_t fake_port_config = {};
    fake_port_config.identifier = fake_identifier;
    serialPort_t fake_port = {};
    EXPECT_CALL(mock, findSerialPortConfig(_))
        .WillOnce(Return(&fake_port_config));
    EXPECT_CALL(mock, openSerialPort(fake_identifier, _, _, _, _, _, _))
        .WillOnce(Return(&fake_port));

    sbusOutInit();

    EXPECT_EQ(sbusOutPort, &fake_port);
}

TEST(SBusOutInit, NoPortConfig) {
    StrictMock<MockInterface> mock;
    g_mock = &mock;

    EXPECT_CALL(mock, findSerialPortConfig(_)).WillOnce(Return(nullptr));

    sbusOutInit();

    // Expect not to call openSerialPort
}

TEST(SBusOutConfig, OutOfBound) {
    MockInterface mock;
    g_mock = &mock;

    sbusOutInit();
    sbusOutChannel_t channel;
    sbusOutConfig(&channel, 66);
    // Invalid channels are all reset to internal value 0.
    EXPECT_EQ(channel, 0);
    // This should not crash by AddressSanitizer.
    // However, AddressSanitizer isn't configured. This technically is not
    // testing anything.
    sbusOutSetOutput(&channel, 22);
}

class SBusOutTestBase : public ::testing::Test {
   public:
    void SetUp() {
        g_mock = &mock_;
        EXPECT_CALL(mock_, findSerialPortConfig(testing::_))
            .WillOnce(Return(nullptr));

        sbusOutInit();

        for (int i = 0; i < SBUS_OUT_CHANNELS; i++) {
            // sbusOutConfig uses 1-based channel number.
            sbusOutConfig(&channels_[i], i + 1);
        }
    }

    sbusOutChannel_t channels_[SBUS_OUT_CHANNELS];

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
    MockInterface mock_;
};

// Test param is: <channel, value>
class SBusOutChannelSweep
    : public SBusOutTestBase,
      public testing::WithParamInterface<std::tuple<int, int>> {};

TEST_P(SBusOutChannelSweep, FrameConstruct) {
    const uint8_t channel = std::get<0>(GetParam());
    const uint16_t value = std::get<1>(GetParam());

    // Set value through API.
    sbusOutSetOutput(&channels_[channel], value);

    // Generate frame
    union {
        sbusOutFrame_t frame;
        uint8_t bytes[26];
    } buffer;
    sbusOutPrepareSbusFrame(&buffer.frame);

    // Expectations:
    // Check the internal storage.
    EXPECT_EQ(sbusOutChannel[channel], value);

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

// Test channel 1 with all possible values.
INSTANTIATE_TEST_SUITE_P(ChannelOneValueSweep, SBusOutChannelSweep,
                         testing::Combine(testing::Values(0),
                                          testing::Range(0, (1 << 11) - 1)));

// Test all channels with a few extreme values (to save time).
INSTANTIATE_TEST_SUITE_P(FullScaleChannelSweep, SBusOutChannelSweep,
                         testing::Combine(testing::Range(0, 16),
                                          testing::Values(0, 4, (1 << 11) - 1)));

// Test all on-off channels.
INSTANTIATE_TEST_SUITE_P(OnOffChannelSweep, SBusOutChannelSweep,
                         testing::Combine(testing::Values(16, 17),
                                          testing::Values(0, 1)));

class SBusOutPWMToSBusTest : public SBusOutTestBase {};

TEST_F(SBusOutPWMToSBusTest, OutOfBoundValues) {
    uint16_t sbus;
    sbus = sbusOutPwmToSbus(&channels_[0], 0);
    EXPECT_EQ(sbus, 0);
    sbus = sbusOutPwmToSbus(&channels_[0], UINT16_MAX);
    EXPECT_EQ(sbus, (1 << 11) - 1);
    sbus = sbusOutPwmToSbus(&channels_[16], 0);
    EXPECT_EQ(sbus, 0);
    sbus = sbusOutPwmToSbus(&channels_[16], UINT16_MAX);
    EXPECT_EQ(sbus, 1);
}

// Test param = PWM value
class SBusOutPWMToSBusSweep : public SBusOutPWMToSBusTest,
                              public testing::WithParamInterface<int> {};

TEST_P(SBusOutPWMToSBusSweep, FullScaleChannel) {
    const uint16_t pwm = GetParam();
    uint16_t sbus = sbusOutPwmToSbus(&channels_[0], pwm);
    // sbus: [192, 1792]
    // pwm: [1000, 2000]
    float sbusf = (sbus - 192.0f) / (1792 - 192 + 1);
    float pwmf = (pwm - 1000.0f) / (2000 - 1000 + 1);
    const float resolution = 1.0f / (2000 - 1000 + 1);
    EXPECT_NEAR(sbusf, pwmf, resolution);
}

TEST_P(SBusOutPWMToSBusSweep, OnOffConversion) {
    const uint16_t pwm = GetParam();
    uint16_t sbus = sbusOutPwmToSbus(&channels_[16], pwm);
    // This is a DRY test but I don't have a better way.
    EXPECT_EQ(sbus, pwm > 1500 ? 1 : 0);
}

INSTANTIATE_TEST_SUITE_P(PWMConversionSweep, SBusOutPWMToSBusSweep,
                         testing::Range(1000, 2000));
