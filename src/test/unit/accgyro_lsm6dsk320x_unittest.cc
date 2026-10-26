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

#include <algorithm>
#include <cstring>
#include <utility>
#include <vector>

extern "C" {
#include "platform.h"
#include "drivers/accgyro/accgyro_spi_lsm6dsk320x.h"
#include "drivers/accgyro/gyro_sync.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "pg/accel.h"
}

#include "gtest/gtest.h"

// Existing persisted and MSP hardware IDs must not move when adding a sensor.
static_assert(GYRO_BMI323 == 20 && GYRO_FAKE == 21 && GYRO_LSM6DSK320X == 22);
static_assert(ACC_BMI323 == 21 && ACC_FAKE == 22 && ACC_LSM6DSK320X == 23);

static uint8_t registers[128];
static std::vector<std::pair<uint8_t, uint8_t>> writes;
static unsigned reset_delay_ms;
static unsigned exti_init_count;
static unsigned failure_mode_count;
static unsigned sequence_count;
static unsigned wait_count;
static uint32_t spi_frequency;
static bool dma_available;

class Lsm6dsk320x : public ::testing::Test {
protected:
    gyroDev_t gyro = {};
    accDev_t acc = {};
    alignas(4) uint8_t tx_buffer[16] = {};
    alignas(4) uint8_t rx_buffer[16] = {};

    void SetUp() override
    {
        memset(registers, 0, sizeof(registers));
        writes.clear();
        reset_delay_ms = exti_init_count = failure_mode_count = sequence_count = wait_count = 0;
        spi_frequency = 0;
        dma_available = false;
        gyro.dev.txBuf = tx_buffer;
        gyro.dev.rxBuf = rx_buffer;
        // The reset verifies the chip identity, so the mock has to answer as a LSM6DSK320X.
        registers[0x0f] = LSM6DSK320X_WHO_AM_I_CONST;
        gyro.mpuDetectionResult.sensor = LSM6DSK320X_SPI;
        acc.mpuDetectionResult.sensor = LSM6DSK320X_SPI;
        acc.gyro = &gyro;
        ASSERT_TRUE(lsm6dsk320xSpiGyroDetect(&gyro));
        ASSERT_TRUE(lsm6dsk320xSpiAccDetect(&acc));
    }

    void setWord(uint8_t address, int16_t value)
    {
        registers[address] = (uint16_t)value & 0xff;
        registers[address + 1] = (uint16_t)value >> 8;
    }

    void setSample()
    {
        setWord(0x22, 0x1234);
        setWord(0x24, -12345);
        setWord(0x26, -32768);
        setWord(0x28, 2048);
        setWord(0x2a, -2048);
        setWord(0x2c, 32767);
    }

    void expectSample()
    {
        EXPECT_EQ(0x1234, gyro.gyroADCRaw[X]);
        EXPECT_EQ(-12345, gyro.gyroADCRaw[Y]);
        EXPECT_EQ(-32768, gyro.gyroADCRaw[Z]);
        EXPECT_EQ(2048, acc.ADCRaw[X]);
        EXPECT_EQ(-2048, acc.ADCRaw[Y]);
        EXPECT_EQ(32767, acc.ADCRaw[Z]);
    }
};

TEST_F(Lsm6dsk320x, DetectsOnlyMatchingChip)
{
    for (unsigned id = 0; id <= 255; id++) {
        registers[0x0f] = id;
        EXPECT_EQ(id == LSM6DSK320X_WHO_AM_I_CONST ? LSM6DSK320X_SPI : MPU_NONE, lsm6dsk320xSpiDetect(&gyro.dev));
    }
    EXPECT_TRUE(writes.empty());
    gyro.mpuDetectionResult.sensor = LSM6DSO_SPI;
    acc.mpuDetectionResult.sensor = LSM6DSO_SPI;
    EXPECT_FALSE(lsm6dsk320xSpiGyroDetect(&gyro));
    EXPECT_FALSE(lsm6dsk320xSpiAccDetect(&acc));
}

TEST_F(Lsm6dsk320x, InitializesHighAccuracyRatesScalesAndInterrupt)
{
    gyroSetSampleRate(&gyro);
    gyro.initFn(&gyro);
    acc.initFn(&acc);
    ASSERT_FALSE(writes.empty());
    // An MCU-only reset can leave the IMU running, so both sensors are powered down before
    // SW_RESET, and the reset is then polled until it reports complete.
    EXPECT_EQ(1, std::count(writes.begin(), writes.end(), std::make_pair(uint8_t(0x10), uint8_t(0x00))));
    EXPECT_EQ(1, std::count(writes.begin(), writes.end(), std::make_pair(uint8_t(0x11), uint8_t(0x00))));
    EXPECT_EQ(1, std::count(writes.begin(), writes.end(), std::make_pair(uint8_t(0x12), uint8_t(0x01))));
    EXPECT_EQ(10u, reset_delay_ms);
    EXPECT_EQ(10000000u, spi_frequency);
    EXPECT_EQ(0x44, registers[0x12]); // BDU and auto-increment
    EXPECT_EQ(0x01, registers[0x62]); // high-accuracy ODR mode 1
    EXPECT_EQ(0x19, registers[0x10]); // high-accuracy accelerometer, 1 kHz
    EXPECT_EQ(0x1c, registers[0x11]); // high-accuracy gyro, 8 kHz
    EXPECT_EQ(0x0c, registers[0x15]); // 2000 dps, normal LPF, mandatory bit 3 set
    EXPECT_EQ(0x03, registers[0x17]); // 16 g, LPF2 ODR/4
    EXPECT_EQ(0x01, registers[0x16]); // gyro LPF1 enabled
    EXPECT_EQ(0x08, registers[0x18]); // accelerometer LPF2 enabled
    EXPECT_EQ(0x02, registers[0x13]); // pulsed DRDY
    // Operating mode is programmed while powered down, and the ODR is only added afterwards
    // with those mode bits retained: writing both at once yields 7.68 kHz / 960 Hz.
    EXPECT_EQ(1, std::count(writes.begin(), writes.end(), std::make_pair(uint8_t(0x10), uint8_t(0x10))));
    EXPECT_EQ(1, std::count(writes.begin(), writes.end(), std::make_pair(uint8_t(0x11), uint8_t(0x10))));
    EXPECT_EQ(std::make_pair(uint8_t(0x11), uint8_t(0x1c)), writes[writes.size() - 2]); // gyro first
    EXPECT_EQ(std::make_pair(uint8_t(0x10), uint8_t(0x19)), writes.back());
    EXPECT_EQ(1u, exti_init_count);
    EXPECT_FLOAT_EQ(0.070f, gyro.scale);
    EXPECT_EQ(2048, acc.acc_1G);
    EXPECT_EQ(GYRO_RATE_8_kHz, gyro.gyroRateKHz);
    EXPECT_EQ(8000, gyro.gyroSampleRateHz);
    EXPECT_EQ(1000, gyro.accSampleRateHz);
    EXPECT_EQ(0, gyro.mpuDividerDrops);
    EXPECT_EQ(0x22, gyro.gyroDataReg);
    EXPECT_EQ(0x28, gyro.accDataReg);
}

TEST_F(Lsm6dsk320x, ReportsInitFailureWhenTheResetNeverCompletes)
{
    registers[0x0f] = 0x70; // an unsupported LSM6DSV16X: the reset verification cannot pass
    gyro.initFn(&gyro);
    EXPECT_EQ(1u, failure_mode_count);
    EXPECT_EQ(0x00, registers[0x15]); // the configuration was never applied
    EXPECT_EQ(0u, exti_init_count);
}

TEST_F(Lsm6dsk320x, AppliesPerDeviceHardwareFilter)
{
    const uint8_t expected[] = {0x0c, 0x2c, 0x1c,
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
        0x3c,
#endif
    };
    for (unsigned filter = 0; filter < GYRO_HARDWARE_LPF_COUNT; filter++) {
        gyro.hardware_lpf = filter;
        gyro.initFn(&gyro);
        EXPECT_EQ(expected[filter], registers[0x15]);
        EXPECT_EQ(0x1c, registers[0x11]);
    }
}

TEST_F(Lsm6dsk320x, ReadsSignedLittleEndianSamplesWithoutDma)
{
    setSample();
    for (auto mode : {GYRO_EXTI_NO_INT, GYRO_EXTI_INT}) {
        gyro.gyroModeSPI = mode;
        ASSERT_TRUE(gyro.readFn(&gyro));
        ASSERT_TRUE(acc.readFn(&acc));
        expectSample();
    }
    EXPECT_EQ(4u, sequence_count);
    EXPECT_EQ(sequence_count, wait_count);
}

TEST_F(Lsm6dsk320x, FallsBackWhenInterruptsAreMissing)
{
    dma_available = true;
    gyro.detectedEXTI = 1000;
    // The first call only configures acquisition, so it reports that no sample was read
    ASSERT_FALSE(gyro.readFn(&gyro));
    EXPECT_EQ(GYRO_EXTI_NO_INT, gyro.gyroModeSPI);
    EXPECT_EQ(0u, sequence_count);
    setSample();
    gyro.readFn(&gyro);
    acc.readFn(&acc);
    expectSample();
}

TEST_F(Lsm6dsk320x, FallsBackWhenDmaIsUnavailable)
{
    gyro.detectedEXTI = 1001;
    ASSERT_FALSE(gyro.readFn(&gyro));
    EXPECT_EQ(GYRO_EXTI_INT, gyro.gyroModeSPI);
}

#ifdef USE_DMA
TEST_F(Lsm6dsk320x, ReadsCombinedDmaFrameAtCorrectOffsets)
{
    dma_available = true;
    gyro.detectedEXTI = 1001;
    ASSERT_FALSE(gyro.readFn(&gyro));
    ASSERT_EQ(GYRO_EXTI_INT_DMA, gyro.gyroModeSPI);
    EXPECT_EQ(0xa2, tx_buffer[0]);
    EXPECT_EQ(13, gyro.segments[0].len);
    EXPECT_EQ(&rx_buffer[1], gyro.segments[0].u.buffers.rxData);
    EXPECT_EQ(tx_buffer, gyro.segments[0].u.buffers.txData);
    EXPECT_TRUE(gyro.segments[0].negateCS);
    EXPECT_EQ(mpuIntcallback, gyro.segments[0].callback);
    EXPECT_EQ(0, gyro.segments[1].len);
    setSample();
    spiSequence(&gyro.dev, gyro.segments); // simulate the EXTI-triggered transfer
    ASSERT_TRUE(gyro.readFn(&gyro));
    ASSERT_TRUE(acc.readFn(&acc));
    expectSample();
    EXPECT_EQ(1u, sequence_count);
    EXPECT_EQ(0u, wait_count); // consume the completed DMA buffer without a new transaction
}
#else
TEST_F(Lsm6dsk320x, FallsBackWhenDmaIsNotCompiled)
{
    dma_available = true;
    gyro.detectedEXTI = 1001;
    ASSERT_FALSE(gyro.readFn(&gyro));
    EXPECT_EQ(GYRO_EXTI_INT, gyro.gyroModeSPI);
}
#endif

extern "C" {
uint8_t spiReadRegMsk(const extDevice_t *, uint8_t reg) { return registers[reg]; }
void spiWriteReg(const extDevice_t *, uint8_t reg, uint8_t value)
{
    writes.emplace_back(reg, value);
    registers[reg] = value;
    if (reg == 0x12 && (value & 0x01)) {
        // SW_RESET is self-clearing and the reset leaves BDU set; modelling that keeps the
        // driver's reset poll from timing out.
        registers[reg] = 0x40;
    }
}
uint16_t spiCalculateDivider(uint32_t frequency) { spi_frequency = frequency; return 16; }
void spiSetClkDivisor(const extDevice_t *, uint16_t) {}
void delay(timeMs_t ms) { reset_delay_ms += ms; }
void delayMicroseconds(uint32_t) {}
void mpuGyroInit(gyroDev_t *) { exti_init_count++; }
void failureMode(failureMode_e) { failure_mode_count++; }
bool spiUseDMA(const extDevice_t *) { return dma_available; }
busStatus_e mpuIntcallback(uint32_t) { return BUS_READY; }
void spiWait(const extDevice_t *) { wait_count++; }
void spiSequence(const extDevice_t *, busSegment_t *segments)
{
    sequence_count++;
    ASSERT_TRUE(segments[0].u.buffers.txData[0] & 0x80);
    const uint8_t address = segments[0].u.buffers.txData[0] & 0x7f;
    ASSERT_LE(address + segments[0].len - 1, sizeof(registers));
    segments[0].u.buffers.rxData[0] = 0xa5; // command-phase byte must never become sensor data
    memcpy(segments[0].u.buffers.rxData + 1, &registers[address], segments[0].len - 1);
}
}
