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

#include "platform.h"
#include "target.h"
#include "drivers/barometer/barometer.h"
#include "drivers/bus.h"

void bmp581Calculate(int32_t *pressure, int32_t *temperature);

extern uint32_t bmp581_up;
extern uint32_t bmp581_ut;
}


#include "unittest_macros.h"
#include "gtest/gtest.h"


TEST(baroBmp581Test, TestBmp581CalculateNominal)
{
    // given
    int32_t pressure, temperature;
    
    // BMP581 provides compensated values directly
    // Temperature: 24-bit signed value, divide by 2^16 to get °C
    // Pressure: 24-bit unsigned value, divide by 2^6 to get Pa
    
    // Test case: 24.60°C and 99645 Pa
    // Temperature raw: 24.60 * 65536 = 1612545.6 approx. 1612545
    bmp581_ut = 1612545;
    
    // Pressure raw: 99645 * 64 = 6377280
    bmp581_up = 6377280;

    // when
    bmp581Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(99645, pressure); // 100653 Pa
    EXPECT_EQ(2460, temperature); // 25.08 degC (in 0.01°C units)
}

TEST(baroBmp581Test, TestBmp581CalculateZeroTemp)
{
    // given
    int32_t pressure, temperature;
    
    // Test case: 0°C and 101325 Pa (standard sea level pressure)
    // Temperature raw: 0 * 65536 = 0
    bmp581_ut = 0;
    
    // Pressure raw: 101325 * 64 = 6484800 (0x62EBC0)
    bmp581_up = 6484800;

    // when
    bmp581Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(101325, pressure); // 101325 Pa
    EXPECT_EQ(0, temperature); // 0.00 degC
}

TEST(baroBmp581Test, TestBmp581CalculateNegativeTemp)
{
    // given
    int32_t pressure, temperature;
    
    // Test case: -10.5°C and 90000 Pa
    // Temperature raw: -10.5 * 65536 = -688128
    // The driver's bmp581GetUP() function performs sign extension from 24-bit to 32-bit
    // So we store the already sign-extended value
    bmp581_ut = (uint32_t)(-688128); // Sign-extended negative value
    
    // Pressure raw: 90000 * 64 = 5760000 (0x57E400)
    bmp581_up = 5760000;

    // when
    bmp581Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(90000, pressure); // 90000 Pa
    EXPECT_EQ(-1050, temperature); // -10.50 degC (in 0.01°C units)
}

TEST(baroBmp581Test, TestBmp581CalculateHighPressure)
{
    // given
    int32_t pressure, temperature;
    
    // Test case: 20°C and 120000 Pa (high pressure)
    // Temperature raw: 20 * 65536 = 1310720
    bmp581_ut = 1310720;
    
    // Pressure raw: 120000 * 64 = 7680000
    bmp581_up = 7680000;

    // when
    bmp581Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(120000, pressure); // 120000 Pa
    EXPECT_EQ(2000, temperature); // 20.00 degC
}

TEST(baroBmp581Test, TestBmp581CalculateLowPressure)
{
    // given
    int32_t pressure, temperature;
    
    // Test case: 15°C and 50000 Pa (low pressure, high altitude)
    // Temperature raw: 15 * 65536 = 983040
    bmp581_ut = 983040;
    
    // Pressure raw: 50000 * 64 = 3200000
    bmp581_up = 3200000;

    // when
    bmp581Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(50000, pressure); // 50000 Pa
    EXPECT_EQ(1500, temperature); // 15.00 degC
}

TEST(baroBmp581Test, TestBmp581CalculateFractionalTemp)
{
    // given
    int32_t pressure, temperature;
    
    // Test case: 23.45°C and 98765 Pa
    // Temperature raw: 23.45 * 65536 = 1536870.4 approx. 1536870
    bmp581_ut = 1536870;
    
    // Pressure raw: 98765 * 64 = 6320960 (0x6070C0)
    bmp581_up = 6320960;

    // when
    bmp581Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(98765, pressure); // 98765 Pa
    EXPECT_EQ(2345, temperature); // 23.45 degC (in 0.01°C units)
}

// STUBS

extern "C" {

void delay(uint32_t) {}
bool busBusy(const extDevice_t*, bool*) {return false;}
bool busReadRegisterBuffer(const extDevice_t*, uint8_t, uint8_t*, uint8_t) {return true;}
bool busReadRegisterBufferStart(const extDevice_t*, uint8_t, uint8_t*, uint8_t) {return true;}
bool busWriteRegister(const extDevice_t*, uint8_t, uint8_t) {return true;}
bool busWriteRegisterStart(const extDevice_t*, uint8_t, uint8_t) {return true;}
void busDeviceRegister(const extDevice_t*) {}

uint16_t spiCalculateDivider() {
    return 2;
}

void spiSetClkDivisor(void) {
}

void spiPreinitByIO(void) {
}

void IOConfigGPIO(void) {
}

void IOHi(void) {
}

void IOInit(void) {
}

void IORelease(void) {
}

void EXTIHandlerInit(extiCallbackRec_t*, void (*)(extiCallbackRec_t*)) {
}

void EXTIConfig(IO_t, extiCallbackRec_t*, int, ioConfig_t, extiTrigger_t) {
}

void EXTIEnable(IO_t) {
}

IO_t IOGetByTag(ioTag_t) {
    return (IO_t)0;
}

void delayMicroseconds(uint32_t) {
}

}
