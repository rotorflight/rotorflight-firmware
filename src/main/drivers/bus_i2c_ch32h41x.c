/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * CH32H417 I2C driver for Rotorflight
 * Provides hardware I2C using CH32H417's I2C peripheral
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(SOFT_I2C)

#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"

// Clock period in us during unstick transfer
#define UNSTICK_CLK_US 10
// Allow 500us for clock stretch to complete during unstick
#define UNSTICK_CLK_STRETCH (500 / UNSTICK_CLK_US)

// I2C timeout
#define I2C_DEFAULT_TIMEOUT 10000

static void i2cUnstick(IO_t scl, IO_t sda);

// CH32H417 I2C hardware definitions
// Alternate functions from CH32H417 datasheet Table: I2C1_SCL PB6(AF4)/PB8(AF4),
// I2C2_SCL PC0(AF9)/PB10(AF4), I2C3_SCL PA8(AF4)/PA14(AF7),
// I2C4_SCL PD12(AF4)/PF12(AF2)/PB6(AF6)/PB8(AF6) etc.
// Pins are set to NONE in target.h and configured at runtime via bus_i2c_config.c
const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = I2C1,
        .sclPins = {
            I2CPINDEF(PB6, GPIO_AF4),
            I2CPINDEF(PB8, GPIO_AF4),
        },
        .sdaPins = {
            I2CPINDEF(PB7, GPIO_AF4),
            I2CPINDEF(PB9, GPIO_AF4),
        },
        .rcc = RCC_HB1(I2C1),
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = I2C2,
        .sclPins = {
            I2CPINDEF(PB10, GPIO_AF4),
            I2CPINDEF(PC0, GPIO_AF9),
        },
        .sdaPins = {
            I2CPINDEF(PB11, GPIO_AF4),
            I2CPINDEF(PC1, GPIO_AF9),
        },
        .rcc = RCC_HB1(I2C2),
    },
#endif
#ifdef USE_I2C_DEVICE_3
    {
        .device = I2CDEV_3,
        .reg = I2C3,
        .sclPins = {
            I2CPINDEF(PA8, GPIO_AF4),
        },
        .sdaPins = {
            I2CPINDEF(PC9, GPIO_AF4),
        },
        .rcc = RCC_HB1(I2C3),
    },
#endif
#ifdef USE_I2C_DEVICE_4
    {
        .device = I2CDEV_4,
        .reg = I2C4,
        .sclPins = {
            I2CPINDEF(PD12, GPIO_AF4),
            I2CPINDEF(PB6, GPIO_AF6),
        },
        .sdaPins = {
            I2CPINDEF(PD13, GPIO_AF4),
            I2CPINDEF(PB7, GPIO_AF6),
        },
        .rcc = RCC_HB2(I2C4),
    },
#endif
};

i2cDevice_t i2cDevice[I2CDEV_COUNT];

// I2C GPIO configuration for CH32H417
#define IOCFG_I2C IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_OD, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)

static volatile uint16_t i2cErrorCount = 0;

static bool i2cHandleHardwareFailure(I2CDevice device)
{
    (void)device;
    i2cErrorCount++;
    // Reinitialize peripheral (TODO: implement recovery)
    return false;
}

static void i2cUnstick(IO_t scl, IO_t sda)
{
    if (scl == IO_NONE || sda == IO_NONE)
    {
        return;
    }

    // Set SCL and SDA as GPIO outputs
    IOConfigGPIO(scl, IOCFG_OUT_OD);
    IOConfigGPIO(sda, IOCFG_OUT_OD);

    IOHi(scl);
    IOHi(sda);

    // Clock out up to 9 bits to release a stuck slave
    for (int i = 0; i < 9; i++)
    {
        // Wait for any clock stretching
        uint32_t timeout = UNSTICK_CLK_STRETCH;
        while (!IORead(scl) && timeout)
        {
            delayMicroseconds(UNSTICK_CLK_US);
            timeout--;
        }

        IOLo(scl);
        delayMicroseconds(UNSTICK_CLK_US);
        IOHi(scl);
        delayMicroseconds(UNSTICK_CLK_US);
    }

    // Generate a stop condition
    IOLo(sda);
    delayMicroseconds(UNSTICK_CLK_US);
    IOHi(scl);
    delayMicroseconds(UNSTICK_CLK_US);
    IOHi(sda);
}

void i2cInit(I2CDevice device)
{
    if (device == I2CINVALID)
    {
        return;
    }

    i2cDevice_t *pDev = &i2cDevice[device];
    const i2cHardware_t *hw = pDev->hardware;

    if (!hw)
    {
        return;
    }

    I2C_TypeDef *I2Cx = hw->reg;

    IO_t scl = pDev->scl;
    IO_t sda = pDev->sda;

    if (scl == IO_NONE || sda == IO_NONE)
    {
        return;
    }

    // Unstick the bus if needed
    i2cUnstick(scl, sda);

    // Init GPIO pins for I2C (Open-Drain, AF).
    // IMPORTANT: The AF for I2C pins differs per pin on CH32H417 (e.g.
    // I2C2_SCL = PC0(AF9) vs PB10(AF4)). The AF is taken from the
    // i2cHardware table via pDev->sclAF/sdaAF (set by i2cHardwareConfigure).
    // The original code hard-coded AF4, which is wrong for PC0/PC1 (AF9) and
    // so the I2C2 bus (barometer) could not work.
    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));
    IOConfigGPIOAF(scl, IOCFG_I2C, pDev->sclAF);
    IOConfigGPIOAF(sda, IOCFG_I2C, pDev->sdaAF);

    // Enable I2C clock
    RCC_ClockCmd(hw->rcc, ENABLE);

    // Reset I2C peripheral
    I2C_DeInit(I2Cx);

    I2C_InitTypeDef I2C_InitStructure;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = (pDev->clockSpeed ? pDev->clockSpeed : 400) * 1000;

    I2C_Init(I2Cx, &I2C_InitStructure);
    I2C_Cmd(I2Cx, ENABLE);
}

// Blocking I2C write
bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(device, addr_, reg_, 1, &data);
}

bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT)
    {
        return false;
    }

    i2cDevice_t *pDev = &i2cDevice[device];
    I2C_TypeDef *I2Cx = pDev->reg;

    if (!I2Cx)
    {
        return false;
    }

    uint32_t timeout;

    // Wait until I2C is not busy
    timeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) && timeout--)
    {
    }
    if (!timeout)
        return i2cHandleHardwareFailure(device);

    // Send START
    I2C_GenerateSTART(I2Cx, ENABLE);
    timeout = I2C_DEFAULT_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) && timeout--)
    {
    }
    if (!timeout)
        return i2cHandleHardwareFailure(device);

    // Send address (write)
    I2C_Send7bitAddress(I2Cx, addr_ << 1, I2C_Direction_Transmitter);
    timeout = I2C_DEFAULT_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeout--)
    {
    }
    if (!timeout)
        return i2cHandleHardwareFailure(device);

    // Send register address
    I2C_SendData(I2Cx, reg_);
    timeout = I2C_DEFAULT_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout--)
    {
    }
    if (!timeout)
        return i2cHandleHardwareFailure(device);

    // Send data bytes
    for (uint8_t i = 0; i < len_; i++)
    {
        I2C_SendData(I2Cx, data[i]);
        timeout = I2C_DEFAULT_TIMEOUT;
        while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout--)
        {
        }
        if (!timeout)
            return i2cHandleHardwareFailure(device);
    }

    // Send STOP
    I2C_GenerateSTOP(I2Cx, ENABLE);

    return true;
}

bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t *buf)
{
    return i2cReadBuffer(device, addr_, reg_, len, buf);
}

bool i2cReadBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t *buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT)
    {
        return false;
    }

    i2cDevice_t *pDev = &i2cDevice[device];
    I2C_TypeDef *I2Cx = pDev->reg;

    if (!I2Cx)
    {
        return false;
    }

    uint32_t timeout;

    // Wait until I2C is not busy
    timeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) && timeout--)
    {
    }
    if (!timeout)
        return i2cHandleHardwareFailure(device);

    // Send START
    I2C_GenerateSTART(I2Cx, ENABLE);
    timeout = I2C_DEFAULT_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) && timeout--)
    {
    }
    if (!timeout)
        return i2cHandleHardwareFailure(device);

    // Send address (write) to set register
    I2C_Send7bitAddress(I2Cx, addr_ << 1, I2C_Direction_Transmitter);
    timeout = I2C_DEFAULT_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeout--)
    {
    }
    if (!timeout)
        return i2cHandleHardwareFailure(device);

    // Send register address
    I2C_SendData(I2Cx, reg_);
    timeout = I2C_DEFAULT_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout--)
    {
    }
    if (!timeout)
        return i2cHandleHardwareFailure(device);

    // Send repeated START
    I2C_GenerateSTART(I2Cx, ENABLE);
    timeout = I2C_DEFAULT_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) && timeout--)
    {
    }
    if (!timeout)
        return i2cHandleHardwareFailure(device);

    // Send address (read)
    I2C_Send7bitAddress(I2Cx, addr_ << 1, I2C_Direction_Receiver);
    timeout = I2C_DEFAULT_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && timeout--)
    {
    }
    if (!timeout)
        return i2cHandleHardwareFailure(device);

    // Read data bytes
    for (uint8_t i = 0; i < len; i++)
    {
        if (i == len - 1)
        {
            // Last byte: NACK
            I2C_AcknowledgeConfig(I2Cx, DISABLE);
        }

        timeout = I2C_DEFAULT_TIMEOUT;
        while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) && timeout--)
        {
        }
        if (!timeout)
            return i2cHandleHardwareFailure(device);

        buf[i] = I2C_ReceiveData(I2Cx);
    }

    // Send STOP
    I2C_GenerateSTOP(I2Cx, ENABLE);
    // Re-enable ACK for next transfer
    I2C_AcknowledgeConfig(I2Cx, ENABLE);

    return true;
}

bool i2cBusy(I2CDevice device, bool *error)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT)
    {
        if (error)
            *error = true;
        return false;
    }

    i2cDevice_t *pDev = &i2cDevice[device];
    I2C_TypeDef *I2Cx = pDev->reg;

    if (error)
    {
        *error = false;
    }

    if (!I2Cx)
    {
        return false;
    }

    return I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) == SET;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

#endif // USE_I2C && !SOFT_I2C
