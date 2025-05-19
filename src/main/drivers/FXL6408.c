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

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "platform.h"

#ifdef FXL6408

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#define FXL6408_I2C_ADDRESS 0x43

#define FXL6408_REG_DEV_ID 0x01
#define FXL6408_REG_IO_DIR 0x03
#define FXL6408_REG_OUT_HIGHZ 0x07
#define FXL6408_REG_OUT_STATE 0x05

#define FXL6408_DEV_ID 0xA0

static extDevice_t fxlDev;
static uint8_t pinState = 0x00;

void fxl6408SetPinState(uint8_t pin, bool state)
{
    if (state) {
        pinState |= (1 << pin);
    } else {
        pinState &= ~(1 << pin);
    }
    busWriteRegister(&fxlDev, FXL6408_REG_OUT_STATE, pinState);
}

void fxl6408SetAllPins(bool state){
    if (state) {
        pinState = 0xFF;
    } else {
        pinState = 0x00;
    }
    busWriteRegister(&fxlDev, FXL6408_REG_OUT_STATE, pinState);
}

static bool fxl6408Init()
{
    bool ack = true;
    busDeviceRegister(&fxlDev);

    ack = ack && busWriteRegister(&fxlDev, FXL6408_REG_IO_DIR, 0x30); // Set all pins to output
    ack = ack && busWriteRegister(&fxlDev, FXL6408_REG_OUT_STATE, 0xFF);
    ack = ack && busWriteRegister(&fxlDev, FXL6408_REG_OUT_HIGHZ, 0xCF);

    if (!ack) {
        return false;
    }

    return true;
}

bool fxl6408Detect()
{
    fxlDev.busType_u.i2c.address = FXL6408_I2C_ADDRESS;
    i2cBusSetInstance(&fxlDev, 1);

    uint8_t sig = 0;
    bool ack = busReadRegisterBuffer(&fxlDev, FXL6408_REG_DEV_ID, &sig, 1);
    if (ack && sig == FXL6408_DEV_ID) {
        fxl6408Init();
        return true;
    }

    return false;
}
#endif
