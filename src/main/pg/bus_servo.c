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

#if defined(USE_SBUS_OUTPUT) || defined(USE_FBUS_MASTER) || defined(USE_BUS_SERVO)

#include "io/serial.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/bus_servo.h"

PG_REGISTER_WITH_RESET_FN(busServoConfig_t, busServoConfig,
                          PG_BUS_SERVO_CONFIG, 0);

void pgResetFn_busServoConfig(busServoConfig_t *config)
{
    // Default first 8 channels to MIXER, rest to RX
    for (int i = 0; i < BUS_SERVO_CHANNELS; i++) {
        config->sourceType[i] = (i < 8) ? BUS_SERVO_SOURCE_MIXER : BUS_SERVO_SOURCE_RX;
    }
}

bool hasBusServosConfigured(void)
{
    return findSerialPortConfig(FUNCTION_SBUS_OUT) || findSerialPortConfig(FUNCTION_FBUS_MASTER);
}

// Storage for bus servo outputs (SBUS/FBUS)
static float busServoOutput[BUS_SERVO_CHANNELS];

void setBusServoOutput(uint8_t channel, float value)
{
    if (channel < BUS_SERVO_CHANNELS) {
        busServoOutput[channel] = value;
    }
}

uint16_t getBusServoOutput(uint8_t channel)
{
    if (channel < BUS_SERVO_CHANNELS) {
        long value = lrintf(busServoOutput[channel]);
        if (value < 0) return 0;
        if (value > UINT16_MAX) return UINT16_MAX;
        return (uint16_t)value;
    }
    return 0;
}
#endif
