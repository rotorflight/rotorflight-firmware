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

#include "types.h"
#include "platform.h"

#include "config/config_reset.h"

#include "pg/pg_ids.h"
#include "pg/voltage.h"


#ifndef VOLTAGE_SCALE_DEFAULT
#define VOLTAGE_SCALE_DEFAULT 110
#endif

#ifndef VOLTAGE_DIVIDER_DEFAULT
#define VOLTAGE_DIVIDER_DEFAULT 10
#endif

#ifndef VOLTAGE_MULTIPLIER_DEFAULT
#define VOLTAGE_MULTIPLIER_DEFAULT 1
#endif

#ifndef VOLTAGE_CUTOFF_DEFAULT
#define VOLTAGE_CUTOFF_DEFAULT 25
#endif


PG_REGISTER_ARRAY_WITH_RESET_FN(voltageSensorADCConfig_t, MAX_VOLTAGE_SENSOR_ADC, voltageSensorADCConfig, PG_VOLTAGE_SENSOR_ADC_CONFIG, 0);

void pgResetFn_voltageSensorADCConfig(voltageSensorADCConfig_t *instance)
{
    for (int i = 0; i < MAX_VOLTAGE_SENSOR_ADC; i++) {
        RESET_CONFIG(voltageSensorADCConfig_t, &instance[i],
            .scale = VOLTAGE_SCALE_DEFAULT,
            .divider = VOLTAGE_DIVIDER_DEFAULT,
            .divmul = VOLTAGE_MULTIPLIER_DEFAULT,
            .cutoff = VOLTAGE_CUTOFF_DEFAULT,
        );
    }
}

