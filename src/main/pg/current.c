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
#include "pg/current.h"


#ifndef CURRENT_METER_SCALE_DEFAULT
#define CURRENT_METER_SCALE_DEFAULT 400
#endif

#ifndef CURRENT_METER_OFFSET_DEFAULT
#define CURRENT_METER_OFFSET_DEFAULT 0
#endif

#ifndef CURRENT_METER_CUTOFF_DEFAULT
#define CURRENT_METER_CUTOFF_DEFAULT 25
#endif


PG_REGISTER_ARRAY_WITH_RESET_FN(currentSensorADCConfig_t, MAX_CURRENT_SENSOR_ADC, currentSensorADCConfig, PG_CURRENT_SENSOR_ADC_CONFIG, 0);

void pgResetFn_currentSensorADCConfig(currentSensorADCConfig_t *instance)
{
    for (int i = 0; i < MAX_CURRENT_SENSOR_ADC; i++) {
        RESET_CONFIG(currentSensorADCConfig_t, &instance[i],
            .scale = CURRENT_METER_SCALE_DEFAULT,
            .offset = CURRENT_METER_OFFSET_DEFAULT,
            .cutoff = CURRENT_METER_CUTOFF_DEFAULT,
        );
    }
}
