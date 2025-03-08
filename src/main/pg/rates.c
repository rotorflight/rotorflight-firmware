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

#include "common/axis.h"

#include "pg/rates.h"

#include "config/config_reset.h"


typedef enum {
    RATES_TYPE_NONE = 0,
    RATES_TYPE_BETAFLIGHT,
    RATES_TYPE_RACEFLIGHT,
    RATES_TYPE_KISS,
    RATES_TYPE_ACTUAL,
    RATES_TYPE_QUICK,
    RATES_TYPE_COUNT
} ratesType_e;


PG_REGISTER_ARRAY_WITH_RESET_FN(controlRateConfig_t, CONTROL_RATE_PROFILE_COUNT, controlRateProfiles, PG_CONTROL_RATE_PROFILES, 4);

void pgResetFn_controlRateProfiles(controlRateConfig_t *controlRateConfig)
{
    for (int i = 0; i < CONTROL_RATE_PROFILE_COUNT; i++) {
        RESET_CONFIG(controlRateConfig_t, &controlRateConfig[i],
            .profileName = INIT_ZERO,
            .rates_type = RATES_TYPE_ACTUAL,
            .rcRates[FD_ROLL] = 18,
            .rcRates[FD_PITCH] = 18,
            .rcRates[FD_YAW] = 18,
            .rcRates[FD_COLL] = 50,
            .rcExpo[FD_ROLL] = 0,
            .rcExpo[FD_PITCH] = 0,
            .rcExpo[FD_YAW] = 0,
            .rcExpo[FD_COLL] = 0,
            .rates[FD_ROLL] = 24,
            .rates[FD_PITCH] = 24,
            .rates[FD_YAW] = 40,
            .rates[FD_COLL] = 50,
            .levelExpo[FD_ROLL] = 0,
            .levelExpo[FD_PITCH] = 0,
            .quickRatesRcExpo = 0,
            .response_time[FD_ROLL] = 0,
            .response_time[FD_PITCH] = 0,
            .response_time[FD_YAW] = 0,
            .response_time[FD_COLL] = 0,
            .accel_limit[FD_ROLL] = 0,
            .accel_limit[FD_PITCH] = 0,
            .accel_limit[FD_YAW] = 0,
            .accel_limit[FD_COLL] = 0,
            .setpoint_boost_gain[FD_ROLL] = 0,
            .setpoint_boost_gain[FD_PITCH] = 0,
            .setpoint_boost_gain[FD_YAW] = 0,
            .setpoint_boost_gain[FD_COLL] = 0,
            .setpoint_boost_cutoff[FD_ROLL] = 15,
            .setpoint_boost_cutoff[FD_PITCH] = 15,
            .setpoint_boost_cutoff[FD_YAW] = 90,
            .setpoint_boost_cutoff[FD_COLL] = 15,
            .yaw_dynamic_deadband_gain = 30,
            .yaw_dynamic_deadband_cutoff = 30,
            .yaw_dynamic_deadband_filter = 60,
        );
    }
}
