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
#include <string.h>

#include "platform.h"

#include "config/config_reset.h"

#include "flight/pid.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pid.h"


PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 3);

PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT,
    .filter_process_denom = FILTER_PROCESS_DENOM_DEFAULT,
);

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 0);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .profileName = "",
        .pid = {
            [PID_ROLL]  = PID_ROLL_DEFAULT,
            [PID_PITCH] = PID_PITCH_DEFAULT,
            [PID_YAW]   = PID_YAW_DEFAULT,
            [PID_WAY]   = PID_YAW_DEFAULT,
        },
        .pid_mode = 2,
        .dterm_mode = 0,
        .error_decay = 25,
        .error_rotation = true,
        .iterm_relax_type = ITERM_RELAX_OFF,
        .iterm_relax_level = { 40, 40, 40 },
        .iterm_relax_cutoff = { 10, 10, 15 },
        .error_limit = { 90, 90, 90 },
        .error_cutoff = { 0, 0, 0 },
        .dterm_cutoff = { 15, 15, 20 },
        .fterm_cutoff = { 0, 0, 0 },
        .gyro_cutoff = { 50, 50, 50 },
        .yaw_cw_stop_gain = 100,
        .yaw_ccw_stop_gain = 100,
        .yaw_cyclic_ff_gain = 0,
        .yaw_collective_ff_gain = 0,
        .yaw_collective_ff_impulse_gain = 0,
        .yaw_collective_ff_impulse_freq = 50,
        .pitch_collective_ff_gain = 0,
        .pitch_collective_ff_impulse_gain = 0,
        .angle.level_strength = 50,
        .angle.level_limit = 55,
        .horizon.level_strength = 50,
        .horizon.transition = 75,
        .horizon.tilt_effect = 75,
        .horizon.tilt_expert_mode = false,
        .trainer.gain = 75,
        .trainer.angle_limit = 20,
        .trainer.lookahead_ms = 50,
        .rescue.mode = 0,
        .rescue.flip_mode = 0,
        .rescue.flip_gain = 40,
        .rescue.level_gain = 30,
        .rescue.pull_up_time = 20,
        .rescue.climb_time = 40,
        .rescue.flip_time = 20,
        .rescue.exit_time = 20,
        .rescue.pull_up_collective = 500,
        .rescue.climb_collective = 350,
        .rescue.hover_collective = 200,
        .rescue.hover_altitude = 600,
        .rescue.alt_a_gain = 250,
        .rescue.alt_p_gain = 250,
        .rescue.alt_i_gain = 250,
        .rescue.max_collective = 750,
        .rescue.max_climb_rate = 250,
        .rescue.max_setpoint_rate = 250,
        .rescue.max_setpoint_accel = 2000,
        .governor.headspeed = 1000,
        .governor.gain = 100,
        .governor.p_gain = 20,
        .governor.i_gain = 20,
        .governor.d_gain = 0,
        .governor.f_gain = 0,
        .governor.tta_gain = 0,
        .governor.tta_limit = 0,
        .governor.cyclic_ff_weight = 40,
        .governor.collective_ff_weight = 100,
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

