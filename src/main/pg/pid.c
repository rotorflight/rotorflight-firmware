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
            [PID_ROLL]  = { .P = 50, .I = 100, .D =  0, .F = 100, .B = 0, .O = 50, },
            [PID_PITCH] = { .P = 50, .I = 100, .D = 40, .F = 100, .B = 0, .O = 50, },
            [PID_YAW]   = { .P = 80, .I = 120, .D = 10, .F =   0, .B = 0, .O =  0, },
        },
        .pid_mode = 3,
        .error_decay_time_ground = 25,
        .error_decay_time_cyclic = 250,
        .error_decay_time_yaw = 0,
        .error_decay_limit_cyclic = 12,
        .error_decay_limit_yaw = 0,
        .offset_flood_relax_level = 40,
        .offset_flood_relax_cutoff = 3,
        .iterm_relax_type = ITERM_RELAX_RPY,
        .iterm_relax_level = { 40, 40, 40 },
        .iterm_relax_cutoff = { 10, 10, 10 },
        .offset_limit = { 45, 45 },
        .error_limit = { 30, 30, 45 },
        .dterm_cutoff = { 15, 15, 20 },
        .bterm_cutoff = { 15, 15, 20 },
        .gyro_cutoff = { 50, 50, 100 },
        .gyro_filter_type = LPF_1ST_ORDER,
        .yaw_cw_stop_gain = 120,
        .yaw_ccw_stop_gain = 80,
        .yaw_precomp_cutoff = 5,
        .yaw_precomp_filter_type = LPF_1ST_ORDER,
        .yaw_cyclic_ff_gain = 10,
        .yaw_collective_ff_gain = 60,
        .yaw_inertia_precomp_gain = 0,
        .yaw_inertia_precomp_cutoff = 25,
        .pitch_collective_ff_gain = 0,
        .cyclic_cross_coupling_gain = 50,
        .cyclic_cross_coupling_ratio = 0,
        .cyclic_cross_coupling_cutoff = 25,
        .angle.level_strength = 40,
        .angle.level_limit = 55,
        .horizon.level_strength = 40,
        .horizon.transition = 75,
        .horizon.tilt_effect = 75,
        .horizon.tilt_expert_mode = false,
        .trainer.gain = 75,
        .trainer.angle_limit = 20,
        .trainer.lookahead_ms = 50,
        .rescue.mode = 0,
        .rescue.flip_mode = 1,
        .rescue.flip_gain = 200,
        .rescue.level_gain = 100,
        .rescue.pull_up_time = 3,
        .rescue.climb_time = 10,
        .rescue.flip_time = 20,
        .rescue.exit_time = 5,
        .rescue.pull_up_collective = 650,
        .rescue.climb_collective = 450,
        .rescue.hover_collective = 350,
        .rescue.hover_altitude = 500,
        .rescue.alt_p_gain = 20,
        .rescue.alt_i_gain = 20,
        .rescue.alt_d_gain = 10,
        .rescue.max_collective = 500,
        .rescue.max_setpoint_rate = 300,
        .rescue.max_setpoint_accel = 3000,
        .governor.flags = 0,
        .governor.headspeed = 1000,
        .governor.max_throttle = 100,
        .governor.min_throttle = 10,
        .governor.gain = 40,
        .governor.p_gain = 40,
        .governor.i_gain = 50,
        .governor.d_gain = 0,
        .governor.f_gain = 10,
        .governor.p_limit = 20,
        .governor.i_limit = 95,
        .governor.d_limit = 20,
        .governor.f_limit = 100,
        .governor.tta_gain = 0,
        .governor.tta_limit = 20,
        .governor.yaw_weight = 10,
        .governor.cyclic_weight = 10,
        .governor.collective_weight = 50,
        .governor.collective_curve = 20,
        .governor.fallback_drop = 10,
        .governor.dyn_min_level = 80,   // TDB remove
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

