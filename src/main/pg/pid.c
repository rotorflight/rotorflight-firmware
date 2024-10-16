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
            [PID_ROLL]  = { .P = 50, .I = 100, .D = 20, .F = 100, .B = 0, .O = 40, },
            [PID_PITCH] = { .P = 50, .I = 100, .D = 50, .F = 100, .B = 0, .O = 40, },
            [PID_YAW]   = { .P = 80, .I = 120, .D = 40, .F =   0, .B = 0, .O =  0, },
        },
        .pid_mode = 3,
        .dterm_mode = 0,
        .dterm_mode_yaw = 0,
        .error_decay_time_ground = 25,
        .error_decay_time_cyclic = 250,
        .error_decay_time_yaw = 0,
        .error_decay_limit_cyclic = 12,
        .error_decay_limit_yaw = 0,
        .error_decay_rate_curve = { 12,13,14,15,17,20,23,28,36,49,78,187,250,250,250,250 },
        .error_decay_limit_curve = { 12,12,12,12,12,12,12,12,12,12,12,12,13,14,15,16 },
        .offset_decay_rate_curve = { 250,250,250,250,25,3,1,0,0,0,0,0,0,0,0,0 },
        .offset_decay_limit_curve = { 5,5,4,3,2,2,2,2,2,2,2,2,2,2,2,2 },
        .offset_bleed_rate_curve = { 0,0,0,0,0,0,2,4,30,250,250,250,250,250,250,250 },
        .offset_bleed_limit_curve = { 0,0,0,0,0,0,15,40,100,150,200,250,250,250,250,250 },
        .offset_charge_curve = { 0,100,100,100,100,100,95,90,82,76,72,68,65,62,60,58 },
        .error_rotation = true,
        .iterm_relax_type = ITERM_RELAX_RPY,
        .iterm_relax_level = { 40, 40, 40 },
        .iterm_relax_cutoff = { 10, 10, 10 },
        .offset_limit = { 45, 45 },
        .error_limit = { 30, 30, 45 },
        .error_cutoff = { 0, 0, 0 },
        .dterm_cutoff = { 15, 15, 20 },
        .bterm_cutoff = { 15, 15, 20 },
        .gyro_cutoff = { 50, 50, 100 },
        .gyro_filter_type = LPF_1ST_ORDER,
        .yaw_cw_stop_gain = 120,
        .yaw_ccw_stop_gain = 80,
        .yaw_precomp_cutoff = 5,
        .yaw_precomp_filter_type = LPF_1ST_ORDER,
        .yaw_cyclic_ff_gain = 0,
        .yaw_collective_ff_gain = 30,
        .yaw_collective_dynamic_gain = 0,
        .yaw_collective_dynamic_decay = 25,
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
        .rescue.flip_mode = 0,
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
        .governor.headspeed = 1000,
        .governor.gain = 40,
        .governor.p_gain = 40,
        .governor.i_gain = 50,
        .governor.d_gain = 0,
        .governor.f_gain = 10,
        .governor.tta_gain = 0,
        .governor.tta_limit = 20,
        .governor.cyclic_ff_weight = 10,
        .governor.collective_ff_weight = 100,
        .governor.max_throttle = 100,
        .governor.min_throttle = 10,
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

