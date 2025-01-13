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

#pragma once

#include <stdbool.h>

#include "common/time.h"
#include "common/axis.h"

#include "pg/pg.h"


typedef struct {
    uint8_t pid_process_denom;
    uint8_t filter_process_denom;
} pidConfig_t;

PG_DECLARE(pidConfig_t, pidConfig);


typedef enum {
    PID_ROLL,
    PID_PITCH,
    PID_YAW,
    PID_ITEM_COUNT
} pidIndex_e;

#define PID_AXIS_COUNT      3
#define CYCLIC_AXIS_COUNT   2

enum {
    ITERM_RELAX_OFF,
    ITERM_RELAX_RP,
    ITERM_RELAX_RPY,
};

typedef struct {
    uint16_t P;
    uint16_t I;
    uint16_t D;
    uint16_t F;
    uint16_t B;
    uint16_t O;
} pidf_t;

typedef struct {
    uint8_t level_strength;
    uint8_t level_limit;           // Max angle in degrees in level mode
} pidAngleMode_t;

typedef struct {
    uint8_t level_strength;
    uint8_t transition;
    uint8_t tilt_effect;           // inclination factor for Horizon mode
    uint8_t tilt_expert_mode;      // OFF or ON
} pidHorizonMode_t;

typedef struct {
    uint8_t gain;                  // The strength of the limiting. Raising may reduce overshoot but also lead to oscillation around the angle limit
    uint8_t angle_limit;           // Acro trainer roll/pitch angle limit in degrees
    uint16_t lookahead_ms;         // The lookahead window in milliseconds used to reduce overshoot
} pidTrainerMode_t;

typedef struct {
    uint16_t    headspeed;
    uint8_t     gain;
    uint8_t     p_gain;
    uint8_t     i_gain;
    uint8_t     d_gain;
    uint8_t     f_gain;
    uint8_t     tta_gain;
    uint8_t     tta_limit;
    uint8_t     yaw_ff_weight;
    uint8_t     cyclic_ff_weight;
    uint8_t     collective_ff_weight;
    uint8_t     max_throttle;
    uint8_t     min_throttle;
} governorProfile_t;

typedef struct {
    uint8_t     mode;
    uint8_t     flip_mode;
    uint8_t     flip_gain;
    uint8_t     level_gain;
    uint8_t     pull_up_time;
    uint8_t     climb_time;
    uint8_t     flip_time;
    uint8_t     exit_time;
    uint16_t    pull_up_collective;
    uint16_t    climb_collective;
    uint16_t    hover_collective;
    uint16_t    hover_altitude;
    uint16_t    alt_p_gain;
    uint16_t    alt_i_gain;
    uint16_t    alt_d_gain;
    uint16_t    max_collective;
    uint16_t    max_setpoint_rate;
    uint16_t    max_setpoint_accel;
} pidRescueConfig_t;


#define MAX_PROFILE_NAME_LENGTH 8u

#define LOOKUP_CURVE_POINTS     16

typedef struct pidProfile_s {

    char                profileName[MAX_PROFILE_NAME_LENGTH + 1];

    pidf_t              pid[PID_ITEM_COUNT];

    uint8_t             pid_mode;
    uint8_t             dterm_mode;
    uint8_t             dterm_mode_yaw;

    uint8_t             error_decay_time_ground;
    uint8_t             error_decay_time_cyclic;
    uint8_t             error_decay_time_yaw;
    uint8_t             error_decay_limit_cyclic;
    uint8_t             error_decay_limit_yaw;

    uint8_t             error_decay_rate_curve[LOOKUP_CURVE_POINTS];
    uint8_t             error_decay_limit_curve[LOOKUP_CURVE_POINTS];
    uint8_t             offset_decay_rate_curve[LOOKUP_CURVE_POINTS];
    uint8_t             offset_decay_limit_curve[LOOKUP_CURVE_POINTS];
    uint8_t             offset_bleed_rate_curve[LOOKUP_CURVE_POINTS];
    uint8_t             offset_bleed_limit_curve[LOOKUP_CURVE_POINTS];
    uint8_t             offset_charge_curve[LOOKUP_CURVE_POINTS];

    uint8_t             error_rotation;

    uint8_t             iterm_relax_type;
    uint8_t             iterm_relax_level[PID_AXIS_COUNT];
    uint8_t             iterm_relax_cutoff[PID_AXIS_COUNT];

    uint8_t             offset_limit[CYCLIC_AXIS_COUNT];
    uint8_t             error_limit[PID_AXIS_COUNT];

    uint8_t             error_cutoff[PID_AXIS_COUNT];
    uint8_t             dterm_cutoff[PID_AXIS_COUNT];
    uint8_t             bterm_cutoff[PID_AXIS_COUNT];
    uint8_t             gyro_cutoff[PID_AXIS_COUNT];

    uint8_t             gyro_filter_type;

    uint8_t             yaw_cw_stop_gain;
    uint8_t             yaw_ccw_stop_gain;

    uint8_t             yaw_precomp_cutoff;
    uint8_t             yaw_precomp_filter_type;

    uint8_t             yaw_cyclic_ff_gain;
    uint8_t             yaw_collective_ff_gain;
    uint8_t             yaw_collective_dynamic_gain;
    uint8_t             yaw_collective_dynamic_decay;

    uint8_t             pitch_collective_ff_gain;

    uint8_t             cyclic_cross_coupling_gain;
    uint8_t             cyclic_cross_coupling_ratio;
    uint8_t             cyclic_cross_coupling_cutoff;

    pidAngleMode_t      angle;
    pidHorizonMode_t    horizon;
    pidTrainerMode_t    trainer;
    pidRescueConfig_t   rescue;
    governorProfile_t   governor;

    uint8_t   abs_control;                    // Enable absolute control
    uint8_t   abs_control_gain;               // How strongly should the absolute accumulated error be corrected for
    uint8_t   abs_control_limit;              // Limit to the correction
    uint8_t   abs_control_error_limit;        // Limit to the accumulated error
    uint8_t   abs_control_cutoff;             // Cutoff frequency for path estimation in abs control
} pidProfile_t;

PG_DECLARE_ARRAY(pidProfile_t, PID_PROFILE_COUNT, pidProfiles);

