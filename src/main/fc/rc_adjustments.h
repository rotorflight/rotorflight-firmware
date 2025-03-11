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

#include "types.h"

#include "fc/rc_modes.h"
#include "flight/pid.h"

#include "pg/adjustments.h"


/*
 * Adjustment Function codes
 *
 * Use explicit values here, so that
 *    - Each value is visible for reference
 *    - Any change to all values will show up in a diff
 */

typedef enum {

    ADJUSTMENT_NONE                     = 0,

    // Profile change
    ADJUSTMENT_RATE_PROFILE             = 1,
    ADJUSTMENT_PID_PROFILE              = 2,
    ADJUSTMENT_LED_PROFILE              = 3,
    ADJUSTMENT_OSD_PROFILE              = 4,

    // Rates
    ADJUSTMENT_PITCH_RATE               = 5,
    ADJUSTMENT_ROLL_RATE                = 6,
    ADJUSTMENT_YAW_RATE                 = 7,
    ADJUSTMENT_PITCH_RC_RATE            = 8,
    ADJUSTMENT_ROLL_RC_RATE             = 9,
    ADJUSTMENT_YAW_RC_RATE              = 10,
    ADJUSTMENT_PITCH_RC_EXPO            = 11,
    ADJUSTMENT_ROLL_RC_EXPO             = 12,
    ADJUSTMENT_YAW_RC_EXPO              = 13,

    // PID
    ADJUSTMENT_PITCH_P_GAIN             = 14,
    ADJUSTMENT_PITCH_I_GAIN             = 15,
    ADJUSTMENT_PITCH_D_GAIN             = 16,
    ADJUSTMENT_PITCH_F_GAIN             = 17,
    ADJUSTMENT_ROLL_P_GAIN              = 18,
    ADJUSTMENT_ROLL_I_GAIN              = 19,
    ADJUSTMENT_ROLL_D_GAIN              = 20,
    ADJUSTMENT_ROLL_F_GAIN              = 21,
    ADJUSTMENT_YAW_P_GAIN               = 22,
    ADJUSTMENT_YAW_I_GAIN               = 23,
    ADJUSTMENT_YAW_D_GAIN               = 24,
    ADJUSTMENT_YAW_F_GAIN               = 25,

    ADJUSTMENT_YAW_CW_GAIN              = 26,
    ADJUSTMENT_YAW_CCW_GAIN             = 27,
    ADJUSTMENT_YAW_CYCLIC_FF            = 28,
    ADJUSTMENT_YAW_COLLECTIVE_FF        = 29,
    ADJUSTMENT_YAW_COLLECTIVE_DYN       = 30,
    ADJUSTMENT_YAW_COLLECTIVE_DECAY     = 31,
    ADJUSTMENT_PITCH_COLLECTIVE_FF      = 32,

    // Gyro cutoffs
    ADJUSTMENT_PITCH_GYRO_CUTOFF        = 33,
    ADJUSTMENT_ROLL_GYRO_CUTOFF         = 34,
    ADJUSTMENT_YAW_GYRO_CUTOFF          = 35,

    // Dterm cutoffs
    ADJUSTMENT_PITCH_DTERM_CUTOFF       = 36,
    ADJUSTMENT_ROLL_DTERM_CUTOFF        = 37,
    ADJUSTMENT_YAW_DTERM_CUTOFF         = 38,

    // Rescue
    ADJUSTMENT_RESCUE_CLIMB_COLLECTIVE  = 39,
    ADJUSTMENT_RESCUE_HOVER_COLLECTIVE  = 40,
    ADJUSTMENT_RESCUE_HOVER_ALTITUDE    = 41,
    ADJUSTMENT_RESCUE_ALT_P_GAIN        = 42,
    ADJUSTMENT_RESCUE_ALT_I_GAIN        = 43,
    ADJUSTMENT_RESCUE_ALT_D_GAIN        = 44,

    // Leveling
    ADJUSTMENT_ANGLE_LEVEL_GAIN         = 45,
    ADJUSTMENT_HORIZON_LEVEL_GAIN       = 46,
    ADJUSTMENT_ACRO_TRAINER_GAIN        = 47,

    // Governor
    ADJUSTMENT_GOV_GAIN                 = 48,
    ADJUSTMENT_GOV_P_GAIN               = 49,
    ADJUSTMENT_GOV_I_GAIN               = 50,
    ADJUSTMENT_GOV_D_GAIN               = 51,
    ADJUSTMENT_GOV_F_GAIN               = 52,
    ADJUSTMENT_GOV_TTA_GAIN             = 53,
    ADJUSTMENT_GOV_CYCLIC_FF            = 54,
    ADJUSTMENT_GOV_COLLECTIVE_FF        = 55,

    // Boost gains
    ADJUSTMENT_PITCH_B_GAIN             = 56,
    ADJUSTMENT_ROLL_B_GAIN              = 57,
    ADJUSTMENT_YAW_B_GAIN               = 58,

    // Offset gains
    ADJUSTMENT_PITCH_O_GAIN             = 59,
    ADJUSTMENT_ROLL_O_GAIN              = 60,

    // Cross-coupling
    ADJUSTMENT_CROSS_COUPLING_GAIN      = 61,
    ADJUSTMENT_CROSS_COUPLING_RATIO     = 62,
    ADJUSTMENT_CROSS_COUPLING_CUTOFF    = 63,

    // Accelerometer
    ADJUSTMENT_ACC_TRIM_PITCH           = 64,
    ADJUSTMENT_ACC_TRIM_ROLL            = 65,

    // Yaw Inertia precomp
    ADJUSTMENT_INERTIA_PRECOMP_GAIN     = 66,
    ADJUSTMENT_INERTIA_PRECOMP_CUTOFF   = 67,

    // Setpoint boost
    ADJUSTMENT_PITCH_SP_BOOST_GAIN      = 68,
    ADJUSTMENT_ROLL_SP_BOOST_GAIN       = 69,
    ADJUSTMENT_YAW_SP_BOOST_GAIN        = 70,
    ADJUSTMENT_COLL_SP_BOOST_GAIN       = 71,

    // Yaw dynamic deadband
    ADJUSTMENT_YAW_DYN_CEILING_GAIN     = 72,
    ADJUSTMENT_YAW_DYN_DEADBAND_GAIN    = 73,
    ADJUSTMENT_YAW_DYN_DEADBAND_FILTER  = 74,

    // Precomp cutoff
    ADJUSTMENT_YAW_PRECOMP_CUTOFF       = 75,

    ADJUSTMENT_FUNCTION_COUNT
} adjustmentFunc_e;


void adjustmentRangeInit(void);
void adjustmentRangeReset(int index);

void processRcAdjustments(void);

const char *getAdjustmentsRangeName(void);
int getAdjustmentsRangeFunc(void);
int getAdjustmentsRangeValue(void);

