/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include "common/time.h"
#include "common/filter.h"
#include "common/axis.h"

#include "pg/pg.h"

#define MAX_PID_PROCESS_DENOM       16
#define PID_CONTROLLER_BETAFLIGHT   1
#define PID_MIXER_SCALING           1000.0f
#define PID_SERVO_MIXER_SCALING     0.7f
#define PIDSUM_LIMIT                500
#define PIDSUM_LIMIT_YAW            400
#define PIDSUM_LIMIT_MIN            100
#define PIDSUM_LIMIT_MAX            1000

#define PID_GAIN_MAX 250
#define F_GAIN_MAX 1000

// Scaling factors for Pids for better tunable range in configurator for betaflight pid controller. The scaling is based on legacy pid controller or previous float
#define PTERM_SCALE 0.032029f
#define ITERM_SCALE 0.244381f
#define DTERM_SCALE 0.000529f

// The constant scale factor to replace the Kd component of the feedforward calculation.
// This value gives the same "feel" as the previous Kd default of 26 (26 * DTERM_SCALE)
#define FEEDFORWARD_SCALE 0.013754f

#define PID_ROLL_DEFAULT  { 45, 80, 40, 120 }
#define PID_PITCH_DEFAULT { 47, 84, 46, 125 }
#define PID_YAW_DEFAULT   { 45, 80,  0, 120 }

#define DTERM_LPF1_DYN_MIN_HZ_DEFAULT 75
#define DTERM_LPF1_DYN_MAX_HZ_DEFAULT 150
#define DTERM_LPF2_HZ_DEFAULT 150

typedef enum {
    PID_ROLL,
    PID_PITCH,
    PID_YAW,
    PID_LEVEL,
    PID_MAG,
    PID_ITEM_COUNT
} pidIndex_e;

typedef enum {
    SUPEREXPO_YAW_OFF = 0,
    SUPEREXPO_YAW_ON,
    SUPEREXPO_YAW_ALWAYS
} pidSuperExpoYaw_e;

typedef struct pidf_s {
    uint8_t P;
    uint8_t I;
    uint8_t D;
    uint16_t F;
} pidf_t;

#define MAX_PROFILE_NAME_LENGTH 8u

typedef struct pidProfile_s {
    uint16_t dterm_lpf1_static_hz;          // Static Dterm lowpass 1 filter cutoff value in hz
    uint16_t dterm_notch_hz;                // Biquad dterm notch hz
    uint16_t dterm_notch_cutoff;            // Biquad dterm notch low cutoff

    pidf_t  pid[PID_ITEM_COUNT];

    uint8_t dterm_lpf1_type;                // Filter type for dterm lowpass 1
    uint16_t pidSumLimit;
    uint16_t pidSumLimitYaw;
    uint8_t levelAngleLimit;                // Max angle in degrees in level mode

    uint8_t horizon_tilt_effect;            // inclination factor for Horizon mode
    uint8_t horizon_tilt_expert_mode;       // OFF or ON

    // Betaflight PID controller parameters
    uint16_t yawRateAccelLimit;             // yaw accel limiter for deg/sec/ms
    uint16_t rateAccelLimit;                // accel limiter roll/pitch deg/sec/ms
    uint16_t itermLimit;
    uint16_t dterm_lpf2_static_hz;          // Static Dterm lowpass 2 filter cutoff value in hz
    uint8_t iterm_rotation;                 // rotates iterm to translate world errors to local coordinate system
    uint8_t acro_trainer_angle_limit;       // Acro trainer roll/pitch angle limit in degrees
    uint8_t acro_trainer_debug_axis;        // The axis for which record debugging values are captured 0=roll, 1=pitch
    uint8_t acro_trainer_gain;              // The strength of the limiting. Raising may reduce overshoot but also lead to oscillation around the angle limit
    uint16_t acro_trainer_lookahead_ms;     // The lookahead window in milliseconds used to reduce overshoot
    uint8_t dterm_lpf2_type;                // Filter type for 2nd dterm lowpass
    uint16_t dterm_lpf1_dyn_min_hz;         // Dterm lowpass filter 1 min hz when in dynamic mode
    uint16_t dterm_lpf1_dyn_max_hz;         // Dterm lowpass filter 1 max hz when in dynamic mode
    char profileName[MAX_PROFILE_NAME_LENGTH + 1]; // Descriptive name for profile

} pidProfile_t;

PG_DECLARE_ARRAY(pidProfile_t, PID_PROFILE_COUNT, pidProfiles);

typedef struct pidConfig_s {
    uint8_t pid_process_denom;              // Processing denominator for PID controller vs gyro sampling rate
} pidConfig_t;

PG_DECLARE(pidConfig_t, pidConfig);

union rollAndPitchTrims_u;
void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs);

typedef struct pidAxisData_s {
    float P;
    float I;
    float D;
    float F;

    float Sum;
} pidAxisData_t;

typedef union dtermLowpass_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
    pt2Filter_t pt2Filter;
    pt3Filter_t pt3Filter;
} dtermLowpass_t;

typedef struct pidCoefficient_s {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
} pidCoefficient_t;

typedef struct pidRuntime_s {
    float dT;
    float pidFrequency;
    float previousPidSetpoint[XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermNotchApplyFn;
    biquadFilter_t dtermNotch[XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermLowpassApplyFn;
    dtermLowpass_t dtermLowpass[XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermLowpass2ApplyFn;
    dtermLowpass_t dtermLowpass2[XYZ_AXIS_COUNT];
    pidCoefficient_t pidCoefficient[XYZ_AXIS_COUNT];
    float levelGain;
    float horizonGain;
    float horizonTransition;
    float horizonCutoffDegrees;
    float horizonFactorRatio;
    uint8_t horizonTiltExpertMode;
    float maxVelocity[XYZ_AXIS_COUNT];
    float itermLimit;
    bool itermRotation;

#ifdef USE_DYN_LPF
    uint8_t dynLpfFilter;
    uint16_t dynLpfMin;
    uint16_t dynLpfMax;
#endif

} pidRuntime_t;

extern pidRuntime_t pidRuntime;

extern const char pidNames[];

extern pidAxisData_t pidData[3];

extern uint32_t targetPidLooptime;

extern pt1Filter_t throttleLpf;

void resetPidProfile(pidProfile_t *profile);

void pidResetIterm(void);

#ifdef UNIT_TEST
#include "sensors/acceleration.h"
extern float axisError[XYZ_AXIS_COUNT];
void rotateItermAndAxisError();
float pidLevel(int axis, const pidProfile_t *pidProfile,
    const rollAndPitchTrims_t *angleTrim, float currentPidSetpoint);
float calcHorizonLevelStrength(void);
#endif
void dynLpfDTermUpdate(float throttle);
float pidGetPreviousSetpoint(int axis);
float pidGetDT();
float pidGetPidFrequency();
