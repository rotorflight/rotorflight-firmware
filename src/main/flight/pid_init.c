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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"

#include "drivers/dshot_command.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "flight/rpm_filter.h"

#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "pid_init.h"

static void pidSetTargetLooptime(uint32_t pidLooptime)
{
    targetPidLooptime = pidLooptime;
    pidRuntime.dT = targetPidLooptime * 1e-6f;
    pidRuntime.pidFrequency = 1.0f / pidRuntime.dT;
#ifdef USE_DSHOT
    dshotSetPidLoopTime(targetPidLooptime);
#endif
}

void pidInitFilters(const pidProfile_t *pidProfile)
{
    STATIC_ASSERT(FD_YAW == 2, FD_YAW_incorrect); // ensure yaw axis is 2

    if (targetPidLooptime == 0) {
        // no looptime set, so set all the filters to null
        pidRuntime.dtermNotchApplyFn = nullFilterApply;
        pidRuntime.dtermLowpassApplyFn = nullFilterApply;
        pidRuntime.dtermLowpass2ApplyFn = nullFilterApply;
        return;
    }

    const uint32_t pidFrequencyNyquist = pidRuntime.pidFrequency / 2; // No rounding needed

    uint16_t dTermNotchHz;
    if (pidProfile->dterm_notch_hz <= pidFrequencyNyquist) {
        dTermNotchHz = pidProfile->dterm_notch_hz;
    } else {
        if (pidProfile->dterm_notch_cutoff < pidFrequencyNyquist) {
            dTermNotchHz = pidFrequencyNyquist;
        } else {
            dTermNotchHz = 0;
        }
    }

    if (dTermNotchHz != 0 && pidProfile->dterm_notch_cutoff != 0) {
        pidRuntime.dtermNotchApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(dTermNotchHz, pidProfile->dterm_notch_cutoff);
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            biquadFilterInit(&pidRuntime.dtermNotch[axis], dTermNotchHz, targetPidLooptime, notchQ, FILTER_NOTCH, 1.0f);
        }
    } else {
        pidRuntime.dtermNotchApplyFn = nullFilterApply;
    }

    //1st Dterm Lowpass Filter
    uint16_t dterm_lpf1_init_hz = pidProfile->dterm_lpf1_static_hz;

#ifdef USE_DYN_LPF
    if (pidProfile->dterm_lpf1_dyn_min_hz) {
        dterm_lpf1_init_hz = pidProfile->dterm_lpf1_dyn_min_hz;
    }
#endif

    if (dterm_lpf1_init_hz > 0) {
        switch (pidProfile->dterm_lpf1_type) {
        case FILTER_PT1:
            pidRuntime.dtermLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt1FilterInit(&pidRuntime.dtermLowpass[axis].pt1Filter, pt1FilterGain(dterm_lpf1_init_hz, pidRuntime.dT));
            }
            break;
        case FILTER_BIQUAD:
            if (pidProfile->dterm_lpf1_static_hz < pidFrequencyNyquist) {
#ifdef USE_DYN_LPF
                pidRuntime.dtermLowpassApplyFn = (filterApplyFnPtr)biquadFilterApplyDF1;
#else
                pidRuntime.dtermLowpassApplyFn = (filterApplyFnPtr)biquadFilterApply;
#endif
                for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                    biquadFilterInitLPF(&pidRuntime.dtermLowpass[axis].biquadFilter, dterm_lpf1_init_hz, targetPidLooptime);
                }
            } else {
                pidRuntime.dtermLowpassApplyFn = nullFilterApply;
            }
            break;
        case FILTER_PT2:
            pidRuntime.dtermLowpassApplyFn = (filterApplyFnPtr)pt2FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt2FilterInit(&pidRuntime.dtermLowpass[axis].pt2Filter, pt2FilterGain(dterm_lpf1_init_hz, pidRuntime.dT));
            }
            break;
        case FILTER_PT3:
            pidRuntime.dtermLowpassApplyFn = (filterApplyFnPtr)pt3FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt3FilterInit(&pidRuntime.dtermLowpass[axis].pt3Filter, pt3FilterGain(dterm_lpf1_init_hz, pidRuntime.dT));
            }
            break;
        default:
            pidRuntime.dtermLowpassApplyFn = nullFilterApply;
            break;
        }
    } else {
        pidRuntime.dtermLowpassApplyFn = nullFilterApply;
    }

    //2nd Dterm Lowpass Filter
    if (pidProfile->dterm_lpf2_static_hz > 0) {
        switch (pidProfile->dterm_lpf2_type) {
        case FILTER_PT1:
            pidRuntime.dtermLowpass2ApplyFn = (filterApplyFnPtr)pt1FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt1FilterInit(&pidRuntime.dtermLowpass2[axis].pt1Filter, pt1FilterGain(pidProfile->dterm_lpf2_static_hz, pidRuntime.dT));
            }
            break;
        case FILTER_BIQUAD:
            if (pidProfile->dterm_lpf2_static_hz < pidFrequencyNyquist) {
                pidRuntime.dtermLowpass2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
                for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                    biquadFilterInitLPF(&pidRuntime.dtermLowpass2[axis].biquadFilter, pidProfile->dterm_lpf2_static_hz, targetPidLooptime);
                }
            } else {
                pidRuntime.dtermLowpassApplyFn = nullFilterApply;
            }
            break;
        case FILTER_PT2:
            pidRuntime.dtermLowpass2ApplyFn = (filterApplyFnPtr)pt2FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt2FilterInit(&pidRuntime.dtermLowpass2[axis].pt2Filter, pt2FilterGain(pidProfile->dterm_lpf2_static_hz, pidRuntime.dT));
            }
            break;
        case FILTER_PT3:
            pidRuntime.dtermLowpass2ApplyFn = (filterApplyFnPtr)pt3FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt3FilterInit(&pidRuntime.dtermLowpass2[axis].pt3Filter, pt3FilterGain(pidProfile->dterm_lpf2_static_hz, pidRuntime.dT));
            }
            break;
        default:
            pidRuntime.dtermLowpass2ApplyFn = nullFilterApply;
            break;
        }
    } else {
        pidRuntime.dtermLowpass2ApplyFn = nullFilterApply;
    }

#if defined(USE_ITERM_RELAX)
    if (pidRuntime.itermRelax) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            pt1FilterInit(&pidRuntime.windupLpf[i], pt1FilterGain(pidRuntime.itermRelaxCutoff, pidRuntime.dT));
        }
    }
#endif

}

void pidInit(const pidProfile_t *pidProfile)
{
    pidSetTargetLooptime(gyro.targetLooptime); // Initialize pid looptime
    pidInitFilters(pidProfile);
    pidInitConfig(pidProfile);
#ifdef USE_RPM_FILTER
    rpmFilterInit(rpmFilterConfig());
#endif
}

void pidInitConfig(const pidProfile_t *pidProfile)
{
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pidRuntime.pidCoefficient[axis].Kp = PTERM_SCALE * pidProfile->pid[axis].P;
        pidRuntime.pidCoefficient[axis].Ki = ITERM_SCALE * pidProfile->pid[axis].I;
        pidRuntime.pidCoefficient[axis].Kd = DTERM_SCALE * pidProfile->pid[axis].D;
        pidRuntime.pidCoefficient[axis].Kf = FEEDFORWARD_SCALE * (pidProfile->pid[axis].F / 100.0f);
    }
    pidRuntime.levelGain = pidProfile->pid[PID_LEVEL].P / 10.0f;
    pidRuntime.horizonGain = pidProfile->pid[PID_LEVEL].I / 10.0f;
    pidRuntime.horizonTransition = (float)pidProfile->pid[PID_LEVEL].D;
    pidRuntime.horizonTiltExpertMode = pidProfile->horizon_tilt_expert_mode;
    pidRuntime.horizonCutoffDegrees = (175 - pidProfile->horizon_tilt_effect) * 1.8f;
    pidRuntime.horizonFactorRatio = (100 - pidProfile->horizon_tilt_effect) * 0.01f;
    pidRuntime.maxVelocity[FD_ROLL] = pidRuntime.maxVelocity[FD_PITCH] = pidProfile->rateAccelLimit * 100 * pidRuntime.dT;
    pidRuntime.maxVelocity[FD_YAW] = pidProfile->yawRateAccelLimit * 100 * pidRuntime.dT;
    pidRuntime.itermLimit = pidProfile->itermLimit;
    pidRuntime.itermRotation = pidProfile->iterm_rotation;

#if defined(USE_ITERM_RELAX)
    pidRuntime.itermRelax = pidProfile->iterm_relax;
    pidRuntime.itermRelaxType = pidProfile->iterm_relax_type;
    pidRuntime.itermRelaxCutoff = pidProfile->iterm_relax_cutoff;
#endif

#ifdef USE_ACRO_TRAINER
    pidRuntime.acroTrainerAngleLimit = pidProfile->acro_trainer_angle_limit;
    pidRuntime.acroTrainerLookaheadTime = (float)pidProfile->acro_trainer_lookahead_ms / 1000.0f;
    pidRuntime.acroTrainerDebugAxis = pidProfile->acro_trainer_debug_axis;
    pidRuntime.acroTrainerGain = (float)pidProfile->acro_trainer_gain / 10.0f;
#endif // USE_ACRO_TRAINER

#ifdef USE_DYN_LPF
    if (pidProfile->dterm_lpf1_dyn_min_hz > 0) {
        switch (pidProfile->dterm_lpf1_type) {
        case FILTER_PT1:
            pidRuntime.dynLpfFilter = DYN_LPF_PT1;
            break;
        case FILTER_BIQUAD:
            pidRuntime.dynLpfFilter = DYN_LPF_BIQUAD;
            break;
        case FILTER_PT2:
            pidRuntime.dynLpfFilter = DYN_LPF_PT2;
            break;
        case FILTER_PT3:
            pidRuntime.dynLpfFilter = DYN_LPF_PT3;
            break;
        default:
            pidRuntime.dynLpfFilter = DYN_LPF_NONE;
            break;
        }
    } else {
        pidRuntime.dynLpfFilter = DYN_LPF_NONE;
    }
    pidRuntime.dynLpfMin = pidProfile->dterm_lpf1_dyn_min_hz;
    pidRuntime.dynLpfMax = pidProfile->dterm_lpf1_dyn_max_hz;
#endif

}

void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT
        && dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}

