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

// Board auto-align: detects which of the right-angle board mountings is in
// use from one gesture. The aircraft rests level (baseline), then the tail
// is lifted by at least 20 degrees. The resting accelerometer vector gives
// the body "up" axis, and the axis the tail-lift rotates about gives the
// pitch axis; together they fix the full body frame. Ported from Wingflight.

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ACC

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "blackbox/blackbox_params.h"

#include "config/config.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "pg/boardalignment.h"

#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"

#include "boardalignment_auto.h"

#define BOARD_AUTO_ALIGN_TIMEOUT_US             (15 * 1000000)
#define BOARD_AUTO_ALIGN_MIN_MATCHED_SAMPLES    12
#define BOARD_AUTO_ALIGN_MIN_TILT_ANGLE_DEG     20.0f

// The accelerometer reads the reaction force, so at rest it reads +1g along
// the body up axis (imu.c compares the raw reading directly to rMat[2]).
// Lifting the tail is a negative rotation about the body pitch axis.
#define BOARD_AUTO_ALIGN_TAIL_LIFT_SIGN         (-1.0f)

static const int16_t rightAngleSteps[] = { 0, 90, 180, 270 };

typedef struct {
    boardAutoAlignStatus_t status;
    timeUs_t startedAtUs;
    bool sampleValid;
    float latestSample[XYZ_AXIS_COUNT];
    float baseline[XYZ_AXIS_COUNT];
    int16_t candidateRoll;
    int16_t candidatePitch;
    int16_t candidateYaw;
} boardAutoAlignRuntime_t;

static boardAutoAlignRuntime_t autoAlign;

static bool normalizeVector(const float *src, float *dst)
{
    const float mag = sqrtf(sq(src[X]) + sq(src[Y]) + sq(src[Z]));
    if (mag < 1e-6f) {
        return false;
    }

    dst[X] = src[X] / mag;
    dst[Y] = src[Y] / mag;
    dst[Z] = src[Z] / mag;
    return true;
}

static void crossProduct(const float *a, const float *b, float *dst)
{
    dst[X] = a[Y] * b[Z] - a[Z] * b[Y];
    dst[Y] = a[Z] * b[X] - a[X] * b[Z];
    dst[Z] = a[X] * b[Y] - a[Y] * b[X];
}

static void rotateVectorByBoardAlignment(const float *src, int16_t rollDeg, int16_t pitchDeg, int16_t yawDeg, float *dst)
{
    fp_angles_t rotationAngles;
    fp_rotationMatrix_t rotationMatrix;

    rotationAngles.angles.roll  = degreesToRadians(rollDeg);
    rotationAngles.angles.pitch = degreesToRadians(pitchDeg);
    rotationAngles.angles.yaw   = degreesToRadians(yawDeg);

    buildRotationMatrix(&rotationAngles, &rotationMatrix);

    dst[X] = src[X];
    dst[Y] = src[Y];
    dst[Z] = src[Z];
    applyMatrixRotation(dst, &rotationMatrix);
}

// Builds a right-handed orthonormal body frame in sensor coordinates from the
// resting baseline and one in-gesture sample:
//   bodyZ = baseline (gravity at rest is the up axis)
//   bodyY = cross(baseline, current), i.e. the axis the tail-lift rotates about
//   bodyX = cross(bodyY, bodyZ)
// bodyX is perpendicular to both by construction, so a sloppy gesture only
// shows up as candidate jitter, which the consecutive-match counter filters.
static bool computeReferenceAxes(const float *baseline, const float *current,
    float *bodyX, float *bodyY, float *bodyZ)
{
    float baselineUnit[XYZ_AXIS_COUNT];
    float currentUnit[XYZ_AXIS_COUNT];

    if (!normalizeVector(baseline, baselineUnit) || !normalizeVector(current, currentUnit)) {
        return false;
    }

    const float dot = baselineUnit[X] * currentUnit[X] + baselineUnit[Y] * currentUnit[Y] + baselineUnit[Z] * currentUnit[Z];
    if (dot > cos_approx(DEGREES_TO_RADIANS(BOARD_AUTO_ALIGN_MIN_TILT_ANGLE_DEG))) {
        return false; // not enough tilt yet to trust the cross product
    }

    bodyZ[X] = baselineUnit[X];
    bodyZ[Y] = baselineUnit[Y];
    bodyZ[Z] = baselineUnit[Z];

    float pitchAxis[XYZ_AXIS_COUNT];
    crossProduct(baselineUnit, currentUnit, pitchAxis);
    if (!normalizeVector(pitchAxis, bodyY)) {
        return false;
    }
    bodyY[X] *= BOARD_AUTO_ALIGN_TAIL_LIFT_SIGN;
    bodyY[Y] *= BOARD_AUTO_ALIGN_TAIL_LIFT_SIGN;
    bodyY[Z] *= BOARD_AUTO_ALIGN_TAIL_LIFT_SIGN;

    crossProduct(bodyY, bodyZ, bodyX);

    return true;
}

// Finds the right-angle board alignment that rotates the observed body frame
// onto the standard one. Scoring is signed, so upside-down and tail-down
// candidates score clearly worse than the correct one instead of tying.
static bool findBestRightAngleAlignment(const float *baseline, const float *current,
    int16_t *rollOut, int16_t *pitchOut, int16_t *yawOut)
{
    float bodyX[XYZ_AXIS_COUNT];
    float bodyY[XYZ_AXIS_COUNT];
    float bodyZ[XYZ_AXIS_COUNT];

    if (!computeReferenceAxes(baseline, current, bodyX, bodyY, bodyZ)) {
        return false;
    }

    float bestScore = -1e9f;

    for (size_t i = 0; i < ARRAYLEN(rightAngleSteps); i++) {
        for (size_t j = 0; j < ARRAYLEN(rightAngleSteps); j++) {
            for (size_t k = 0; k < ARRAYLEN(rightAngleSteps); k++) {
                const int16_t roll = rightAngleSteps[i];
                const int16_t pitch = rightAngleSteps[j];
                const int16_t yaw = rightAngleSteps[k];

                float xRotated[XYZ_AXIS_COUNT];
                float yRotated[XYZ_AXIS_COUNT];
                float zRotated[XYZ_AXIS_COUNT];
                rotateVectorByBoardAlignment(bodyX, roll, pitch, yaw, xRotated);
                rotateVectorByBoardAlignment(bodyY, roll, pitch, yaw, yRotated);
                rotateVectorByBoardAlignment(bodyZ, roll, pitch, yaw, zRotated);

                // Agreement with the standard basis; 3.0 is a perfect match
                const float score = xRotated[X] + yRotated[Y] + zRotated[Z];

                if (score > bestScore) {
                    bestScore = score;
                    *rollOut = roll;
                    *pitchOut = pitch;
                    *yawOut = yaw;
                }
            }
        }
    }

    return true;
}

// Called from accUpdate() with the accelerometer sample after sensor
// alignment but before board alignment and trims.
void boardAutoAlignProcessSample(const float *sensorVector)
{
    autoAlign.latestSample[X] = sensorVector[X];
    autoAlign.latestSample[Y] = sensorVector[Y];
    autoAlign.latestSample[Z] = sensorVector[Z];
    autoAlign.sampleValid = true;

    if (autoAlign.status.state != BOARD_AUTO_ALIGN_WAITING_FOR_TAIL_LIFT) {
        return;
    }

    if (ARMING_FLAG(ARMED)) {
        autoAlign.status.state = BOARD_AUTO_ALIGN_REJECTED_ARMED;
        autoAlign.status.matchedSamples = 0;
        return;
    }

    if (cmpTimeUs(micros(), autoAlign.startedAtUs) > BOARD_AUTO_ALIGN_TIMEOUT_US) {
        autoAlign.status.state = BOARD_AUTO_ALIGN_TIMEOUT;
        autoAlign.status.matchedSamples = 0;
        return;
    }

    int16_t roll, pitch, yaw;

    if (!findBestRightAngleAlignment(autoAlign.baseline, autoAlign.latestSample, &roll, &pitch, &yaw)) {
        autoAlign.status.matchedSamples = 0;
        return;
    }

    if (autoAlign.status.matchedSamples == 0 ||
        (roll == autoAlign.candidateRoll && pitch == autoAlign.candidatePitch && yaw == autoAlign.candidateYaw)) {
        autoAlign.status.matchedSamples++;
    } else {
        autoAlign.status.matchedSamples = 1;
    }

    autoAlign.candidateRoll = roll;
    autoAlign.candidatePitch = pitch;
    autoAlign.candidateYaw = yaw;

    if (autoAlign.status.matchedSamples < BOARD_AUTO_ALIGN_MIN_MATCHED_SAMPLES) {
        return;
    }

    blackboxParamsOpBegin(BBP_SRC_TASK, BBP_TASK_BOARD_AUTO_ALIGN);
    boardAlignmentMutable()->rollDegrees = roll;
    boardAlignmentMutable()->pitchDegrees = pitch;
    boardAlignmentMutable()->yawDegrees = yaw;
    initBoardAlignment(boardAlignment());
    blackboxParamsOpEnd();
    setConfigDirty();

    autoAlign.status.rollDegrees = roll;
    autoAlign.status.pitchDegrees = pitch;
    autoAlign.status.yawDegrees = yaw;
    autoAlign.status.state = BOARD_AUTO_ALIGN_SUCCESS;
}

bool boardAutoAlignStart(void)
{
    autoAlign.status.matchedSamples = 0;

    if (ARMING_FLAG(ARMED)) {
        autoAlign.status.state = BOARD_AUTO_ALIGN_REJECTED_ARMED;
        return false;
    }

    // Samples are taken before the acc trims are applied, so an uncalibrated
    // per-axis bias would skew both the baseline and the derived pitch axis.
    if (!accHasBeenCalibrated()) {
        autoAlign.status.state = BOARD_AUTO_ALIGN_REJECTED_UNCALIBRATED;
        return false;
    }

    if (!autoAlign.sampleValid) {
        autoAlign.status.state = BOARD_AUTO_ALIGN_NO_MATCH;
        return false;
    }

    memcpy(autoAlign.baseline, autoAlign.latestSample, sizeof(autoAlign.baseline));
    autoAlign.startedAtUs = micros();
    autoAlign.status.state = BOARD_AUTO_ALIGN_WAITING_FOR_TAIL_LIFT;
    autoAlign.status.rollDegrees = boardAlignment()->rollDegrees;
    autoAlign.status.pitchDegrees = boardAlignment()->pitchDegrees;
    autoAlign.status.yawDegrees = boardAlignment()->yawDegrees;
    autoAlign.candidateRoll = 0;
    autoAlign.candidatePitch = 0;
    autoAlign.candidateYaw = 0;

    return true;
}

boardAutoAlignStatus_t boardAutoAlignGetStatus(void)
{
    boardAutoAlignStatus_t status = autoAlign.status;

    if (status.state == BOARD_AUTO_ALIGN_IDLE) {
        status.rollDegrees = boardAlignment()->rollDegrees;
        status.pitchDegrees = boardAlignment()->pitchDegrees;
        status.yawDegrees = boardAlignment()->yawDegrees;
    }

    return status;
}

#endif
