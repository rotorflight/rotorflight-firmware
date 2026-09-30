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
#include <stdint.h>

#include "common/axis.h"

// Statistics run at this rate: each sample is the mean of the PID loops in its block.
#define TA_SAMPLE_HZ            100

// Feed-forward match: setpoint delayed by 0..TA_LAG_MAX samples (10 ms each) against gyro.
#define TA_LAG_MAX              25

// Setpoint bands, deg/s. Below TA_SP_MIN the ratio is dominated by trim and noise.
#define TA_SP_MIN               40.0f
#define TA_SP_MID               100.0f
#define TA_SP_HIGH              200.0f

// |Collective| bands (0..1). The governor holds the throttle, so collective is what
// changes the rotor load and the cyclic/tail authority.
#define TA_COLL_LOW             0.25f
#define TA_COLL_HIGH            0.5f

// Stick-release bounce detection, deg/s and samples
#define TA_STICK_CENTER         5.0f
#define TA_RELEASE_SP_MIN       60.0f
#define TA_RELEASE_GYRO_MIN     60.0f
#define TA_RELEASE_PEAK_TAIL    10      // keep tracking the peak rate this long after release
#define TA_RELEASE_MIN_QUIET    15      // stick must stay centred this long for the release to count
#define TA_RELEASE_WINDOW       80      // rebound is searched this long after release
#define TA_REBOUND_BIG          0.15f   // rebound fraction counted as a "big" bounce

// Full stick: |deflection| at or above this
#define TA_FULL_STICK           0.9f

// Samples count as "flying" only if the body rotated this fast (any axis, so a pirouette counts)
// within the last TA_MOTION_HOLD samples. Excludes spooled-up stick checks on the ground.
#define TA_MOTION_RATE          40.0f
#define TA_MOTION_HOLD          200

typedef enum {
    TA_SP_BAND_LOW = 0,         // TA_SP_MIN .. TA_SP_MID
    TA_SP_BAND_MID,             // TA_SP_MID .. TA_SP_HIGH
    TA_SP_BAND_HIGH,            // TA_SP_HIGH and above
    TA_SP_BAND_COUNT
} taSpBand_e;

typedef enum {
    TA_COLL_BAND_LOW = 0,
    TA_COLL_BAND_MID,
    TA_COLL_BAND_HIGH,
    TA_COLL_BAND_COUNT
} taCollBand_e;

// One 10 ms sample
typedef struct {
    bool valid;                 // spooled up, airborne, plain rate flight (no leveling/rescue)
    float collective;           // |collective| 0..1
    float setpoint[XYZ_AXIS_COUNT];     // PID setpoint, deg/s
    float gyro[XYZ_AXIS_COUNT];         // deg/s
    float iterm[XYZ_AXIS_COUNT];        // I output, surface units
    float feedback[XYZ_AXIS_COUNT];     // P+I+D+B output (no F, precomp or offset), 1 = full travel
    float deflection[XYZ_AXIS_COUNT];   // stick -1..1
    bool saturated[XYZ_AXIS_COUNT];
} tuneAdvisorSample_t;

typedef struct {
    float gain;                 // gyro / setpoint
    uint32_t count;
} taBand_t;

// Per-axis results, as reported over MSP
typedef struct {
    // Feed-forward match over TA_SP_MIN..TA_SP_HIGH, unsaturated, at the best lag
    uint32_t ffCount;
    float ffGain;               // gyro / setpoint. 1.0 = F matches the airframe
    float ffCorr;               // -1..1, how consistent the ratio is
    uint16_t ffLagMs;
    taBand_t spBand[TA_SP_BAND_COUNT];
    taBand_t collBand[TA_COLL_BAND_COUNT];

    // Full stick
    uint32_t fullCount;
    uint32_t fullSatCount;
    float fullRatio;            // rate reached / rate requested at full stick
    float fullMaxRate;          // deg/s

    // Stick releases
    uint16_t releases;
    uint16_t bigRebounds;       // rebound >= TA_REBOUND_BIG of the peak rate
    float meanRebound;          // rebound / peak rate
    float meanOvershoot;        // peak rate / peak setpoint
    float meanCounter;          // controller's peak counter-output (P+I+D+B) after release
    float meanIterm;            // I at release, positive = opposing the roll that just ended
} tuneAdvisorAxis_t;

void tuneAdvisorReset(void);

// One 10 ms sample. Exposed for unit tests; the flight code calls tuneAdvisorUpdate().
void tuneAdvisorProcessSample(const tuneAdvisorSample_t *sample);

// Every PID loop, after pidController()
void tuneAdvisorUpdate(float dT);

void tuneAdvisorGetAxis(int axis, tuneAdvisorAxis_t *out);
uint32_t tuneAdvisorGetValidSamples(void);
bool tuneAdvisorIsCollecting(void);
