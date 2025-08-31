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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_fielddefs.h"

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/config.h"

#include "fc/rc_controls.h"
#include "fc/rc_rates.h"
#include "fc/rc.h"

#include "flight/pid.h"
#include "flight/mixer.h"
#include "flight/rescue.h"
#include "flight/trainer.h"
#include "flight/governor.h"
#include "flight/leveling.h"

#include "io/beeper.h"
#include "io/ledstrip.h"

#include "drivers/time.h"

#include "osd/osd.h"

#include "pg/rx.h"
#include "pg/adjustments.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"

#include "rc_adjustments.h"


//// Internal Types

typedef int     (*getVal_f)(int adjFunc);
typedef void    (*setVal_f)(int adjFunc, int value);

typedef struct {
    char *      cfgName;
    getVal_f    cfgGet;
    setVal_f    cfgSet;
    int         cfgMin;
    int         cfgMax;
} adjustmentConfig_t;

typedef struct {
    timeMs_t deadTime;
    timeMs_t trigTime;
    int      adjValue;
    int      chValue;
} adjustmentState_t;


//// Internal Data

static adjustmentState_t adjustmentState[MAX_ADJUSTMENT_RANGE_COUNT];

static timeMs_t       adjustmentTime   = 0;
static const char *   adjustmentName   = NULL;
static int            adjustmentFunc   = 0;
static int            adjustmentValue  = 0;



//// Get/Set Functions

static int get_ADJUSTMENT_NONE(__unused int adjFunc)
{
    return 0;
}

static void set_ADJUSTMENT_NONE(__unused int adjFunc, __unused int value)
{
    // Nothing
}

static int get_ADJUSTMENT_PID_PROFILE(__unused int adjFunc)
{
    return getCurrentPidProfileIndex() + 1;
}

static void set_ADJUSTMENT_PID_PROFILE(__unused int adjFunc, int value)
{
    changePidProfile(value - 1);
}

static int get_ADJUSTMENT_RATE_PROFILE(__unused int adjFunc)
{
    return getCurrentControlRateProfileIndex() + 1;
}

static void set_ADJUSTMENT_RATE_PROFILE(__unused int adjFunc, int value)
{
    changeControlRateProfile(value - 1);
}

static int get_ADJUSTMENT_LED_PROFILE(__unused int adjFunc)
{
#ifdef USE_LED_STRIP
    return getLedProfile() + 1;
#else
    return 0;
#endif
}

static void set_ADJUSTMENT_LED_PROFILE(__unused int adjFunc, int value)
{
#ifdef USE_LED_STRIP
    setLedProfile(value - 1);
#endif
}

static int get_ADJUSTMENT_OSD_PROFILE(__unused int adjFunc)
{
#ifdef USE_OSD_PROFILES
    return getCurrentOsdProfileIndex();
#else
    return 0;
#endif
}

static void set_ADJUSTMENT_OSD_PROFILE(__unused int adjFunc, __unused int value)
{
#ifdef USE_OSD_PROFILES
    changeOsdProfileIndex(value);
#endif
}


//// Internal functions

#define ADJ_ENTRY(id, min, max)  [ADJUSTMENT_##id] = \
    {                                                       \
        .cfgName  = #id,                                    \
        .cfgMin   = (min),                                  \
        .cfgMax   = (max),                                  \
        .cfgGet   = get_ADJUSTMENT_##id,                     \
        .cfgSet   = set_ADJUSTMENT_##id,                     \
    }

static const adjustmentConfig_t adjustmentConfigs[ADJUSTMENT_FUNCTION_COUNT] =
{
    ADJ_ENTRY(NONE,                         0, 0),

    ADJ_ENTRY(RATE_PROFILE,                 1, 6),
    ADJ_ENTRY(PID_PROFILE,                  1, 6),
    ADJ_ENTRY(LED_PROFILE,                  1, 4),
    ADJ_ENTRY(OSD_PROFILE,                  1, 3),

    ADJ_ENTRY(PITCH_RATE,                   0, CONTROL_RATE_CONFIG_SUPER_RATE_MAX),
    ADJ_ENTRY(ROLL_RATE,                    0, CONTROL_RATE_CONFIG_SUPER_RATE_MAX),
    ADJ_ENTRY(YAW_RATE,                     0, CONTROL_RATE_CONFIG_SUPER_RATE_MAX),
    ADJ_ENTRY(PITCH_RC_RATE,                1, CONTROL_RATE_CONFIG_RC_RATES_MAX),
    ADJ_ENTRY(ROLL_RC_RATE,                 1, CONTROL_RATE_CONFIG_RC_RATES_MAX),
    ADJ_ENTRY(YAW_RC_RATE,                  1, CONTROL_RATE_CONFIG_RC_RATES_MAX),
    ADJ_ENTRY(PITCH_RC_EXPO,                0, CONTROL_RATE_CONFIG_RC_EXPO_MAX),
    ADJ_ENTRY(ROLL_RC_EXPO,                 0, CONTROL_RATE_CONFIG_RC_EXPO_MAX),
    ADJ_ENTRY(YAW_RC_EXPO,                  0, CONTROL_RATE_CONFIG_RC_EXPO_MAX),

    ADJ_ENTRY(PITCH_SP_BOOST_GAIN,          0, 255),
    ADJ_ENTRY(ROLL_SP_BOOST_GAIN,           0, 255),
    ADJ_ENTRY(YAW_SP_BOOST_GAIN,            0, 255),
    ADJ_ENTRY(COLL_SP_BOOST_GAIN,           0, 255),

    ADJ_ENTRY(YAW_DYN_CEILING_GAIN,         0, 250),
    ADJ_ENTRY(YAW_DYN_DEADBAND_GAIN,        0, 250),
    ADJ_ENTRY(YAW_DYN_DEADBAND_FILTER,      0, 250),

    ADJ_ENTRY(PITCH_P_GAIN,                 0, PID_GAIN_MAX),
    ADJ_ENTRY(PITCH_I_GAIN,                 0, PID_GAIN_MAX),
    ADJ_ENTRY(PITCH_D_GAIN,                 0, PID_GAIN_MAX),
    ADJ_ENTRY(PITCH_F_GAIN,                 0, PID_GAIN_MAX),
    ADJ_ENTRY(ROLL_P_GAIN,                  0, PID_GAIN_MAX),
    ADJ_ENTRY(ROLL_I_GAIN,                  0, PID_GAIN_MAX),
    ADJ_ENTRY(ROLL_D_GAIN,                  0, PID_GAIN_MAX),
    ADJ_ENTRY(ROLL_F_GAIN,                  0, PID_GAIN_MAX),
    ADJ_ENTRY(YAW_P_GAIN,                   0, PID_GAIN_MAX),
    ADJ_ENTRY(YAW_I_GAIN,                   0, PID_GAIN_MAX),
    ADJ_ENTRY(YAW_D_GAIN,                   0, PID_GAIN_MAX),
    ADJ_ENTRY(YAW_F_GAIN,                   0, PID_GAIN_MAX),

    ADJ_ENTRY(PITCH_B_GAIN,                 0, PID_GAIN_MAX),
    ADJ_ENTRY(ROLL_B_GAIN,                  0, PID_GAIN_MAX),
    ADJ_ENTRY(YAW_B_GAIN,                   0, PID_GAIN_MAX),

    ADJ_ENTRY(PITCH_O_GAIN,                 0, PID_GAIN_MAX),
    ADJ_ENTRY(ROLL_O_GAIN,                  0, PID_GAIN_MAX),

    ADJ_ENTRY(YAW_CW_GAIN,                  25, 250),
    ADJ_ENTRY(YAW_CCW_GAIN,                 25, 250),
    ADJ_ENTRY(YAW_CYCLIC_FF,                0, 250),
    ADJ_ENTRY(YAW_COLLECTIVE_FF,            0, 250),

    ADJ_ENTRY(PITCH_COLLECTIVE_FF,          0, 250),

    ADJ_ENTRY(PITCH_GYRO_CUTOFF,            0, 250),
    ADJ_ENTRY(ROLL_GYRO_CUTOFF,             0, 250),
    ADJ_ENTRY(YAW_GYRO_CUTOFF,              0, 250),

    ADJ_ENTRY(PITCH_DTERM_CUTOFF,           0, 250),
    ADJ_ENTRY(ROLL_DTERM_CUTOFF,            0, 250),
    ADJ_ENTRY(YAW_DTERM_CUTOFF,             0, 250),

    ADJ_ENTRY(YAW_PRECOMP_CUTOFF,           0, 250),

    ADJ_ENTRY(INERTIA_PRECOMP_GAIN,         0, 250),
    ADJ_ENTRY(INERTIA_PRECOMP_CUTOFF,       0, 250),

    ADJ_ENTRY(CROSS_COUPLING_GAIN,          0, 250),
    ADJ_ENTRY(CROSS_COUPLING_RATIO,         0, 200),
    ADJ_ENTRY(CROSS_COUPLING_CUTOFF,        1, 250),

    ADJ_ENTRY(ANGLE_LEVEL_GAIN,             0, 200),
    ADJ_ENTRY(HORIZON_LEVEL_GAIN,           0, 200),
    ADJ_ENTRY(ACRO_TRAINER_GAIN,            25, 255),

    ADJ_ENTRY(RESCUE_CLIMB_COLLECTIVE,      0, 1000),
    ADJ_ENTRY(RESCUE_HOVER_COLLECTIVE,      0, 1000),
    ADJ_ENTRY(RESCUE_HOVER_ALTITUDE,        0, 2500),
    ADJ_ENTRY(RESCUE_ALT_P_GAIN,            0, 1000),
    ADJ_ENTRY(RESCUE_ALT_I_GAIN,            0, 1000),
    ADJ_ENTRY(RESCUE_ALT_D_GAIN,            0, 1000),

    ADJ_ENTRY(GOV_GAIN,                     0, 250),
    ADJ_ENTRY(GOV_P_GAIN,                   0, 250),
    ADJ_ENTRY(GOV_I_GAIN,                   0, 250),
    ADJ_ENTRY(GOV_D_GAIN,                   0, 250),
    ADJ_ENTRY(GOV_F_GAIN,                   0, 250),
    ADJ_ENTRY(GOV_TTA_GAIN,                 0, 250),
    ADJ_ENTRY(GOV_CYCLIC_FF,                0, 250),
    ADJ_ENTRY(GOV_COLLECTIVE_FF,            0, 250),

    ADJ_ENTRY(ACC_TRIM_PITCH,               -300, 300),
    ADJ_ENTRY(ACC_TRIM_ROLL,                -300, 300),
};


static void blackboxAdjustmentEvent(int adjFunc, int value)
{
#ifndef USE_BLACKBOX
    UNUSED(adjFunc);
    UNUSED(value);
#else
    if (blackboxConfig()->device) {
        flightLogEvent_inflightAdjustment_t eventData;
        eventData.adjustmentFunction = adjFunc;
        eventData.newValue = value;
        eventData.floatFlag = false;
        blackboxLogEvent(FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT, (flightLogEventData_t*)&eventData);
    }
#endif
}

#define ADJUSTMENT_LATENCY_MS 3000

static void updateAdjustmentData(int adjFunc, int value)
{
    const timeMs_t now = millis();

    if (adjFunc != ADJUSTMENT_NONE &&
#ifdef USE_OSD_PROFILES
        adjFunc != ADJUSTMENT_OSD_PROFILE &&
#endif
        adjFunc != ADJUSTMENT_PID_PROFILE &&
        adjFunc != ADJUSTMENT_RATE_PROFILE)
    {
        adjustmentTime   = now;
        adjustmentName   = adjustmentConfigs[adjFunc].cfgName;
        adjustmentFunc   = adjFunc;
        adjustmentValue  = value;
    }

    if (cmp32(now, adjustmentTime) > ADJUSTMENT_LATENCY_MS) {
        adjustmentName = NULL;
    }
}

#define REPEAT_DELAY     500
#define TRIGGER_DELAY    100

void processRcAdjustments(void)
{
    bool changed = false;

    if (rxIsReceivingSignal())
    {
        for (int index = 0; index < MAX_ADJUSTMENT_RANGE_COUNT; index++)
        {
            const adjustmentRange_t * adjRange = adjustmentRanges(index);
            const uint8_t adjFunc = adjRange->function;

            if (adjRange->enaChannel == 0xff || isRangeActive(adjRange->enaChannel, &adjRange->enaRange))
            {
                const adjustmentConfig_t * adjConfig = &adjustmentConfigs[adjFunc];
                adjustmentState_t * adjState = &adjustmentState[index];
                const timeMs_t now = millis();

                int adjval = adjState->adjValue;

                if (cmp32(now, adjState->deadTime) < 0)
                    continue;

                const int chValue = rcInput[adjRange->adjChannel + CONTROL_CHANNEL_COUNT];

                // Stepped adjustment
                if (adjRange->adjStep) {
                    if (abs(chValue - adjState->chValue) > 2) {
                        adjState->trigTime = now + TRIGGER_DELAY;
                        adjState->chValue = chValue;
                        continue;
                    }
                    if (cmp32(now, adjState->trigTime) < 0) {
                        continue;
                    }
                    if (isRangeActive(adjRange->adjChannel, &adjRange->adjRange1)) {
                        adjval = adjConfig->cfgGet(adjFunc) - adjRange->adjStep;
                    }
                    else if (isRangeActive(adjRange->adjChannel, &adjRange->adjRange2)) {
                        adjval = adjConfig->cfgGet(adjFunc) + adjRange->adjStep;
                    }
                    else {
                        continue;
                    }
                }
                // Continuous adjustment
                else {
                    const int rangeLower = STEP_TO_CHANNEL_VALUE(adjRange->adjRange1.startStep);
                    const int rangeUpper = STEP_TO_CHANNEL_VALUE(adjRange->adjRange1.endStep);
                    const int rangeWidth = rangeUpper - rangeLower;
                    const int valueWidth = adjRange->adjMax - adjRange->adjMin;

                    if (rangeWidth > 0 && valueWidth > 0) {
                        const int rangeMargin = MAX(5, rangeWidth / (valueWidth * 2));
                        if (chValue > rangeLower - rangeMargin && chValue < rangeUpper + rangeMargin) {
                            const int offset = rangeWidth / 2;
                            adjval = adjRange->adjMin + ((chValue - rangeLower) * valueWidth + offset) / rangeWidth;
                        }
                    }
                }

                adjval = constrain(adjval, adjConfig->cfgMin, adjConfig->cfgMax);
                adjval = constrain(adjval, adjRange->adjMin, adjRange->adjMax);

                if (adjval != adjState->adjValue) {
                    adjConfig->cfgSet(adjFunc, adjval);

                    updateAdjustmentData(adjFunc, adjval);
                    blackboxAdjustmentEvent(adjFunc, adjval);

                    // PID profile change does it's own confirmation, no of beeps eq profile no,
                    // a single beep here will kill that.
                    if (adjFunc != ADJUSTMENT_PID_PROFILE) {
                      beeperConfirmationBeeps(1);
                    }
                    setConfigDirty();

                    adjState->deadTime = now + REPEAT_DELAY;
                    adjState->adjValue = adjval;

                    changed = true;
                }
            }
        }
    }

    if (!changed) {
        updateAdjustmentData(ADJUSTMENT_NONE, 0);
    }
}

const char * getAdjustmentsRangeName(void)
{
    return adjustmentName;
}

int getAdjustmentsRangeFunc(void)
{
    return adjustmentFunc;
}

int getAdjustmentsRangeValue(void)
{
    return adjustmentValue;
}

INIT_CODE void adjustmentRangeInit(void)
{
    for (int index = 0; index < MAX_ADJUSTMENT_RANGE_COUNT; index++) {
        adjustmentRangeReset(index);
    }
}

INIT_CODE void adjustmentRangeReset(int index)
{
    const int adjFunc = adjustmentRanges(index)->function;
    const adjustmentConfig_t * adjConfig = &adjustmentConfigs[adjFunc];

    adjustmentState[index].adjValue = adjConfig->cfgGet(adjFunc);
    adjustmentState[index].deadTime = 0;
    adjustmentState[index].trigTime = 0;
    adjustmentState[index].chValue  = 0;
}
