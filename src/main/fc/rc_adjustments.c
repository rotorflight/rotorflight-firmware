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
#include <math.h>

#include "platform.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_fielddefs.h"

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/time.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc.h"

#include "flight/pid.h"
#include "flight/governor.h"

#include "io/beeper.h"
#include "io/ledstrip.h"
#include "io/motors.h"

#include "osd/osd.h"

#include "pg/rx.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/rx.h"

#include "rc_adjustments.h"

PG_REGISTER_ARRAY(adjustmentRange_t, MAX_ADJUSTMENT_RANGE_COUNT, adjustmentRanges, PG_ADJUSTMENT_RANGE_CONFIG, 3);

static adjustmentState_t adjustmentStates[MAX_ADJUSTMENT_RANGE_COUNT];

#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
static timeMs_t       osdAdjustmentTime   = 0;
static const char *   osdAdjustmentName   = NULL;
static int            osdAdjustmentValue  = 0;
#endif

#define ADJ_CONFIG(id, type, min, max)  [ADJUSTMENT_##id] = \
    {                                                       \
        .cfgName  = #id,                                    \
        .cfgType  = ADJUSTMENT_TYPE_##type,                 \
        .cfgMin   = (min),                                  \
        .cfgMax   = (max),                                  \
    }

static const adjustmentConfig_t adjustmentConfigs[ADJUSTMENT_FUNCTION_COUNT] =
{
    ADJ_CONFIG(NONE,               NONE,  0, 0),

    ADJ_CONFIG(PITCH_RATE,         RATE,  0, CONTROL_RATE_CONFIG_RATE_MAX),
    ADJ_CONFIG(ROLL_RATE,          RATE,  0, CONTROL_RATE_CONFIG_RATE_MAX),
    ADJ_CONFIG(YAW_RATE,           RATE,  0, CONTROL_RATE_CONFIG_RATE_MAX),
    ADJ_CONFIG(PITCH_RC_RATE,      RATE,  1, CONTROL_RATE_CONFIG_RC_RATES_MAX),
    ADJ_CONFIG(ROLL_RC_RATE,       RATE,  1, CONTROL_RATE_CONFIG_RC_RATES_MAX),
    ADJ_CONFIG(YAW_RC_RATE,        RATE,  1, CONTROL_RATE_CONFIG_RC_RATES_MAX),
    ADJ_CONFIG(PITCH_RC_EXPO,      RATE,  0, CONTROL_RATE_CONFIG_RC_EXPO_MAX),
    ADJ_CONFIG(ROLL_RC_EXPO,       RATE,  0, CONTROL_RATE_CONFIG_RC_EXPO_MAX),
    ADJ_CONFIG(YAW_RC_EXPO,        RATE,  0, CONTROL_RATE_CONFIG_RC_EXPO_MAX),

    ADJ_CONFIG(PITCH_P_GAIN,       PID,   0, 200),
    ADJ_CONFIG(PITCH_I_GAIN,       PID,   0, 200),
    ADJ_CONFIG(PITCH_D_GAIN,       PID,   0, 200),
    ADJ_CONFIG(PITCH_F_GAIN,       PID,   0, 200),
    ADJ_CONFIG(ROLL_P_GAIN,        PID,   0, 200),
    ADJ_CONFIG(ROLL_I_GAIN,        PID,   0, 200),
    ADJ_CONFIG(ROLL_D_GAIN,        PID,   0, 200),
    ADJ_CONFIG(ROLL_F_GAIN,        PID,   0, 200),
    ADJ_CONFIG(YAW_P_GAIN,         PID,   0, 200),
    ADJ_CONFIG(YAW_I_GAIN,         PID,   0, 200),
    ADJ_CONFIG(YAW_D_GAIN,         PID,   0, 200),
    ADJ_CONFIG(YAW_F_GAIN,         PID,   0, 200),

    ADJ_CONFIG(YAW_CENTER,         PID,  -250, 250),
    ADJ_CONFIG(YAW_CYCLIC_FF,      PID,   0, 1000),
    ADJ_CONFIG(YAW_COLLECTIVE_FF,  PID,   0, 1000),
    ADJ_CONFIG(YAW_IMPULSE_FF,     PID,   0, 1000),

    ADJ_CONFIG(RESCUE_COLLECTIVE,  PID,   0, 1000),

    ADJ_CONFIG(ANGLE_LEVEL_GAIN,   PID,   0, 200),
    ADJ_CONFIG(HORIZON_LEVEL_GAIN, PID,   0, 200),
    ADJ_CONFIG(ACRO_TRAINER_GAIN,  PID,  25, 255),

    ADJ_CONFIG(GOV_GAIN,           GOV,   0, 500),
    ADJ_CONFIG(GOV_P_GAIN,         GOV,   0, 500),
    ADJ_CONFIG(GOV_I_GAIN,         GOV,   0, 500),
    ADJ_CONFIG(GOV_D_GAIN,         GOV,   0, 500),
    ADJ_CONFIG(GOV_F_GAIN,         GOV,   0, 500),
    ADJ_CONFIG(GOV_CYCLIC_FF,      GOV,   0, 500),
    ADJ_CONFIG(GOV_COLLECTIVE_FF,  GOV,   0, 500),

    ADJ_CONFIG(OSD_PROFILE,        NONE,  1,  3),
    ADJ_CONFIG(LED_PROFILE,        NONE,  0,  2),
    ADJ_CONFIG(PID_PROFILE,        NONE,  0,  5),
    ADJ_CONFIG(RATE_PROFILE,       NONE,  0,  5),
};


static int getAdjustmentValue(uint8_t adjFunc)
{
    int value = 0;

    switch (adjFunc) {
    case ADJUSTMENT_PITCH_RATE:
        value = currentControlRateProfile->rates[FD_PITCH];
        break;
    case ADJUSTMENT_ROLL_RATE:
        value = currentControlRateProfile->rates[FD_ROLL];
        break;
    case ADJUSTMENT_YAW_RATE:
        value = currentControlRateProfile->rates[FD_YAW];
        break;
    case ADJUSTMENT_PITCH_RC_RATE:
        value = currentControlRateProfile->rcRates[FD_PITCH];
        break;
    case ADJUSTMENT_ROLL_RC_RATE:
        value = currentControlRateProfile->rcRates[FD_ROLL];
        break;
    case ADJUSTMENT_YAW_RC_RATE:
        value = currentControlRateProfile->rcRates[FD_YAW];
        break;
    case ADJUSTMENT_PITCH_RC_EXPO:
        value = currentControlRateProfile->rcExpo[FD_PITCH];
        break;
    case ADJUSTMENT_ROLL_RC_EXPO:
        value = currentControlRateProfile->rcExpo[FD_ROLL];
        break;
    case ADJUSTMENT_YAW_RC_EXPO:
        value = currentControlRateProfile->rcExpo[FD_YAW];
        break;
    case ADJUSTMENT_PITCH_P_GAIN:
        value = currentPidProfile->pid[PID_PITCH].P;
        break;
    case ADJUSTMENT_ROLL_P_GAIN:
        value = currentPidProfile->pid[PID_ROLL].P;
        break;
    case ADJUSTMENT_YAW_P_GAIN:
        value = currentPidProfile->pid[PID_YAW].P;
        break;
    case ADJUSTMENT_PITCH_I_GAIN:
        value = currentPidProfile->pid[PID_PITCH].I;
        break;
    case ADJUSTMENT_ROLL_I_GAIN:
        value = currentPidProfile->pid[PID_ROLL].I;
        break;
    case ADJUSTMENT_YAW_I_GAIN:
        value = currentPidProfile->pid[PID_YAW].I;
        break;
    case ADJUSTMENT_PITCH_D_GAIN:
        value = currentPidProfile->pid[PID_PITCH].D;
        break;
    case ADJUSTMENT_ROLL_D_GAIN:
        value = currentPidProfile->pid[PID_ROLL].D;
        break;
    case ADJUSTMENT_YAW_D_GAIN:
        value = currentPidProfile->pid[PID_YAW].D;
        break;
    case ADJUSTMENT_PITCH_F_GAIN:
        value = currentPidProfile->pid[PID_PITCH].F;
        break;
    case ADJUSTMENT_ROLL_F_GAIN:
        value = currentPidProfile->pid[PID_ROLL].F;
        break;
    case ADJUSTMENT_YAW_F_GAIN:
        value = currentPidProfile->pid[PID_YAW].F;
        break;
    case ADJUSTMENT_YAW_CENTER:
        value = currentPidProfile->yaw_center_offset;
        break;
    case ADJUSTMENT_YAW_CYCLIC_FF:
        value = currentPidProfile->yaw_cyclic_ff_gain;
        break;
    case ADJUSTMENT_YAW_COLLECTIVE_FF:
        value = currentPidProfile->yaw_collective_ff_gain;
        break;
    case ADJUSTMENT_YAW_IMPULSE_FF:
        value = currentPidProfile->yaw_collective_ff_impulse_gain;
        break;
    case ADJUSTMENT_RESCUE_COLLECTIVE:
        value = currentPidProfile->rescue_collective;
        break;
    case ADJUSTMENT_ANGLE_LEVEL_GAIN:
        value = currentPidProfile->angle_level_strength;
        break;
    case ADJUSTMENT_HORIZON_LEVEL_GAIN:
        value = currentPidProfile->horizon_level_strength;
        break;
    case ADJUSTMENT_ACRO_TRAINER_GAIN:
        value = currentPidProfile->acro_trainer_gain;
        break;
    case ADJUSTMENT_GOV_GAIN:
        value = governorConfig()->gov_gain;
        break;
    case ADJUSTMENT_GOV_P_GAIN:
        value = governorConfig()->gov_p_gain;
        break;
    case ADJUSTMENT_GOV_I_GAIN:
        value = governorConfig()->gov_i_gain;
        break;
    case ADJUSTMENT_GOV_D_GAIN:
        value = governorConfig()->gov_d_gain;
        break;
    case ADJUSTMENT_GOV_F_GAIN:
        value = governorConfig()->gov_f_gain;
        break;
    case ADJUSTMENT_GOV_CYCLIC_FF:
        value = governorConfig()->gov_cyclic_ff_weight;
        break;
    case ADJUSTMENT_GOV_COLLECTIVE_FF:
        value = governorConfig()->gov_collective_ff_weight;
        break;
    case ADJUSTMENT_RATE_PROFILE:
        value = getCurrentControlRateProfileIndex();
        break;
    case ADJUSTMENT_PID_PROFILE:
        value = getCurrentPidProfileIndex();
        break;
    case ADJUSTMENT_OSD_PROFILE:
#ifdef USE_OSD_PROFILES
        value = getCurrentOsdProfileIndex();
#endif
        break;
    case ADJUSTMENT_LED_PROFILE:
#ifdef USE_LED_STRIP
        value = getLedProfile();
#endif
        break;
    }

    return value;
}

static void setAdjustmentValue(uint8_t adjFunc, int value)
{
    switch (adjFunc) {
    case ADJUSTMENT_PITCH_RATE:
        currentControlRateProfile->rates[FD_PITCH] = value;
        break;
    case ADJUSTMENT_ROLL_RATE:
        currentControlRateProfile->rates[FD_ROLL] = value;
        break;
    case ADJUSTMENT_YAW_RATE:
        currentControlRateProfile->rates[FD_YAW] = value;
        break;
    case ADJUSTMENT_PITCH_RC_RATE:
        currentControlRateProfile->rcRates[FD_PITCH] = value;
        break;
    case ADJUSTMENT_ROLL_RC_RATE:
        currentControlRateProfile->rcRates[FD_ROLL] = value;
        break;
    case ADJUSTMENT_YAW_RC_RATE:
        currentControlRateProfile->rcRates[FD_YAW] = value;
        break;
    case ADJUSTMENT_PITCH_RC_EXPO:
        currentControlRateProfile->rcExpo[FD_PITCH] = value;
        break;
    case ADJUSTMENT_ROLL_RC_EXPO:
        currentControlRateProfile->rcExpo[FD_ROLL] = value;
        break;
    case ADJUSTMENT_YAW_RC_EXPO:
        currentControlRateProfile->rcExpo[FD_YAW] = value;
        break;
    case ADJUSTMENT_PITCH_P_GAIN:
        currentPidProfile->pid[PID_PITCH].P = value;
        break;
    case ADJUSTMENT_ROLL_P_GAIN:
        currentPidProfile->pid[PID_ROLL].P = value;
        break;
    case ADJUSTMENT_YAW_P_GAIN:
        currentPidProfile->pid[PID_YAW].P = value;
        break;
    case ADJUSTMENT_PITCH_I_GAIN:
        currentPidProfile->pid[PID_PITCH].I = value;
        break;
    case ADJUSTMENT_ROLL_I_GAIN:
        currentPidProfile->pid[PID_ROLL].I = value;
        break;
    case ADJUSTMENT_YAW_I_GAIN:
        currentPidProfile->pid[PID_YAW].I = value;
        break;
    case ADJUSTMENT_PITCH_D_GAIN:
        currentPidProfile->pid[PID_PITCH].D = value;
        break;
    case ADJUSTMENT_ROLL_D_GAIN:
        currentPidProfile->pid[PID_ROLL].D = value;
        break;
    case ADJUSTMENT_YAW_D_GAIN:
        currentPidProfile->pid[PID_YAW].D = value;
        break;
    case ADJUSTMENT_PITCH_F_GAIN:
        currentPidProfile->pid[PID_PITCH].F = value;
        break;
    case ADJUSTMENT_ROLL_F_GAIN:
        currentPidProfile->pid[PID_ROLL].F = value;
        break;
    case ADJUSTMENT_YAW_F_GAIN:
        currentPidProfile->pid[PID_YAW].F = value;
        break;
    case ADJUSTMENT_YAW_CENTER:
        currentPidProfile->yaw_center_offset = value;
        break;
    case ADJUSTMENT_YAW_CYCLIC_FF:
        currentPidProfile->yaw_cyclic_ff_gain = value;
        break;
    case ADJUSTMENT_YAW_COLLECTIVE_FF:
        currentPidProfile->yaw_collective_ff_gain = value;
        break;
    case ADJUSTMENT_YAW_IMPULSE_FF:
        currentPidProfile->yaw_collective_ff_impulse_gain = value;
        break;
    case ADJUSTMENT_RESCUE_COLLECTIVE:
        currentPidProfile->rescue_collective = value;
        break;
    case ADJUSTMENT_ANGLE_LEVEL_GAIN:
        currentPidProfile->angle_level_strength = value;
        break;
    case ADJUSTMENT_HORIZON_LEVEL_GAIN:
        currentPidProfile->horizon_level_strength = value;
        break;
    case ADJUSTMENT_ACRO_TRAINER_GAIN:
        currentPidProfile->acro_trainer_gain = value;
        break;
    case ADJUSTMENT_GOV_GAIN:
        governorConfigMutable()->gov_gain = value;
        break;
    case ADJUSTMENT_GOV_P_GAIN:
        governorConfigMutable()->gov_p_gain = value;
        break;
    case ADJUSTMENT_GOV_I_GAIN:
        governorConfigMutable()->gov_i_gain = value;
        break;
    case ADJUSTMENT_GOV_D_GAIN:
        governorConfigMutable()->gov_d_gain = value;
        break;
    case ADJUSTMENT_GOV_F_GAIN:
        governorConfigMutable()->gov_f_gain = value;
        break;
    case ADJUSTMENT_GOV_CYCLIC_FF:
        governorConfigMutable()->gov_cyclic_ff_weight = value;
        break;
    case ADJUSTMENT_GOV_COLLECTIVE_FF:
        governorConfigMutable()->gov_collective_ff_weight = value;
        break;
    case ADJUSTMENT_RATE_PROFILE:
        changeControlRateProfile(value);
        break;
    case ADJUSTMENT_PID_PROFILE:
        changePidProfile(value);
        break;
    case ADJUSTMENT_OSD_PROFILE:
#ifdef USE_OSD_PROFILES
        changeOsdProfileIndex(value);
#endif
        break;
    case ADJUSTMENT_LED_PROFILE:
#ifdef USE_LED_STRIP
        setLedProfile(value);
#endif
        break;
    }
}

static void blackboxAdjustmentEvent(uint8_t adjFunc, int newValue)
{
#ifndef USE_BLACKBOX
    UNUSED(adjFunc);
    UNUSED(newValue);
#else
    if (blackboxConfig()->device) {
        flightLogEvent_inflightAdjustment_t eventData;
        eventData.adjustmentFunction = adjFunc;
        eventData.newValue = newValue;
        eventData.floatFlag = false;
        blackboxLogEvent(FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT, (flightLogEventData_t*)&eventData);
    }
#endif
}

#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)

#define OSD_DISPLAY_LATENCY_MS 2500

static void updateOsdAdjustmentData(uint8_t adjFunc, int newValue)
{
    const timeMs_t now = millis();

    if (adjFunc != ADJUSTMENT_NONE &&
#ifdef USE_OSD_PROFILES
        adjFunc != ADJUSTMENT_OSD_PROFILE &&
#endif
        adjFunc != ADJUSTMENT_PID_PROFILE &&
        adjFunc != ADJUSTMENT_RATE_PROFILE)
    {
        osdAdjustmentTime = now;
        osdAdjustmentName = adjustmentConfigs[adjFunc].cfgName;
        osdAdjustmentValue = newValue;
    }

    if (cmp32(now, osdAdjustmentTime) > OSD_DISPLAY_LATENCY_MS) {
        osdAdjustmentName = NULL;
        osdAdjustmentValue = 0;
    }
}
#endif

#define REPEAT_STEP_FREQ  (1000 / 2)

void processRcAdjustments(void)
{
    uint8_t changed = 0;

    if (rxIsReceivingSignal())
    {
        for (int index = 0; index < MAX_ADJUSTMENT_RANGE_COUNT; index++)
        {
            const adjustmentRange_t *adjRange = adjustmentRanges(index);
            const adjustmentConfig_t *adjConfig = &adjustmentConfigs[adjRange->adjFunction];
            adjustmentState_t *adjState = &adjustmentStates[index];

            if (isRangeActive(adjRange->enaChannel, &adjRange->enaRange)) {
                uint8_t adjFunc = adjRange->adjFunction;
                timeMs_t now = millis();

                if (adjState->timer && cmp32(now, adjState->timer) < 0)
                    continue;

                adjState->timer = 0;

                int chValue = rcData[adjRange->adjChannel + NON_AUX_CHANNEL_COUNT];
                int newValue = 0;

                if (adjRange->adjStep) {
                    int delta = chValue - rxConfig()->midrc;
                    int steps = 0;
                    if (delta > 200)
                        steps = adjRange->adjStep;
                    else if (delta < -200)
                        steps = -adjRange->adjStep;

                    newValue = getAdjustmentValue(adjFunc) + steps;
                }
                else {
                    int chRange = adjRange->adjMax - adjRange->adjMin;
                    int chStart = 1000 - (500 / chRange);
                    newValue = adjRange->adjMin + (chRange * (chValue - chStart)) / 1000;
                }

                newValue = constrain(newValue, adjRange->adjMin, adjRange->adjMax);
                newValue = constrain(newValue, adjConfig->cfgMin, adjConfig->cfgMax);

                if (newValue != adjState->value) {

                    setAdjustmentValue(adjFunc, newValue);

                    setConfigDirty();
                    beeperConfirmationBeeps(1);
                    blackboxAdjustmentEvent(adjFunc, newValue);
#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
                    updateOsdAdjustmentData(adjFunc, newValue);
#endif
                    adjState->timer = now + REPEAT_STEP_FREQ;
                    adjState->value = newValue;

                    changed |= adjConfig->cfgType;
                }
            }
        }
    }

    if (changed & ADJUSTMENT_TYPE_PID) {
        pidInitConfig(currentPidProfile);
    }
    if (changed & ADJUSTMENT_TYPE_GOV) {
        governorUpdateGains();
    }

#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
    if (!changed) {
        updateOsdAdjustmentData(ADJUSTMENT_NONE, 0);
    }
#endif
}

#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
const char *getAdjustmentsRangeName(void)
{
    return osdAdjustmentName;
}

int getAdjustmentsRangeValue(void)
{
    return osdAdjustmentValue;
}
#endif

void adjustmentRangeInit(void)
{
    memset(adjustmentStates, 0, sizeof(adjustmentStates));
}

void adjustmentRangeReset(int index)
{
    memset(&adjustmentStates[index], 0, sizeof(adjustmentState_t));
}

