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
#include "flight/governor.h"

#include "io/beeper.h"
#include "io/ledstrip.h"

#include "drivers/time.h"

#include "osd/osd.h"

#include "pg/rx.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/rx.h"

#include "rc_adjustments.h"

PG_REGISTER_ARRAY(adjustmentRange_t, MAX_ADJUSTMENT_RANGE_COUNT, adjustmentRanges, PG_ADJUSTMENT_RANGE_CONFIG, 3);

static adjustmentState_t adjustmentState[MAX_ADJUSTMENT_RANGE_COUNT];

static timeMs_t       adjustmentTime   = 0;
static const char *   adjustmentName   = NULL;
static int            adjustmentFunc   = 0;
static int            adjustmentValue  = 0;


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

    ADJ_CONFIG(PITCH_RATE,         RATE,  0, CONTROL_RATE_CONFIG_SUPER_RATE_MAX),
    ADJ_CONFIG(ROLL_RATE,          RATE,  0, CONTROL_RATE_CONFIG_SUPER_RATE_MAX),
    ADJ_CONFIG(YAW_RATE,           RATE,  0, CONTROL_RATE_CONFIG_SUPER_RATE_MAX),
    ADJ_CONFIG(PITCH_RC_RATE,      RATE,  1, CONTROL_RATE_CONFIG_RC_RATES_MAX),
    ADJ_CONFIG(ROLL_RC_RATE,       RATE,  1, CONTROL_RATE_CONFIG_RC_RATES_MAX),
    ADJ_CONFIG(YAW_RC_RATE,        RATE,  1, CONTROL_RATE_CONFIG_RC_RATES_MAX),
    ADJ_CONFIG(PITCH_RC_EXPO,      RATE,  0, CONTROL_RATE_CONFIG_RC_EXPO_MAX),
    ADJ_CONFIG(ROLL_RC_EXPO,       RATE,  0, CONTROL_RATE_CONFIG_RC_EXPO_MAX),
    ADJ_CONFIG(YAW_RC_EXPO,        RATE,  0, CONTROL_RATE_CONFIG_RC_EXPO_MAX),

    ADJ_CONFIG(PITCH_P_GAIN,       PROF,  0, 1000),
    ADJ_CONFIG(PITCH_I_GAIN,       PROF,  0, 1000),
    ADJ_CONFIG(PITCH_D_GAIN,       PROF,  0, 1000),
    ADJ_CONFIG(PITCH_F_GAIN,       PROF,  0, 1000),
    ADJ_CONFIG(ROLL_P_GAIN,        PROF,  0, 1000),
    ADJ_CONFIG(ROLL_I_GAIN,        PROF,  0, 1000),
    ADJ_CONFIG(ROLL_D_GAIN,        PROF,  0, 1000),
    ADJ_CONFIG(ROLL_F_GAIN,        PROF,  0, 1000),
    ADJ_CONFIG(YAW_P_GAIN,         PROF,  0, 1000),
    ADJ_CONFIG(YAW_I_GAIN,         PROF,  0, 1000),
    ADJ_CONFIG(YAW_D_GAIN,         PROF,  0, 1000),
    ADJ_CONFIG(YAW_F_GAIN,         PROF,  0, 1000),

    ADJ_CONFIG(YAW_CW_GAIN,        PROF,  25, 250),
    ADJ_CONFIG(YAW_CCW_GAIN,       PROF,  25, 250),
    ADJ_CONFIG(YAW_CYCLIC_FF,      PROF,  0, 2500),
    ADJ_CONFIG(YAW_COLLECTIVE_FF,  PROF,  0, 2500),
    ADJ_CONFIG(YAW_IMPULSE_FF,     PROF,  0, 2500),

    ADJ_CONFIG(PITCH_COLL_FF,      PROF,  0, 2500),
    ADJ_CONFIG(PITCH_IMPULSE_FF,   PROF,  0, 2500),

    ADJ_CONFIG(RESCUE_CLIMB_COLLECTIVE,  PROF,  0, 1000),
    ADJ_CONFIG(RESCUE_HOVER_COLLECTIVE,  PROF,  0, 1000),

    ADJ_CONFIG(ANGLE_LEVEL_GAIN,   PROF,  0, 200),
    ADJ_CONFIG(HORIZON_LEVEL_GAIN, PROF,  0, 200),
    ADJ_CONFIG(ACRO_TRAINER_GAIN,  PROF, 25, 255),

    ADJ_CONFIG(GOV_GAIN,           GOV,   0, 250),
    ADJ_CONFIG(GOV_P_GAIN,         GOV,   0, 250),
    ADJ_CONFIG(GOV_I_GAIN,         GOV,   0, 250),
    ADJ_CONFIG(GOV_D_GAIN,         GOV,   0, 250),
    ADJ_CONFIG(GOV_F_GAIN,         GOV,   0, 250),
    ADJ_CONFIG(GOV_TTA_GAIN,       GOV,   0, 250),
    ADJ_CONFIG(GOV_CYCLIC_FF,      GOV,   0, 250),
    ADJ_CONFIG(GOV_COLLECTIVE_FF,  GOV,   0, 250),

    ADJ_CONFIG(TAIL_MOTOR_IDLE,    MIX,   0, 250),
    ADJ_CONFIG(SWASH_PHASE,        MIX,  -1800, 1800),

    ADJ_CONFIG(OSD_PROFILE,        NONE,  1,  3),
    ADJ_CONFIG(LED_PROFILE,        NONE,  1,  4),
    ADJ_CONFIG(PID_PROFILE,        NONE,  1,  6),
    ADJ_CONFIG(RATE_PROFILE,       NONE,  1,  6),

    // RF TODO for evalution only
    ADJ_CONFIG(WAY_P_GAIN,         PROF,  0, 1000),
    ADJ_CONFIG(WAY_I_GAIN,         PROF,  0, 1000),
    ADJ_CONFIG(WAY_D_GAIN,         PROF,  0, 1000),
    ADJ_CONFIG(WAY_F_GAIN,         PROF,  0, 1000),

    ADJ_CONFIG(PITCH_ERROR_CUTOFF, PROF,  0, 250),
    ADJ_CONFIG(PITCH_DTERM_CUTOFF, PROF,  0, 250),
    ADJ_CONFIG(PITCH_FTERM_CUTOFF, PROF,  0, 250),
    ADJ_CONFIG(ROLL_ERROR_CUTOFF,  PROF,  0, 250),
    ADJ_CONFIG(ROLL_DTERM_CUTOFF,  PROF,  0, 250),
    ADJ_CONFIG(ROLL_FTERM_CUTOFF,  PROF,  0, 250),
    ADJ_CONFIG(YAW_ERROR_CUTOFF,   PROF,  0, 250),
    ADJ_CONFIG(YAW_DTERM_CUTOFF,   PROF,  0, 250),
    ADJ_CONFIG(YAW_FTERM_CUTOFF,   PROF,  0, 250),

    ADJ_CONFIG(RESCUE_ALT_P_GAIN, PROF,  0, 1000),
    ADJ_CONFIG(RESCUE_ALT_I_GAIN, PROF,  0, 1000),
    ADJ_CONFIG(RESCUE_ALT_D_GAIN, PROF,  0, 1000),
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
        case ADJUSTMENT_YAW_CW_GAIN:
            value = currentPidProfile->yaw_cw_stop_gain;
            break;
        case ADJUSTMENT_YAW_CCW_GAIN:
            value = currentPidProfile->yaw_ccw_stop_gain;
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
        case ADJUSTMENT_PITCH_COLL_FF:
            value = currentPidProfile->pitch_collective_ff_gain;
            break;
        case ADJUSTMENT_PITCH_IMPULSE_FF:
            value = currentPidProfile->pitch_collective_ff_impulse_gain;
            break;
        case ADJUSTMENT_RESCUE_CLIMB_COLLECTIVE:
            value = currentPidProfile->rescue.climb_collective;
            break;
        case ADJUSTMENT_RESCUE_HOVER_COLLECTIVE:
            value = currentPidProfile->rescue.hover_collective;
            break;
        case ADJUSTMENT_ANGLE_LEVEL_GAIN:
            value = currentPidProfile->angle.level_strength;
            break;
        case ADJUSTMENT_HORIZON_LEVEL_GAIN:
            value = currentPidProfile->horizon.level_strength;
            break;
        case ADJUSTMENT_ACRO_TRAINER_GAIN:
            value = currentPidProfile->trainer.gain;
            break;
        case ADJUSTMENT_GOV_GAIN:
            value = currentPidProfile->governor.gain;
            break;
        case ADJUSTMENT_GOV_P_GAIN:
            value = currentPidProfile->governor.p_gain;
            break;
        case ADJUSTMENT_GOV_I_GAIN:
            value = currentPidProfile->governor.i_gain;
            break;
        case ADJUSTMENT_GOV_D_GAIN:
            value = currentPidProfile->governor.d_gain;
            break;
        case ADJUSTMENT_GOV_F_GAIN:
            value = currentPidProfile->governor.f_gain;
            break;
        case ADJUSTMENT_GOV_TTA_GAIN:
            value = currentPidProfile->governor.tta_gain;
            break;
        case ADJUSTMENT_GOV_CYCLIC_FF:
            value = currentPidProfile->governor.cyclic_ff_weight;
            break;
        case ADJUSTMENT_GOV_COLLECTIVE_FF:
            value = currentPidProfile->governor.collective_ff_weight;
            break;
        case ADJUSTMENT_TAIL_MOTOR_IDLE:
            value = mixerConfig()->tail_motor_idle;
            break;
        case ADJUSTMENT_SWASH_PHASE:
            value = mixerConfig()->swash_phase;
            break;
        case ADJUSTMENT_RATE_PROFILE:
            value = getCurrentControlRateProfileIndex() + 1;
            break;
        case ADJUSTMENT_PID_PROFILE:
            value = getCurrentPidProfileIndex() + 1;
            break;
        case ADJUSTMENT_OSD_PROFILE:
#ifdef USE_OSD_PROFILES
            value = getCurrentOsdProfileIndex();
#endif
            break;
        case ADJUSTMENT_LED_PROFILE:
#ifdef USE_LED_STRIP
            value = getLedProfile() + 1;
#endif
            break;

        // RF TODO for evalution only
        case ADJUSTMENT_WAY_P_GAIN:
            value = currentPidProfile->pid[PID_WAY].P;
            break;
        case ADJUSTMENT_WAY_I_GAIN:
            value = currentPidProfile->pid[PID_WAY].I;
            break;
        case ADJUSTMENT_WAY_D_GAIN:
            value = currentPidProfile->pid[PID_WAY].D;
            break;
        case ADJUSTMENT_WAY_F_GAIN:
            value = currentPidProfile->pid[PID_WAY].F;
            break;

        case ADJUSTMENT_PITCH_ERROR_CUTOFF:
            value = currentPidProfile->error_cutoff[PID_PITCH];
            break;
        case ADJUSTMENT_ROLL_ERROR_CUTOFF:
            value = currentPidProfile->error_cutoff[PID_ROLL];
            break;
        case ADJUSTMENT_YAW_ERROR_CUTOFF:
            value = currentPidProfile->error_cutoff[PID_YAW];
            break;
        case ADJUSTMENT_PITCH_DTERM_CUTOFF:
            value = currentPidProfile->dterm_cutoff[PID_PITCH];
            break;
        case ADJUSTMENT_ROLL_DTERM_CUTOFF:
            value = currentPidProfile->dterm_cutoff[PID_ROLL];
            break;
        case ADJUSTMENT_YAW_DTERM_CUTOFF:
            value = currentPidProfile->dterm_cutoff[PID_YAW];
            break;
        case ADJUSTMENT_PITCH_FTERM_CUTOFF:
            value = currentPidProfile->fterm_cutoff[PID_PITCH];
            break;
        case ADJUSTMENT_ROLL_FTERM_CUTOFF:
            value = currentPidProfile->fterm_cutoff[PID_ROLL];
            break;
        case ADJUSTMENT_YAW_FTERM_CUTOFF:
            value = currentPidProfile->fterm_cutoff[PID_YAW];
            break;

        case ADJUSTMENT_RESCUE_ALT_P_GAIN:
            value = currentPidProfile->rescue.alt_p_gain;
            break;
        case ADJUSTMENT_RESCUE_ALT_I_GAIN:
            value = currentPidProfile->rescue.alt_i_gain;
            break;
        case ADJUSTMENT_RESCUE_ALT_D_GAIN:
            value = currentPidProfile->rescue.alt_d_gain;
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
        case ADJUSTMENT_YAW_CW_GAIN:
            currentPidProfile->yaw_cw_stop_gain = value;
            break;
        case ADJUSTMENT_YAW_CCW_GAIN:
            currentPidProfile->yaw_ccw_stop_gain = value;
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
        case ADJUSTMENT_PITCH_COLL_FF:
            currentPidProfile->pitch_collective_ff_gain = value;
            break;
        case ADJUSTMENT_PITCH_IMPULSE_FF:
            currentPidProfile->pitch_collective_ff_impulse_gain = value;
            break;
        case ADJUSTMENT_RESCUE_CLIMB_COLLECTIVE:
            currentPidProfile->rescue.climb_collective = value;
            break;
        case ADJUSTMENT_RESCUE_HOVER_COLLECTIVE:
            currentPidProfile->rescue.hover_collective = value;
            break;
        case ADJUSTMENT_ANGLE_LEVEL_GAIN:
            currentPidProfile->angle.level_strength = value;
            break;
        case ADJUSTMENT_HORIZON_LEVEL_GAIN:
            currentPidProfile->horizon.level_strength = value;
            break;
        case ADJUSTMENT_ACRO_TRAINER_GAIN:
            currentPidProfile->trainer.gain = value;
            break;
        case ADJUSTMENT_GOV_GAIN:
            currentPidProfile->governor.gain = value;
            break;
        case ADJUSTMENT_GOV_P_GAIN:
            currentPidProfile->governor.p_gain = value;
            break;
        case ADJUSTMENT_GOV_I_GAIN:
            currentPidProfile->governor.i_gain = value;
            break;
        case ADJUSTMENT_GOV_D_GAIN:
            currentPidProfile->governor.d_gain = value;
            break;
        case ADJUSTMENT_GOV_F_GAIN:
            currentPidProfile->governor.f_gain = value;
            break;
        case ADJUSTMENT_GOV_TTA_GAIN:
            currentPidProfile->governor.tta_gain = value;
            break;
        case ADJUSTMENT_GOV_CYCLIC_FF:
            currentPidProfile->governor.cyclic_ff_weight = value;
            break;
        case ADJUSTMENT_GOV_COLLECTIVE_FF:
            currentPidProfile->governor.collective_ff_weight = value;
            break;
        case ADJUSTMENT_TAIL_MOTOR_IDLE:
            mixerConfigMutable()->tail_motor_idle = value;
            break;
        case ADJUSTMENT_SWASH_PHASE:
            mixerConfigMutable()->swash_phase = value;
            break;
        case ADJUSTMENT_RATE_PROFILE:
            changeControlRateProfile(value - 1);
            break;
        case ADJUSTMENT_PID_PROFILE:
            changePidProfile(value - 1);
            break;
        case ADJUSTMENT_OSD_PROFILE:
#ifdef USE_OSD_PROFILES
            changeOsdProfileIndex(value);
#endif
            break;
        case ADJUSTMENT_LED_PROFILE:
#ifdef USE_LED_STRIP
            setLedProfile(value - 1);
#endif
            break;

        // RF TODO for evalution only
        case ADJUSTMENT_WAY_P_GAIN:
            currentPidProfile->pid[PID_WAY].P = value;
            break;
        case ADJUSTMENT_WAY_I_GAIN:
            currentPidProfile->pid[PID_WAY].I = value;
            break;
        case ADJUSTMENT_WAY_D_GAIN:
            currentPidProfile->pid[PID_WAY].D = value;
            break;
        case ADJUSTMENT_WAY_F_GAIN:
            currentPidProfile->pid[PID_WAY].F = value;
            break;

        case ADJUSTMENT_PITCH_ERROR_CUTOFF:
            currentPidProfile->error_cutoff[PID_PITCH] = value;
            break;
        case ADJUSTMENT_ROLL_ERROR_CUTOFF:
            currentPidProfile->error_cutoff[PID_ROLL] = value;
            break;
        case ADJUSTMENT_YAW_ERROR_CUTOFF:
            currentPidProfile->error_cutoff[PID_YAW] = value;
            break;
        case ADJUSTMENT_PITCH_DTERM_CUTOFF:
            currentPidProfile->dterm_cutoff[PID_PITCH] = value;
            break;
        case ADJUSTMENT_ROLL_DTERM_CUTOFF:
            currentPidProfile->dterm_cutoff[PID_ROLL] = value;
            break;
        case ADJUSTMENT_YAW_DTERM_CUTOFF:
            currentPidProfile->dterm_cutoff[PID_YAW] = value;
            break;
        case ADJUSTMENT_PITCH_FTERM_CUTOFF:
            currentPidProfile->fterm_cutoff[PID_PITCH] = value;
            break;
        case ADJUSTMENT_ROLL_FTERM_CUTOFF:
            currentPidProfile->fterm_cutoff[PID_ROLL] = value;
            break;
        case ADJUSTMENT_YAW_FTERM_CUTOFF:
            currentPidProfile->fterm_cutoff[PID_YAW] = value;
            break;

        case ADJUSTMENT_RESCUE_ALT_P_GAIN:
            currentPidProfile->rescue.alt_p_gain = value;
            break;
        case ADJUSTMENT_RESCUE_ALT_I_GAIN:
            currentPidProfile->rescue.alt_i_gain = value;
            break;
        case ADJUSTMENT_RESCUE_ALT_D_GAIN:
            currentPidProfile->rescue.alt_d_gain = value;
            break;
    }
}

static void blackboxAdjustmentEvent(uint8_t adjFunc, int value)
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

#define ADJUSTMENT_LATENCY_MS 2500

static void updateAdjustmentData(uint8_t adjFunc, int value)
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
    uint8_t changed = 0;

    if (rxIsReceivingSignal())
    {
        for (int index = 0; index < MAX_ADJUSTMENT_RANGE_COUNT; index++)
        {
            const adjustmentRange_t * adjRange = adjustmentRanges(index);
            const adjustmentConfig_t * adjConfig = &adjustmentConfigs[adjRange->function];

            if (adjRange->enaChannel == 0xff || isRangeActive(adjRange->enaChannel, &adjRange->enaRange))
            {
                adjustmentState_t * adjState = &adjustmentState[index];
                const uint8_t adjFunc = adjRange->function;
                const timeMs_t now = millis();
                int adjval = adjState->adjValue;

                if (cmp32(now, adjState->deadTime) < 0)
                    continue;

                const int chValue = lrintf(rcData[adjRange->adjChannel + NON_AUX_CHANNEL_COUNT]);

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
                        adjval = getAdjustmentValue(adjFunc) - adjRange->adjStep;
                    }
                    else if (isRangeActive(adjRange->adjChannel, &adjRange->adjRange2)) {
                        adjval = getAdjustmentValue(adjFunc) + adjRange->adjStep;
                    }
                    else {
                        continue;
                    }
                }
                // Continuous adjustment
                else {
                    const int adjWidth = adjRange->adjMax - adjRange->adjMin;
                    const int chWidth = STEP_TO_CHANNEL_VALUE(adjRange->adjRange1.endStep) -
                                        STEP_TO_CHANNEL_VALUE(adjRange->adjRange1.startStep);
                    const int chStep  = chWidth / adjWidth / 2;
                    const int chStart = STEP_TO_CHANNEL_VALUE(adjRange->adjRange1.startStep) - chStep;
                    const int chEnd   = STEP_TO_CHANNEL_VALUE(adjRange->adjRange1.endStep) + chStep;

                    if (chValue > chStart && chValue < chEnd) {
                        adjval = adjRange->adjMin + (chValue - chStart) * adjWidth / chWidth;
                    }
                }

                adjval = constrain(adjval, adjConfig->cfgMin, adjConfig->cfgMax);
                adjval = constrain(adjval, adjRange->adjMin, adjRange->adjMax);

                if (adjval != adjState->adjValue) {
                    setAdjustmentValue(adjFunc, adjval);
                    updateAdjustmentData(adjFunc, adjval);
                    blackboxAdjustmentEvent(adjFunc, adjval);

                    beeperConfirmationBeeps(1);
                    setConfigDirty();

                    adjState->deadTime = now + REPEAT_DELAY;
                    adjState->adjValue = adjval;

                    changed |= adjConfig->cfgType;
                }
            }
        }

        if (changed & ADJUSTMENT_TYPE_RATE) {
            loadControlRateProfile();
        }
        if (changed & ADJUSTMENT_TYPE_PROF) {
            pidInitProfile(currentPidProfile);
        }
        if (changed & ADJUSTMENT_TYPE_GOV) {
            governorInitProfile(currentPidProfile);
        }
        if (changed & ADJUSTMENT_TYPE_MIX) {
            mixerInitConfig();
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

void adjustmentRangeInit(void)
{
    for (int index = 0; index < MAX_ADJUSTMENT_RANGE_COUNT; index++) {
        adjustmentRangeReset(index);
    }
}

void adjustmentRangeReset(int index)
{
    adjustmentState[index].deadTime = 0;
    adjustmentState[index].trigTime = 0;
    adjustmentState[index].adjValue = getAdjustmentValue(index);
    adjustmentState[index].chValue  = 0;
}
