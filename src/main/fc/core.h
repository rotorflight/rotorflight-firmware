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

#include "common/time.h"
#include "pg/pg.h"

#if defined(USE_GPS) || defined(USE_MAG)
extern int16_t magHold;
#endif

typedef enum {
    DISARM_REASON_ARMING_DISABLED   = 0,
    DISARM_REASON_FAILSAFE          = 1,
    DISARM_REASON_THROTTLE_TIMEOUT  = 2,
    DISARM_REASON_STICKS            = 3,
    DISARM_REASON_SWITCH            = 4,
    //DISARM_REASON_CRASH_PROTECTION  = 5,
    //DISARM_REASON_RUNAWAY_TAKEOFF   = 6,
    DISARM_REASON_GPS_RESCUE        = 7,
    DISARM_REASON_SERIAL_COMMAND    = 8,
#ifdef UNIT_TEST
    DISARM_REASON_SYSTEM            = 255,
#endif
} flightLogDisarmReason_e;

// Subtasks of the PID loop (taskMainPidLoop)
typedef enum {
    CORE_ST_POSITION = 0,
    CORE_ST_SETPOINT,
    CORE_ST_PID,
    CORE_ST_MIXER,
    CORE_ST_MOTORS,
    CORE_ST_FILTER_UPDATE,
    CORE_ST_BLACKBOX,
    CORE_ST_BLACKBOX_FLUSH,
    CORE_ST_COUNT
} coreSubtask_e;

void resetArmingDisabled(void);

void disarm(flightLogDisarmReason_e reason);
void tryArm(void);

bool processRx(timeUs_t currentTimeUs);
void processRxModes(timeUs_t currentTimeUs);
void updateArmingStatus(void);

void taskGyroSample(timeUs_t currentTimeUs);
bool gyroFilterReady(void);
bool pidLoopReady(void);
void taskFiltering(timeUs_t currentTimeUs);
void taskMainPidLoop(timeUs_t currentTimeUs);

timeUs_t getLastDisarmTimeUs(void);
bool isTryingToArm(void);
void resetTryingToArm(void);

void subTaskTelemetryPollSensors(timeUs_t currentTimeUs);

uint8_t coreSubtaskTick(coreSubtask_e subtask);
uint8_t getPidUpdateCounter(void);

