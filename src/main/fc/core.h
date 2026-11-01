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

// Which of the causes behind ARMING_DISABLED_MOTOR_PROTOCOL applies.
typedef enum {
    MOTOR_PROTO_CAUSE_NONE = 0,
    MOTOR_PROTO_CAUSE_DISABLED,         // motor protocol not set, or not supported
    MOTOR_PROTO_CAUSE_INIT_FAILED,      // motor outputs failed to set up
    MOTOR_PROTO_CAUSE_NO_TAIL_MOTOR,    // motorised tail, but no MOTOR 2
} motorProtocolCause_e;

motorProtocolCause_e getMotorProtocolCause(void);

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

