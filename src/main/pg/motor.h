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

#include "pg/pg.h"

#include "drivers/io.h"
#include "drivers/dshot_bitbang.h"

typedef enum {
    DSHOT_BITBANGED_TIMER_AUTO = 0,
    DSHOT_BITBANGED_TIMER_TIM1,
    DSHOT_BITBANGED_TIMER_TIM8,
} dshotBitbangedTimer_e;

typedef enum {
    DSHOT_DMAR_OFF,
    DSHOT_DMAR_ON,
    DSHOT_DMAR_AUTO
} dshotDmar_e;

typedef struct motorDevConfig_s {
    uint16_t motorPwmRate;                  // The update rate of motor outputs (50-498Hz)
    uint8_t  motorPwmProtocol;
    uint8_t  motorTransportProtocol;

    uint8_t  motorControlMode[MAX_SUPPORTED_MOTORS];

    uint8_t  useUnsyncedPwm;
    uint8_t  useBurstDshot;
    uint8_t  useDshotTelemetry;
    uint8_t  useDshotBitbang;
    uint8_t  useDshotBitbangedTimer;

    ioTag_t  ioTags[MAX_SUPPORTED_MOTORS];

} motorDevConfig_t;

typedef struct motorConfig_s {
    motorDevConfig_t dev;

    uint16_t minthrottle;                   // Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
    uint16_t maxthrottle;                   // This is the maximum value for the ESCs at full power this value can be increased up to 2000
    uint16_t mincommand;                    // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs

    uint8_t motorPoleCount[MAX_SUPPORTED_MOTORS]; // Magnetic poles in the motors for calculating actual RPM from eRPM provided by ESC telemetry
    uint8_t motorRpmLpf[MAX_SUPPORTED_MOTORS];    // RPM low pass filter cutoff frequency

    uint16_t mainRotorGearRatio[2];         // Main motor to main rotor gear ratio [N,D]
    uint16_t tailRotorGearRatio[2];         // Main rotor to tail rotor gear ratio [N,D]

} motorConfig_t;

PG_DECLARE(motorConfig_t, motorConfig);
