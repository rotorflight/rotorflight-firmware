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

#include "platform.h"

#ifdef USE_MOTOR

#include "drivers/pwm_output.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/motor.h"

PG_REGISTER_WITH_RESET_FN(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 1);

void pgResetFn_motorConfig(motorConfig_t *motorConfig)
{
    motorConfig->dev.motorPwmProtocol = PWM_TYPE_STANDARD;
    motorConfig->dev.motorPwmRate = 250;
    motorConfig->dev.useUnsyncedPwm = false;
    motorConfig->minthrottle = 1070;
    motorConfig->maxthrottle = 2000;
    motorConfig->mincommand = 1000;

#ifdef USE_DSHOT_DMAR
    motorConfig->dev.useBurstDshot = ENABLE_DSHOT_DMAR;
#endif

#ifdef USE_TIMER
    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS; motorIndex++) {
        motorConfig->dev.ioTags[motorIndex] = timerioTagGetByUsage(TIM_USE_MOTOR, motorIndex);
    }
#endif

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS; motorIndex++) {
        motorConfig->motorRpmLpf[motorIndex] = 100;
        motorConfig->motorRpmFactor[motorIndex] = 0;
        motorConfig->motorPoleCount[motorIndex] = 0;
    }

    motorConfig->mainRotorGearRatio[0] = 1;
    motorConfig->mainRotorGearRatio[1] = 1;
    motorConfig->tailRotorGearRatio[0] = 1;
    motorConfig->tailRotorGearRatio[1] = 1;
    motorConfig->motorType = MOTOR_TYPE_ELECTRIC;

#ifdef USE_DSHOT_BITBANG
    motorConfig->dev.useDshotBitbang = DSHOT_BITBANG_DEFAULT;
    motorConfig->dev.useDshotBitbangedTimer = DSHOT_BITBANGED_TIMER_DEFAULT;
#endif
}

#endif // USE_MOTOR
