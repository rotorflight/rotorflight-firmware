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

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/filter.h"

#include "config/feature.h"
#include "config/config.h"

#include "pg/motor.h"

#include "drivers/pwm_output.h"
#include "drivers/dshot_command.h"
#include "drivers/dshot.h"
#include "drivers/motor.h"
#include "drivers/freq.h"
#include "drivers/time.h"
#include "drivers/io.h"

#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/core.h"
#include "fc/rc.h"

#include "flight/motors.h"
#include "flight/servos.h"
#include "flight/mixer.h"
#include "flight/pid.h"


typedef enum {
    RPM_SRC_NONE = 0,
    RPM_SRC_DSHOT_TELEM,
    RPM_SRC_FREQ_SENSOR,
    RPM_SRC_ESC_SENSOR,
} rpmSource_e;


static FAST_DATA_ZERO_INIT uint8_t        motorCount;

static FAST_DATA_ZERO_INIT float          motorOutput[MAX_SUPPORTED_MOTORS];
static FAST_DATA_ZERO_INIT int16_t        motorOverride[MAX_SUPPORTED_MOTORS];

static FAST_DATA_ZERO_INIT float          motorRpm[MAX_SUPPORTED_MOTORS];
static FAST_DATA_ZERO_INIT float          motorRpmRaw[MAX_SUPPORTED_MOTORS];
static FAST_DATA_ZERO_INIT uint8_t        motorRpmDiv[MAX_SUPPORTED_MOTORS];
static FAST_DATA_ZERO_INIT uint8_t        motorRpmSource[MAX_SUPPORTED_MOTORS];
static FAST_DATA_ZERO_INIT filter_t       motorRpmFilter[MAX_SUPPORTED_MOTORS];

static FAST_DATA_ZERO_INIT float          headSpeed;
static FAST_DATA_ZERO_INIT float          tailSpeed;

static FAST_DATA_ZERO_INIT float          mainGearRatio;
static FAST_DATA_ZERO_INIT float          tailGearRatio;

static FAST_DATA_ZERO_INIT float          motorRpmFactor[MAX_SUPPORTED_MOTORS];


/*** Access functions ***/

uint8_t getMotorCount(void)
{
    return motorCount;
}

int16_t getMotorOutput(uint8_t motor)
{
    return lrintf(motorOutput[motor] * 1000);
}

bool hasMotorOverride(uint8_t motor)
{
    return (motorOverride[motor] != MOTOR_OVERRIDE_OFF);
}

int16_t getMotorOverride(uint8_t motor)
{
    return motorOverride[motor];
}

int16_t setMotorOverride(uint8_t motor, int16_t value)
{
    return motorOverride[motor] = value;
}

void resetMotorOverride(void)
{
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        motorOverride[i] = MOTOR_OVERRIDE_OFF;
}

float getMainGearRatio(void)
{
    return mainGearRatio;
}

float getTailGearRatio(void)
{
    return tailGearRatio;
}

int getHeadSpeed(void)
{
    return lrintf(headSpeed);
}

float getHeadSpeedf(void)
{
    return headSpeed;
}

int getTailSpeed(void)
{
    return lrintf(tailSpeed);
}

float getTailSpeedf(void)
{
    return tailSpeed;
}

int getMotorRPM(uint8_t motor)
{
    return lrintf(motorRpm[motor]);
}

float getMotorRPMf(uint8_t motor)
{
    return motorRpm[motor];
}

float getMotorRawRPMf(uint8_t motor)
{
    return motorRpmRaw[motor];
}

int calcMotorRPM(uint8_t motor, int erpm)
{
    return erpm / motorRpmDiv[motor];
}


bool isMotorRpmSourceActive(uint8_t motor)
{
    return (motor < motorCount && motorRpmSource[motor] != RPM_SRC_NONE);
}

bool isMotorFastRpmSourceActive(uint8_t motor)
{
    return (motor < motorCount && (motorRpmSource[motor] == RPM_SRC_DSHOT_TELEM || motorRpmSource[motor] == RPM_SRC_FREQ_SENSOR));
}

bool isRpmSourceActive(void)
{
    return (motorRpmSource[0] != RPM_SRC_NONE);
}

bool areMotorsRunning(void)
{
    if (ARMING_FLAG(ARMED))
        return true;

    for (int i = 0; i < motorCount; i++)
        if (motorOutput[i] != 0)
            return true;

    return false;
}


/*** Init functions ***/

INIT_CODE void rpmSourceInit(void)
{
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
#ifdef USE_FREQ_SENSOR
        if (featureIsEnabled(FEATURE_FREQ_SENSOR) && isFreqSensorPortInitialized(i))
            motorRpmSource[i] = RPM_SRC_FREQ_SENSOR;
        else
#endif
#ifdef USE_DSHOT_TELEMETRY
        if (isMotorProtocolDshot() && motorConfig()->dev.useDshotTelemetry)
            motorRpmSource[i] = RPM_SRC_DSHOT_TELEM;
        else
#endif
#ifdef USE_ESC_SENSOR
        if (featureIsEnabled(FEATURE_ESC_SENSOR) && isEscSensorActive())
            motorRpmSource[i] = RPM_SRC_ESC_SENSOR;
        else
#endif
            motorRpmSource[i] = RPM_SRC_NONE;

        motorRpmDiv[i] = constrain(motorConfig()->motorPoleCount[i] / 2, 1, 100);

        motorRpmFactor[i] = 1.0f + motorConfig()->motorRpmFactor[i] / 100000.0f;

        lowpassFilterInit(&motorRpmFilter[i], LPF_BESSEL, motorConfig()->motorRpmLpf[i], gyro.targetRateHz, 0);
    }
}

INIT_CODE void motorInit(void)
{
    const ioTag_t *ioTags = motorConfig()->dev.ioTags;

    for (motorCount = 0;
         motorCount < MAX_SUPPORTED_MOTORS && ioTags[motorCount] != IO_TAG_NONE;
         motorCount++);

    motorDevInit(&motorConfig()->dev, motorCount);

    mainGearRatio = fmaxf(motorConfig()->mainRotorGearRatio[0], 1) /
                    fmaxf(motorConfig()->mainRotorGearRatio[1], 1);

    tailGearRatio = fmaxf(motorConfig()->tailRotorGearRatio[0], 1) /
                    fmaxf(motorConfig()->tailRotorGearRatio[1], 1);

    if (!mixerMotorizedTail())
        tailGearRatio = mainGearRatio / tailGearRatio;
}


/*** Runtime functions ***/

static float getSensorRPMf(uint8_t motor)
{
    float erpm;

#ifdef USE_FREQ_SENSOR
    if (motorRpmSource[motor] == RPM_SRC_FREQ_SENSOR)
        erpm = getFreqSensorFreq(motor) * 60;
    else
#endif
#ifdef USE_DSHOT_TELEMETRY
    if (motorRpmSource[motor] == RPM_SRC_DSHOT_TELEM)
        erpm = getDshotTelemetry(motor);
    else
#endif
#ifdef USE_ESC_SENSOR
    if (motorRpmSource[motor] == RPM_SRC_ESC_SENSOR)
        erpm = getEscSensorRPM(motor);
    else
#endif
        erpm = 0;

    return motorRpmFactor[motor] * erpm / motorRpmDiv[motor];
}

void motorUpdate(void)
{
    float output;

    for (int i = 0; i < motorCount; i++) {
        if (ARMING_FLAG(ARMED))
            output = mixerGetMotorOutput(i);
        else
            output = motorOverride[i] / 1000.0f;

        motorOutput[i] = constrainf(output, -1, 1);
    }

    motorWriteAll(motorOutput);

    for (int i = 0; i < motorCount; i++) {
        motorRpmRaw[i] = getSensorRPMf(i);
        motorRpm[i] = fmaxf(filterApply(&motorRpmFilter[i], motorRpmRaw[i]), 0);
        DEBUG(RPM_SOURCE, i, motorRpmRaw[i]);
    }

    headSpeed = motorRpm[0] * mainGearRatio;
    tailSpeed = motorRpm[(motorCount > 1) ? 1 : 0] * tailGearRatio;
}

void motorStop(void)
{
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        motorOutput[i] = 0;

    motorWriteAll(motorOutput);
    delay(50);
}

