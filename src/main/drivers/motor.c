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
 *
 * Author: jflyper
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_MOTOR

#include "common/maths.h"

#include "config/feature.h"

#include "build/debug.h"

#include "drivers/dshot_command.h"
#include "drivers/dshot_bitbang.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/pwm_output.h"
#include "drivers/time.h"

#include "motor.h"

static FAST_DATA_ZERO_INIT bool motorProtocolEnabled;
static FAST_DATA_ZERO_INIT bool motorProtocolDshot;

static FAST_DATA_ZERO_INIT motorDevice_t *motorDevice;


// Functions for Null vTable

void motorPostInitNull(void)
{
}

bool motorEnableNull(void)
{
    return false;
}

void motorDisableNull(void)
{
}

void motorShutdownNull(void)
{
}

bool motorUpdateStartNull(void)
{
    return true;
}

void motorUpdateCompleteNull(void)
{
}

void motorWriteNull(uint8_t index, uint8_t mode, float value)
{
    UNUSED(index);
    UNUSED(mode);
    UNUSED(value);
}

void motorWriteIntNull(uint8_t index, uint16_t value)
{
    UNUSED(index);
    UNUSED(value);
}

bool motorIsEnabledNull(uint8_t index)
{
    UNUSED(index);
    return false;
}


static const motorVTable_t motorNullVTable = {
    .postInit = motorPostInitNull,
    .enable = motorEnableNull,
    .disable = motorDisableNull,
    .shutdown = motorShutdownNull,
    .updateStart = motorUpdateStartNull,
    .updateComplete = motorUpdateCompleteNull,
    .write = motorWriteNull,
    .writeInt = motorWriteIntNull,
    .isMotorEnabled = motorIsEnabledNull,
};

static motorDevice_t motorNullDevice = {
    .initialized = false,
    .enabled = false,
};


motorVTable_t motorGetVTable(void)
{
    return motorDevice->vTable;
}

unsigned motorDeviceCount(void)
{
    return motorDevice->count;
}

bool isMotorProtocolEnabled(void)
{
    return motorProtocolEnabled;
}

bool isMotorProtocolDshot(void)
{
    return motorProtocolDshot;
}

bool isMotorProtocolCastlePWM(void)
{
#ifdef USE_TELEMETRY_CASTLE
    return motorConfig()->dev.motorPwmProtocol == PWM_TYPE_CASTLE_LINK;
#else
    return false;
#endif
}

bool checkMotorProtocolEnabled(const motorDevConfig_t *motorDevConfig)
{
    switch (motorDevConfig->motorPwmProtocol) {
    case PWM_TYPE_STANDARD:
    case PWM_TYPE_ONESHOT125:
    case PWM_TYPE_ONESHOT42:
    case PWM_TYPE_MULTISHOT:
#ifdef USE_DSHOT
    case PWM_TYPE_DSHOT150:
    case PWM_TYPE_DSHOT300:
    case PWM_TYPE_DSHOT600:
    case PWM_TYPE_PROSHOT1000:
#endif
#ifdef USE_TELEMETRY_CASTLE
    case PWM_TYPE_CASTLE_LINK:
#endif
        return true;
    }
    return false;
}

bool checkMotorProtocolDshot(const motorDevConfig_t *motorDevConfig)
{
#ifdef USE_DSHOT
    switch (motorDevConfig->motorPwmProtocol) {
    case PWM_TYPE_DSHOT150:
    case PWM_TYPE_DSHOT300:
    case PWM_TYPE_DSHOT600:
    case PWM_TYPE_PROSHOT1000:
        return true;
    }
#endif
    return false;
}

void motorShutdown(void)
{
    motorDevice->vTable.shutdown();
    motorDevice->enabled = false;
    motorDevice->motorEnableTimeMs = 0;
    motorDevice->initialized = false;

    delayMicroseconds(1500);
}

void motorWriteAll(float *values)
{
#ifdef USE_PWM_OUTPUT
    if (motorDevice->enabled) {
#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
        if (!motorDevice->vTable.updateStart()) {
            return;
        }
#endif
        for (int i = 0; i < motorDevice->count; i++) {
            motorDevice->vTable.write(i, motorDevice->motorControlMode[i], values[i]);
        }
        motorDevice->vTable.updateComplete();
    }
#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY_STATS)
    if (debugMode == DEBUG_DSHOT_RPM_ERRORS && useDshotTelemetry) {
        const uint8_t count = MIN(motorDevice->count, 4);
        for (uint8_t i = 0; i < count; i++) {
            debug[i] = getDshotTelemetryMotorInvalidPercent(i);
        }
    }
#endif
#endif
}

void motorDevInit(const motorDevConfig_t *motorDevConfig, uint8_t motorCount)
{
    motorProtocolEnabled = checkMotorProtocolEnabled(motorDevConfig);
    motorProtocolDshot   = checkMotorProtocolDshot(motorDevConfig);

    if (motorProtocolEnabled) {
#ifdef USE_DSHOT
        if (motorProtocolDshot) {
#ifdef USE_DSHOT_BITBANG
            if (isDshotBitbangActive(motorDevConfig)) {
                motorDevice = dshotBitbangDevInit(motorDevConfig, motorCount);
            } else
#endif
            {
                motorDevice = dshotPwmDevInit(motorDevConfig, motorCount);
            }
        } else
#endif
        {
            motorDevice = motorPwmDevInit(motorDevConfig, motorCount);
        }
    }

    if (motorDevice) {
        motorDevice->count = motorCount;
        motorDevice->initialized = true;
        motorDevice->enabled = false;
        motorDevice->motorEnableTimeMs = 0;
        for (int motorIndex = 0; motorIndex < motorCount; motorIndex++)
            motorDevice->motorControlMode[motorIndex] = motorDevConfig->motorControlMode[motorIndex];
    }
    else {
        motorNullDevice.vTable = motorNullVTable;
        motorDevice = &motorNullDevice;
    }
}

void motorPostInit()
{
    motorDevice->vTable.postInit();
}

void motorDisable(void)
{
    motorDevice->vTable.disable();
    motorDevice->enabled = false;
    motorDevice->motorEnableTimeMs = 0;
}

void motorEnable(void)
{
    if (motorDevice->initialized && motorDevice->vTable.enable()) {
        motorDevice->enabled = true;
        motorDevice->motorEnableTimeMs = millis();
    }
}

bool motorIsEnabled(void)
{
    return motorDevice->enabled;
}

bool motorIsMotorEnabled(uint8_t index)
{
    return motorDevice->vTable.isMotorEnabled(index);
}

#ifdef USE_DSHOT
timeMs_t motorGetMotorEnableTimeMs(void)
{
    return motorDevice->motorEnableTimeMs;
}
#endif

#ifdef USE_DSHOT_BITBANG
bool isDshotBitbangActive(const motorDevConfig_t *motorDevConfig)
{
#ifdef STM32F4
    return motorDevConfig->useDshotBitbang == DSHOT_BITBANG_ON ||
        (motorDevConfig->useDshotBitbang == DSHOT_BITBANG_AUTO &&
         motorDevConfig->useDshotTelemetry &&
         motorDevConfig->motorPwmProtocol != PWM_TYPE_PROSHOT1000);
#else
    return motorDevConfig->useDshotBitbang == DSHOT_BITBANG_ON ||
        (motorDevConfig->useDshotBitbang == DSHOT_BITBANG_AUTO &&
         motorDevConfig->motorPwmProtocol != PWM_TYPE_PROSHOT1000);
#endif
}
#endif

#endif // USE_MOTOR
