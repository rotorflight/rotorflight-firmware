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
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_PWM_OUTPUT

#include "common/maths.h"

#include "drivers/castle_telemetry_decode.h"
#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/pwm_output.h"
#include "drivers/time.h"
#include "drivers/timer.h"

#include "pg/motor.h"

FAST_DATA_ZERO_INIT pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value, uint8_t output)
{
#if defined(USE_HAL_DRIVER)
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(tim);
    if (Handle == NULL) return;

    TIM_OC_InitTypeDef TIM_OCInitStructure;

    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
    TIM_OCInitStructure.OCPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_SET;
    TIM_OCInitStructure.OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;
    TIM_OCInitStructure.Pulse = value;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(Handle, &TIM_OCInitStructure, channel);
#else
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

    if (output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
        TIM_OCInitStructure.TIM_OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_Low : TIM_OCNPolarity_High;
    } else {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCPolarity =  (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
    }
    TIM_OCInitStructure.TIM_Pulse = value;

    timerOCInit(tim, channel, &TIM_OCInitStructure);
    timerOCPreloadConfig(tim, channel, TIM_OCPreload_Enable);
#endif
}

void pwmOutConfig(timerChannel_t *channel, const timerHardware_t *timerHardware, uint32_t hz, uint16_t period, uint16_t value, uint8_t inversion)
{
#if defined(USE_HAL_DRIVER)
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(timerHardware->tim);
    if (Handle == NULL) return;
#endif

    configTimeBase(timerHardware->tim, period, hz);
    pwmOCConfig(timerHardware->tim,
        timerHardware->channel,
        value,
        inversion ? timerHardware->output ^ TIMER_OUTPUT_INVERTED : timerHardware->output
        );

#if defined(USE_HAL_DRIVER)
    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL)
        HAL_TIMEx_PWMN_Start(Handle, timerHardware->channel);
    else
        HAL_TIM_PWM_Start(Handle, timerHardware->channel);
    HAL_TIM_Base_Start(Handle);
#else
    TIM_CtrlPWMOutputs(timerHardware->tim, ENABLE);
    TIM_Cmd(timerHardware->tim, ENABLE);
#endif

    channel->ccr = timerChCCR(timerHardware);

    channel->tim = timerHardware->tim;

    *channel->ccr = 0;
}


/* MOTORS */

static FAST_DATA_ZERO_INIT motorDevice_t motorPwmDevice;

static void pwmWriteUnused(uint8_t index, uint8_t mode, float value)
{
    UNUSED(index);
    UNUSED(mode);
    UNUSED(value);
}

static float pwmConvertToInternal(uint8_t index, uint8_t mode, float throttle)
{
    UNUSED(index);

    float value = motorConfig()->mincommand;

    if (mode == MOTOR_CONTROL_BIDIR) {
        if (throttle != 0)
            value = scaleRangef(throttle, -1, 1, motorConfig()->minthrottle, motorConfig()->maxthrottle);
    }
    else {
        if (throttle > 0)
            value = scaleRangef(throttle, 0, 1, motorConfig()->minthrottle, motorConfig()->maxthrottle);
    }

    return value;
}

static void pwmWriteStandard(uint8_t index, uint8_t mode, float throttle)
{
    float value = pwmConvertToInternal(index,mode,throttle);
    float pulse = value * motors[index].pulseScale + motors[index].pulseOffset;

    *motors[index].channel.ccr = lrintf(pulse);
}

void pwmShutdownPulsesForAllMotors(void)
{
    for (int index = 0; index < motorPwmDevice.count; index++) {
        // Set the compare register to 0, which stops the output pulsing if the timer overflows
        if (motors[index].channel.ccr) {
            *motors[index].channel.ccr = 0;
        }
    }
}

void pwmDisableMotors(void)
{
    pwmShutdownPulsesForAllMotors();
}

static motorVTable_t motorPwmVTable;

bool pwmEnableMotors(void)
{
    /* check motors can be enabled */
    if (motorPwmVTable.write == &pwmWriteUnused) {
        return false;
    }

    /* Reconfigure GPIO pins with proper alternate function after 4way mode or other reset */
    for (int i = 0; i < motorPwmDevice.count; i++) {
        if (motors[i].enabled && motors[i].io != IO_NONE && motors[i].timerHardware) {
            IOConfigGPIOAF(motors[i].io, IOCFG_AF_PP, motors[i].timerHardware->alternateFunction);
        }
    }

    return true;
}

bool pwmIsMotorEnabled(uint8_t index)
{
    return motors[index].enabled;
}

static void pwmCompleteOneshotMotorUpdate(void)
{
    for (int index = 0; index < motorPwmDevice.count; index++) {
        if (motors[index].forceOverflow) {
            timerForceOverflow(motors[index].channel.tim);
        }
        // Set the compare register to 0, which stops the output pulsing if the timer overflows before the main loop completes again.
        // This compare register will be set to the output value on the next main loop.
        *motors[index].channel.ccr = 0;
    }
}

static motorVTable_t motorPwmVTable = {
    .postInit = motorPostInitNull,
    .enable = pwmEnableMotors,
    .disable = pwmDisableMotors,
    .isMotorEnabled = pwmIsMotorEnabled,
    .shutdown = pwmShutdownPulsesForAllMotors,
};

motorDevice_t *motorPwmDevInit(const motorDevConfig_t *motorConfig, uint8_t motorCount)
{
    bool useUnsyncedPwm = motorConfig->useUnsyncedPwm;

    motorPwmDevice.vTable = motorPwmVTable;

    float sMin = 0;
    float sLen = 0;
    switch (motorConfig->motorPwmProtocol) {
    default:
    case PWM_TYPE_ONESHOT125:
        sMin = 125e-6f;
        sLen = 125e-6f;
        break;
    case PWM_TYPE_ONESHOT42:
        sMin = 42e-6f;
        sLen = 42e-6f;
        break;
    case PWM_TYPE_MULTISHOT:
        sMin = 5e-6f;
        sLen = 20e-6f;
        break;
    case PWM_TYPE_STANDARD:
        sMin = 1e-3f;
        sLen = 1e-3f;
        useUnsyncedPwm = true;
        break;
#ifdef USE_TELEMETRY_CASTLE
    case PWM_TYPE_CASTLE_LINK:
        sMin = 1e-3f;
        sLen = 1e-3f;
        useUnsyncedPwm = true;
        break;
#endif
    }

    motorPwmDevice.vTable.write = pwmWriteStandard;
    motorPwmDevice.vTable.updateStart = motorUpdateStartNull;
    motorPwmDevice.vTable.updateComplete = useUnsyncedPwm ? motorUpdateCompleteNull : pwmCompleteOneshotMotorUpdate;

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        const ioTag_t tag = motorConfig->ioTags[motorIndex];
        const timerHardware_t *timerHardware = timerAllocate(tag, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));

        if (timerHardware == NULL) {
            /* not enough motors initialised for the mixer or a break in the motors */
            motorPwmDevice.vTable.write = &pwmWriteUnused;
            motorPwmDevice.vTable.updateComplete = motorUpdateCompleteNull;
            /* TODO: block arming and add reason system cannot arm */
            return NULL;
        }

        motors[motorIndex].io = IOGetByTag(tag);
        IOInit(motors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
        motors[motorIndex].timerHardware = timerHardware;

        IOConfigGPIOAF(motors[motorIndex].io, IOCFG_AF_PP, timerHardware->alternateFunction);

        /* standard PWM outputs */
        // margin of safety is 4 periods when unsynced
        const unsigned pwmRateHz = useUnsyncedPwm ? motorConfig->motorPwmRate : ceilf(1 / ((sMin + sLen) * 4));

        const uint32_t clock = timerClock(timerHardware->tim);
        /* used to find the desired timer frequency for max resolution */
        const unsigned prescaler = ((clock / pwmRateHz) + 0xffff) / 0x10000; /* rounding up */
        const uint32_t hz = clock / prescaler;
        const unsigned period = useUnsyncedPwm ? hz / pwmRateHz : 0xffff;

        /*
            if brushed then it is the entire length of the period.
            TODO: this can be moved back to periodMin and periodLen
            once mixer outputs a 0..1 float value.
        */
        motors[motorIndex].pulseScale = (sLen * hz) / 1000.0f;
        motors[motorIndex].pulseOffset = (sMin * hz) - (motors[motorIndex].pulseScale * 1000);

#ifdef USE_TELEMETRY_CASTLE
        if (motorConfig->motorPwmProtocol == PWM_TYPE_CASTLE_LINK) {
	    pwmOutConfig(&motors[motorIndex].channel, timerHardware, hz, period, 0, true);
            castleInputConfig(timerHardware, &motors[motorIndex].channel, hz);
        }
	else
#endif
	{
	    pwmOutConfig(&motors[motorIndex].channel, timerHardware, hz, period, 0, 0);
	}

        bool timerAlreadyUsed = false;
        for (int i = 0; i < motorIndex; i++) {
            if (motors[i].channel.tim == motors[motorIndex].channel.tim) {
                timerAlreadyUsed = true;
                break;
            }
        }
        motors[motorIndex].forceOverflow = !timerAlreadyUsed;
        motors[motorIndex].enabled = true;
    }

    return &motorPwmDevice;
}

pwmOutputPort_t *pwmGetMotors(void)
{
    return motors;
}

#endif // USE_PWM_OUTPUT
