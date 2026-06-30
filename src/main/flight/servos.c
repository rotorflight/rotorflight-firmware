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

#ifdef USE_SERVOS

#include "build/build_config.h"

#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/time.h"
#include "drivers/pwm_output.h"

#include "sensors/gyro.h"

#include "fc/runtime_config.h"

#include "flight/servos.h"
#include "flight/mixer.h"

#include "pg/servos.h"

/*
//서보 선형성 셋팅시 사용
#include "pg/pg.h"
#include "pg/pid.h"
#include "pid.h"
*/

static FAST_DATA_ZERO_INIT uint8_t      servoCount;

static FAST_DATA_ZERO_INIT float        servoInput[MAX_SUPPORTED_SERVOS];
static FAST_DATA_ZERO_INIT float        servoOutput[MAX_SUPPORTED_SERVOS];
static FAST_DATA_ZERO_INIT float        servoResolution[MAX_SUPPORTED_SERVOS];

static FAST_DATA_ZERO_INIT int16_t      servoOverride[MAX_SUPPORTED_SERVOS];

static FAST_DATA_ZERO_INIT timerChannel_t servoChannel[MAX_SUPPORTED_SERVOS];

// 보정 테이블
 // 서보 선형성 보정버전
/*static const float boostTable[3][2][4] = {
    {{2.11f, 4.17f, 4.65f, 3.70f}, {3.13f, 4.59f, 4.59f, 3.12f}}, // i=0 (뒤:서보1)
    {{3.57f, 3.71f, 3.80f, 2.65f}, {3.12f, 4.95f, 5.25f, 5.70f}}, // i=1 (왼쪽앞:서보2)
    {{3.36f, 4.27f, 4.51f, 3.14f}, {3.71f, 5.00f, 4.62f, 3.12f}}// i=2 (오른쪽앞:서보3)
};*/

// 피치 선형성 보정버전
static const float boostTable[3][2][4] = {
    {{2.10f, 2.81f, 3.67f, 3.01f}, {2.77f, 3.54f, 3.19f, 2.63f}}, // i=0 (뒤:서보1)
    {{4.01f, 4.25f, 4.16f, 2.14f}, {2.52f, 3.74f, 4.00f, 3.48f}}, // i=1 (왼쪽앞:서보2)
    {{2.84f, 3.32f, 2.97f, 1.85f}, {3.73f, 4.13f, 3.68f, 2.39f}}// i=2 (오른쪽앞:서보3)
};

uint8_t getServoCount(void)
{
    return servoCount;
}

uint16_t getServoOutput(uint8_t servo)
{
#if defined(USE_SBUS_OUTPUT) || defined(USE_FBUS_MASTER)
    // Check if this is a bus servo (SBUS/FBUS)
    if (servo >= BUS_SERVO_OFFSET && servo < BUS_SERVO_OFFSET + BUS_SERVO_CHANNELS) {
        const uint8_t busServoIndex = servo - BUS_SERVO_OFFSET;
        return getBusServoOutput(busServoIndex);
    }
#endif
    // PWM servo
    if (servo >= MAX_SUPPORTED_SERVOS) {
        return 0;
    }
    return lrintf(servoOutput[servo]);
}

bool hasServoOverride(uint8_t servo)
{
    return (servoOverride[servo] >= SERVO_OVERRIDE_MIN && servoOverride[servo] <= SERVO_OVERRIDE_MAX);
}

int16_t getServoOverride(uint8_t servo)
{
    return servoOverride[servo];
}

int16_t setServoOverride(uint8_t servo, int16_t val)
{
    return servoOverride[servo] = val;
}

bool isServoOverrideActive(void)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        if (hasServoOverride(i))
            return true;
    }
    return false;
}

void validateAndFixServoConfig(void)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        volatile servoParam_t *servo = servoParamsMutable(i);
        const bool isBusServo = (i >= BUS_SERVO_OFFSET);
        const uint16_t minSignal = isBusServo ? BUS_SERVO_MIN_SIGNAL : PWM_SERVO_PULSE_MIN;
        const uint16_t maxSignal = isBusServo ? BUS_SERVO_MAX_SIGNAL : PWM_SERVO_PULSE_MAX;
        
#ifndef USE_SERVO_GEOMETRY_CORRECTION
        servo->flags &= ~SERVO_FLAG_GEO_CORR;
#endif

        // Constrain midpoint to the valid signal range.
        servo->mid = constrain(servo->mid, minSignal, maxSignal);

        // Constrain travel to valid offset limits first.
        servo->min = constrain(servo->min, SERVO_LIMIT_MIN, 0);
        servo->max = constrain(servo->max, 0, SERVO_LIMIT_MAX);

        // Ensure the resulting absolute signal stays within allowed range.
        const int16_t minAllowed = (int16_t)minSignal - (int16_t)servo->mid;
        const int16_t maxAllowed = (int16_t)maxSignal - (int16_t)servo->mid;

        if (servo->min < minAllowed) {
            servo->min = minAllowed;
        }
        if (servo->max > maxAllowed) {
            servo->max = maxAllowed;
        }
    }
}

void servoInit(void)
{
    const ioTag_t *ioTags = servoConfig()->ioTags;
    const timerHardware_t *timer[MAX_SUPPORTED_SERVOS];
    uint32_t rates[MAX_SUPPORTED_SERVOS];
    uint8_t index, jndex;

    for (index = 0; index < MAX_SUPPORTED_SERVOS; index++)
    {
        servoOutput[index] = servoParams(index)->mid;
        servoOverride[index] = SERVO_OVERRIDE_OFF;
    }

    for (index = 0; index < MAX_SUPPORTED_SERVOS && ioTags[index]; index++)
    {
        const ioTag_t tag = ioTags[index];
        const IO_t io = IOGetByTag(tag);

        timer[index] = timerAllocate(tag, OWNER_SERVO, RESOURCE_INDEX(index));

        if (!timer[index])
            break;

        IOInit(io, OWNER_SERVO, RESOURCE_INDEX(index));
        IOConfigGPIOAF(io, IOCFG_AF_PP, timer[index]->alternateFunction);
    }

    servoCount = index;

    for (index = 0; index < servoCount; index++)
    {
        uint32_t update_rate = servoParams(index)->rate;

        for (jndex = 0; jndex < servoCount; jndex++) {
            if (timer[index]->tim == timer[jndex]->tim) {
                uint32_t maxpulse = servoParams(jndex)->mid + servoParams(jndex)->max;
                uint32_t maxrate = MIN(servoParams(jndex)->rate, 950000 / maxpulse);  // 1000000 / (maxpulse +5%)
                if (maxrate < update_rate)
                    update_rate = maxrate;
            }
        }

        rates[index] = constrain(update_rate, SERVO_RATE_MIN, SERVO_RATE_MAX);
    }

    for (index = 0; index < servoCount; index++)
    {
        const uint32_t timer_clock = timerClock(timer[index]->tim);
        const uint32_t update_rate = rates[index];
        uint32_t timebase;

        servoParamsMutable(index)->rate = update_rate;

        if (update_rate > 500) {
            const uint32_t timer_rate = update_rate * 64000;
            const uint32_t timer_div = (timer_clock + timer_rate - 1) / timer_rate;
            timebase = timer_clock / timer_div;
        }
        else if (update_rate > 154 && timer_clock % 10000000 == 0) {
            timebase = 10000000;
        }
        else if (update_rate > 124 && timer_clock % 8000000 == 0) {
            timebase = 8000000;
        }
        else if (update_rate > 77 && timer_clock % 5000000 == 0) {
            timebase = 5000000;
        }
        else if (update_rate > 62 && timer_clock % 4000000 == 0) {
            timebase = 4000000;
        }
        else if (update_rate > 31 && timer_clock % 2000000 == 0) {
            timebase = 2000000;
        }
        else {
            timebase = 1000000;
        }

        servoResolution[index] = timebase * 1e-6f;

        pwmOutConfig(&servoChannel[index], timer[index], timebase, timebase / update_rate, 0, 0);
    }
}

void servoShutdown(void)
{
    for (int index = 0; index < MAX_SUPPORTED_SERVOS; index++)
    {
        if (servoChannel[index].ccr) {
            *servoChannel[index].ccr = 0;
            servoChannel[index].ccr = NULL;
        }
    }

    delay(100);
}

static inline void servoSetOutput(uint8_t index, float pos)
{
    servoOutput[index] = pos;

    if (servoChannel[index].ccr)
        *servoChannel[index].ccr = lrintf(pos * servoResolution[index]);
}

static inline float limitTravel(uint8_t servo, float pos, float min, float max)
{
    if (pos > max) {
        mixerSaturateServoOutput(servo);
        return max;
    } else if (pos < min) {
        mixerSaturateServoOutput(servo);
        return min;
    }
    return pos;
}

static inline float limitSpeed(float old, float new, float speed)
{
    float rate = 1200 * pidGetDT() / speed;
    float diff = new - old;

    if (diff > rate)
        new = old + rate;
    else if (diff < -rate)
        new = old - rate;

    return new;
 }

 static inline float limitRatio(float old, float new, float ratio)
 {
    return old + (new - old) * ratio;
 }

#ifdef USE_SERVO_GEOMETRY_CORRECTION
float geometryCorrection(float pos)
{
    // 1.0 == 50° without correction
    float height = constrainf(pos * 0.7660444431f, -1, 1);

    // Scale 50° in rad => 1.0
    float rotation = asin_approx(height) * 1.14591559026f;

    return rotation;
}
#endif

void servoUpdate(void)
{
    float input[MAX_SUPPORTED_SERVOS];
    float cyclic_ratio = 1;

    for (int i = 0; i < servoCount; i++)
    {
        const servoParam_t *servo = servoParams(i);

        if (!ARMING_FLAG(ARMED) && hasServoOverride(i))
            input[i] = servoOverride[i] / 1000.0f;
        else
            input[i] = mixerGetServoOutput(i);

#ifdef USE_SERVO_GEOMETRY_CORRECTION
        if (servo->flags & SERVO_FLAG_GEO_CORR)
            input[i] = geometryCorrection(input[i]);
#endif

        if (servo->speed && mixerIsCyclicServo(i)) {
            const float limit = 1200 * pidGetDT() / servo->speed;
            const float speed = fabsf(input[i] - servoInput[i]);
            if (speed > limit)
                cyclic_ratio = fminf(cyclic_ratio, limit / speed);
        }
    }

    for (int i = 0; i < servoCount; i++)
    {
        const servoParam_t *servo = servoParams(i);
        float pos = input[i];

        if (servo->speed > 0) {
            if (mixerIsCyclicServo(i))
                pos = limitRatio(servoInput[i], pos, cyclic_ratio);
            else
                pos = limitSpeed(servoInput[i], pos, servo->speed);
        }

        servoInput[i] = pos;

        if (servo->flags & SERVO_FLAG_REVERSED)
            pos = -pos;

        float scale = (pos > 0) ? servo->rpos : servo->rneg;

        // 1. 기본 목표값 계산
        float target_pos = scale * pos;

        // 3. 현재 출력 위치 및 방향 판정
        float x = fabsf(pos) * 55.0f;
        float boost = 0.0f;
        // 서보 2번(i==1)만 방향 판정 뒤집기
        int dir = (i == 1) ? ((pos >= 0.0f) ? 1 : 0) : ((pos >= 0.0f) ? 0 : 1);
     
        /*
        //서보 선형성 튜닝(PID 프로파일값 활용 400이 0)
        int tuningServoIdx = 0; // 0, 1, 2 중 선택
        const pidProfile_t *tempProfile = pidProfiles(1);
        
        if (tempProfile != NULL) { // 데이터가 있을 때만 실행
            // 양수 구간 (Roll PID)
            boostTable[tuningServoIdx][0][0] = ((float)tempProfile->pid[PID_ROLL].P - 400.0f) / 100.0f;
            boostTable[tuningServoIdx][0][1] = ((float)tempProfile->pid[PID_ROLL].I - 400.0f) / 100.0f;
            boostTable[tuningServoIdx][0][2] = ((float)tempProfile->pid[PID_ROLL].D - 400.0f) / 100.0f;
            boostTable[tuningServoIdx][0][3] = ((float)tempProfile->pid[PID_ROLL].F - 400.0f) / 100.0f;
        
            // 음수 구간 (Pitch PID)
            boostTable[tuningServoIdx][1][0] = ((float)tempProfile->pid[PID_PITCH].P - 400.0f) / 100.0f;
            boostTable[tuningServoIdx][1][1] = ((float)tempProfile->pid[PID_PITCH].I - 400.0f) / 100.0f;
            boostTable[tuningServoIdx][1][2] = ((float)tempProfile->pid[PID_PITCH].D - 400.0f) / 100.0f;
            boostTable[tuningServoIdx][1][3] = ((float)tempProfile->pid[PID_PITCH].F - 400.0f) / 100.0f;
        }     
        //서보 선형성 튜닝(PID 프로파일값 활용
        */
     
        if (i < 3) {
             if (x < 40.0f) {
                int idx = (int)(x / 10.0f);
                float v0 = (idx == 0) ? 0.0f : boostTable[i][dir][idx - 1];
                float v1 = boostTable[i][dir][idx];
                boost = lerp(v0, v1, (x - (idx * 10.0f)) / 10.0f);
            } 
            else if (x < 55.0f) {
                boost = lerp(boostTable[i][dir][3], 0.0f, (x - 40.0f) / 15.0f);
                }

            // 5. 최종 출력 적용 (누락되었던 부분)
            float final_boost = (boost / 40.0f) * scale;
            target_pos += (pos >= 0.0f) ? final_boost : -final_boost;
        }
     
        // 6. 최종 안전 범위 검사 및 출력
        pos = limitTravel(i, target_pos, servo->min, servo->max);

        //pos = limitTravel(i, scale * pos, servo->min, servo->max);
        pos = servo->mid + pos;

        servoSetOutput(i, pos);
    }
}

#endif
