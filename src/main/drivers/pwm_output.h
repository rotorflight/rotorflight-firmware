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

#include "platform.h"

#include "common/time.h"

#include "drivers/dma.h"
#include "drivers/io_types.h"
#include "drivers/motor.h"
#include "drivers/timer.h"

#define MOTORS_MAX_PWM_RATE 480

#define ALL_MOTORS 255

#define PWM_TIMER_1MHZ        MHZ_TO_HZ(1)

struct timerHardware_s;

typedef struct timerChannel_s {
    volatile timCCR_t *ccr;
    TIM_TypeDef       *tim;
} timerChannel_t;

typedef struct {
    timerChannel_t channel;
    float pulseScale;
    float pulseOffset;
    bool forceOverflow;
    bool enabled;
    IO_t io;
    const timerHardware_t *timerHardware;
} pwmOutputPort_t;

extern FAST_DATA_ZERO_INIT pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];

struct motorDevConfig_s;
motorDevice_t *motorPwmDevInit(const struct motorDevConfig_s *motorDevConfig, uint8_t motorCount);

void pwmOutConfig(timerChannel_t *channel, const timerHardware_t *timerHardware, uint32_t hz, uint16_t period, uint16_t value, uint8_t inversion);

pwmOutputPort_t *pwmGetMotors(void);
bool pwmIsSynced(void);
