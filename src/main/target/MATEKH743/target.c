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

#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] =
{
    DEF_TIM(TIM2,  CH1, PA15, TIM_USE_BEEPER, 0, 0, 0),  // BZ

    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_ANY, 0, 14, 0),    // LED

    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_ANY, 0, 0, 2),     // S1
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_ANY, 0, 1, 2),     // S2

    DEF_TIM(TIM5,  CH1, PA0,  TIM_USE_ANY, 0, 2, 0),     // S3
    DEF_TIM(TIM5,  CH2, PA1,  TIM_USE_ANY, 0, 3, 0),     // S4

    DEF_TIM(TIM5,  CH3, PA2,  TIM_USE_ANY, 0, 4, 0),     // S5
    DEF_TIM(TIM5,  CH4, PA3,  TIM_USE_ANY, 0, 5, 0),     // S6

    DEF_TIM(TIM4,  CH1, PD12, TIM_USE_ANY, 0, 0, 0),     // S7
    DEF_TIM(TIM4,  CH2, PD13, TIM_USE_ANY, 0, 0, 0),     // S8
    DEF_TIM(TIM4,  CH3, PD14, TIM_USE_ANY, 0, 0, 0),     // S9
    DEF_TIM(TIM4,  CH4, PD15, TIM_USE_ANY, 0, 0, 0),     // S10

    DEF_TIM(TIM15, CH1, PE5,  TIM_USE_ANY, 0, 0, 0),     // S11
    DEF_TIM(TIM15, CH2, PE6,  TIM_USE_ANY, 0, 0, 0),     // S12

    DEF_TIM(TIM8,  CH2, PC7,  TIM_USE_ANY, 0, 0, 0),     // RX6
    DEF_TIM(TIM8,  CH1, PC6,  TIM_USE_ANY, 0, 0, 0),     // TX6

    DEF_TIM(TIM16, CH1, PB8,  TIM_USE_ANY, 0, 0, 0),     // RX4
    DEF_TIM(TIM17, CH1, PB9,  TIM_USE_ANY, 0, 0, 0),     // TX4
};
