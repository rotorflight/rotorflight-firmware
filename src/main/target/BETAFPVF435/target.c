#include "platform.h"
#ifdef USE_TIMER
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

#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

// Default motor numbering follows the board's four onboard ESC connectors.
// Other timer-capable pads are available for user resource configuration.
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TMR3, CH3, PB0, TIM_USE_MOTOR, 0, 1, 0), // ESC1
    DEF_TIM(TMR3, CH4, PB1, TIM_USE_MOTOR, 0, 2, 0), // ESC2
    DEF_TIM(TMR2, CH4, PA3, TIM_USE_MOTOR, 0, 7, 3), // ESC3
    DEF_TIM(TMR2, CH3, PA2, TIM_USE_MOTOR, 0, 6, 3), // ESC4
    DEF_TIM(TMR1, CH2, PA9, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TMR1, CH3, PA10, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TMR8, CH1, PC6, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TMR8, CH3, PC8, TIM_USE_ANY, 0, 4, 5),
    DEF_TIM(TMR1, CH1, PA8, TIM_USE_ANY, 0, 5, 6),
    DEF_TIM(TMR4, CH1, PB6, TIM_USE_ANY, 0, 0, 1),
};
#endif
