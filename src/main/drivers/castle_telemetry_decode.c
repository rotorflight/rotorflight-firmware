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

#include "platform.h"

#ifdef USE_TELEMETRY_CASTLE

#include "build/atomic.h"

#include "drivers/castle_telemetry_decode.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h"
#include "drivers/timer.h"

typedef struct castleInterrupt_s {
    timerCCHandlerRec_t pwmEdgeCb;
    volatile timCCR_t *timingChannelCCR;
    volatile timCCR_t *directChannelCCR;
    timerChannel_t *timer;
    timCCR_t outputEnableTime;
    timCCR_t pwmEdge;
    timCCR_t saveCCR;
    castleTelemetry_t telem[2];
    uint8_t directChannelIndex;
    uint8_t whichTelem; // telem we are writing to (0 or 1).
    uint8_t telemIndex; // where in the telem struct are we?
} castleInterrupt_t;

static FAST_DATA_ZERO_INIT castleInterrupt_t castleState;

void getCastleTelemetry(castleTelemetry_t* telem)
{
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        memcpy(telem, &castleState.telem[castleState.whichTelem ^ 1], sizeof(castleTelemetry_t));
    }
}

// Both the HAL and the LL drivers generate a lot more code than we need; doing
// direct register manipulation instead saves us several microseconds per
// PWM cycle.
#define IC_ENABLE_POLARITY_RISING TIM_CCER_CC1E
#define IC_ENABLE_POLARITY_FALLING (TIM_CCER_CC1P | TIM_CCER_CC1E)
#define IC_ENABLE_POLARITY_BOTHEDGES (TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC1E)
#define OC_ENABLE_POLARITY_HIGH TIM_CCER_CC1E
#define OC_ENABLE_POLARITY_LOW (TIM_CCER_CC1P | TIM_CCER_CC1E)

#define DISABLE_CHANNEL(tim, channelIndex) (tim->CCER &= ~((timCCER_t)0xB << (channelIndex << 2)))
// This ENABLE_CHANNEL_POLARITY only works if the relevant CCER bits
// start out all clear, which they will if they were disabled with
// DISABLE_CHANNEL
#define ENABLE_CHANNEL_POLARITY(tim, channelIndex, polarity) (tim->CCER |= ((timCCER_t)polarity << (channelIndex << 2)))

#define IC_FILTER_FDIV1 0
// 15 more IC_FILTER values omitted.
#define IC_PSC_DIV1 0
#define IC_PSC_DIV2 TIM_CCMR1_IC1PSC_0
#define IC_PSC_DIV4 TIM_CCMR1_IC1PSC_1
#define IC_PSC_DIV8 TIM_CCMR1_IC1PSC
#define IC_INPUT_DIRECT TIM_CCMR1_CC1S_0
#define IC_INPUT_INDIRECT TIM_CCMR1_CC1S_1
#define IC_INPUT_TRC TIM_CCMR1_CC1S

#define OC_MODE_FROZEN 0
#define OC_MODE_ACTIVE TIM_CCMR1_OC1M_0
#define OC_MODE_INACTIVE TIM_CCMR1_OC1M_1
#define OC_MODE_TOGGLE (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0)
#define OC_MODE_FORCED_ACTIVE TIM_CCMR1_OC1M_2
#define OC_MODE_FORCED_INACTIVE (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0)
#define OC_MODE_PWM1 (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1)
#define OC_MODE_PWM2 (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0)
// modes with OCxM_3 set aren't supported here.
#define OC_PRELOAD_DISABLE 0
#define OC_PRELOAD_ENABLE TIM_CCMR1_OC1PE
#define OC_FAST_DISABLE 0
#define OC_FAST_ENABLE TIM_CCMR1_OC1FE

// Must provide OC_MODE and OC_FAST and OC_PRELOAD
// or IC_FILTER, IC_PSC, and IC_INPUT
typedef uint32_t timCCMR_t;
#define CAPTURE_CONFIGURE(tim, channelIndex, conf) do {                 \
        timCCMR_t val = *(&(tim)->CCMR1 + (((channelIndex) >> 1)&1));   \
        val = (val & (~((timCCMR_t)0xFF)<< (((channelIndex) & 1)<<3))) | ((timCCMR_t)(conf) << (((channelIndex) & 1)<<3)); \
        *(&(tim)->CCMR1 + (((channelIndex) >> 1)&1)) = val;     \
    } while(0)

static void pwmEdgeCallback(timerCCHandlerRec_t *cbRec, captureCompare_t timingCompare)
{
    castleInterrupt_t *state = container_of(cbRec, castleInterrupt_t, pwmEdgeCb);
    // Castle uses an inverted (active low) PWM pulse and between pulses, the ESC will quickly
    // pull the line down and then release it (they call it a 'tick').  The timing of the tick
    // starting from the end of the PWM pulse is the telemetry value.
    //
    // The timing of the interrupts are not critical; all sensitive timing is done by the timer.
    // The only requirements on the interrupt is the PWM-end interrupt must be serviced before 0.5ms
    // after the end of the pulse, and the output-on interrupt must be serviced before the next counter
    // reset.
    if (timingCompare >= state->outputEnableTime) {
        uint16_t telemVal = *state->directChannelCCR;
        if (telemVal <= state->pwmEdge) {
            // The capture register retained the PWM value. Thus no capture
            // occurred, so this was a sync frame.
            state->telemIndex = 1;
        } else if (state->telemIndex > 0) {
            telemVal -= state->pwmEdge;
            ((uint16_t*)&state->telem[state->whichTelem])[state->telemIndex] = telemVal;
            if (telemVal <= (state->pwmEdge >> 4)) {
                // When the battery is disconnected we get some spurious
                // telemetry frames.
                state->telemIndex = 0;
            } else if (++state->telemIndex == CASTLE_TELEM_NFRAMES) {
                state->telemIndex = 0;
                // Note the first valid telemetry generation is 1.
                state->telem[state->whichTelem ^ 1].generation =
                    ++state->telem[state->whichTelem].generation;
                state->whichTelem ^= 1;
            }
        }
        // Reconfigure the channel to generate the next PWM pulse.
        DISABLE_CHANNEL(state->timer->tim, state->directChannelIndex);
        // Switch to output PWM mode.  Because the CCR has either the capture value or the last motor
        // control value (if no capture occurred), and the counter is high, the output will remain
        // low when we enable the channel.
        CAPTURE_CONFIGURE(state->timer->tim, state->directChannelIndex,
                          OC_FAST_DISABLE | OC_PRELOAD_ENABLE | OC_MODE_PWM1);
        // Put the latest motor control value in the preload CCR register.  This will be loaded when
        // the timer resets.
        *state->directChannelCCR = state->saveCCR;
        // Also put it in the timing channel, so we get an interrupt.
        *state->timingChannelCCR = state->saveCCR;
        ENABLE_CHANNEL_POLARITY(state->timer->tim, state->directChannelIndex, OC_ENABLE_POLARITY_LOW);
    } else {
        // Save the compare value used for the rising (end) edge of the PWM pullse.
        state->pwmEdge = *state->directChannelCCR;

        // Reconfigure the channel to record the falling edge of the
        // 'tick'.  We do not need to take an interrupt on capture; we
        // can record the edge when the interrupt to turn on output
        // happens.
        DISABLE_CHANNEL(state->timer->tim, state->directChannelIndex);
        CAPTURE_CONFIGURE(state->timer->tim, state->directChannelIndex,
                          IC_FILTER_FDIV1 | IC_PSC_DIV1 | IC_INPUT_DIRECT);
        ENABLE_CHANNEL_POLARITY(state->timer->tim, state->directChannelIndex, IC_ENABLE_POLARITY_FALLING);
        *state->timingChannelCCR = state->outputEnableTime;
    }
}

// Assumes timer is already set up for output.
bool castleInputConfig(const timerHardware_t* timerHardware,
                       timerChannel_t *timerChannel,
                       uint32_t hz)
{
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(timerHardware->tim);
    if (Handle == NULL)
        return false;

    if (castleState.timer) {
        return false;
    }
    // Find an unassigned capture/compare channel.
    uint8_t timingChannel = 0xFF;
    for (int8_t channelIndex = CC_CHANNELS_PER_TIMER - 1; channelIndex > 0; channelIndex--) {
        uint8_t channel = CC_CHANNEL_FROM_INDEX(channelIndex);
        if (!timerGetConfiguredByNumberAndChannel(timerGetTIMNumber(timerHardware->tim),
                                                  channel)) {
            timingChannel = channel;
            break;
        }
    }
    if (timingChannel == 0xFF) {
        /* No channels were available, so no telemetry will be collected. */
        return false;
    }
    castleState.timer = timerChannel;
    castleState.directChannelIndex = CC_INDEX_FROM_CHANNEL(timerHardware->channel);
    // The motor control code will not be writing the actual timer register.
    castleState.saveCCR = *timerChannel->ccr;
    timerChannel->ccr = &castleState.saveCCR;
    // We set an interrupt to turn on the output 25us before timer
    // reset.  It is at that time that the motor control value from the main loop in
    // castleState.saveCCR will be written to the timer.  This means the interrupt must be serviced
    // in less than 25us; measurements show it is usually less than 3us.
    uint16_t twentyFiveMicros = (25 * hz) / 1000000;
    castleState.outputEnableTime = __HAL_TIM_GET_AUTORELOAD(Handle) - twentyFiveMicros;
    castleState.directChannelCCR = timerCCR(timerHardware->tim, timerHardware->channel);
    castleState.timingChannelCCR = timerCCR(timerHardware->tim, timingChannel);

    // Initialize the timer compare register for when to turn off the output.
    TIM_OC_InitTypeDef TIM_OCInitStructure;
    // Initialize the other channel for timing.
    TIM_OCInitStructure.OCMode = TIM_OCMODE_TIMING;
    TIM_OCInitStructure.Pulse = castleState.saveCCR;

    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
    TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_LOW;
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_SET;
    TIM_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_OC_ConfigChannel(Handle, &TIM_OCInitStructure, timingChannel);

    // Configure interrupts on the timing channel.  The main channel
    // does not need interrupts.
    timerHardware_t otherHardware = *timerHardware;
    otherHardware.channel = timingChannel;
    timerChCCHandlerInit(&castleState.pwmEdgeCb, pwmEdgeCallback);
    timerChConfigCallbacks(&otherHardware, &castleState.pwmEdgeCb, NULL);
    timerNVICConfigure(timerInputIrq(timerHardware->tim));
    HAL_TIM_OC_Start_IT(Handle, otherHardware.channel);
    return true;
}

#endif // USE_TELEMETRY_CASTLE
