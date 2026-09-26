#include <stdint.h>

#include "platform.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

//  motor/servo PWM outputs (from schematic)
//   Servos S1-S3 = PE3-PE5 (TIM8_CH1-3, AF0)
//   Main Motor (ESC) = PE6 (TIM12_CH4, AF3)
//   Tail Motor = PB11 (TIM2_CH4, AF1)
//   LED_STRIP pad = PD3 (TIM11_CH1, AF2)
// NOTE: PE6 also supports TIM8_CH4 (AF0), but TIM8 is shared with the servo
// channels S1-S3. servoInit() runs after motor init and reconfigures the TIM8
// timebase for the servo rate, which breaks the ESC pulse scaling. PE6's
// alternate function TIM12_CH4 (AF3, per datasheet Table 2-2-4) gives the ESC
// its own timer, so motor and servo timing are independent.
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM8, CH1, PE3, TIM_USE_SERVO, 0, 0),  // S1 (Servo 1)
    DEF_TIM(TIM8, CH2, PE4, TIM_USE_SERVO, 0, 1),  // S2 (Servo 2)
    DEF_TIM(TIM8, CH3, PE5, TIM_USE_SERVO, 0, 2),  // S3 (Servo 3)
    DEF_TIM(TIM12, CH4, PE6, TIM_USE_MOTOR, 0, 6), // Main Motor (ESC - PWM)
    DEF_TIM(TIM2, CH4, PB11, TIM_USE_MOTOR, 0, 4), // Tail Motor (PWM/DShot)
    DEF_TIM(TIM11, CH1, PD3, TIM_USE_LED, 0, 5),   // LED_STRIP (WS2812)
};