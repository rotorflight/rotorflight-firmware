#include <stdint.h>

#include "platform.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

// FC-30_H415-v0.3 motor/servo PWM outputs (from schematic)
//   M1..M4 = SIG1..SIG4 (CN13) on TIM8 CH1-4  (PE3-PE6)
//   SERVO1 = H1 header                        (PB11, TIM2_CH4)
//   LED_STRIP pad                            (PD3, TIM11_CH1) - WS2812
// Motor/servo outputs use their own DMA channel (dmaopt 0-4 -> DMA1_CH1-5),
// the LED strip uses DMA1_CH7 (dmaopt 6) to avoid conflicts.
// Note: PB6 is UART1_TX, not available for LED strip.
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM8, CH1, PE3, TIM_USE_MOTOR, 0, 0),  // M1 / SIG1
    DEF_TIM(TIM8, CH2, PE4, TIM_USE_MOTOR, 0, 1),  // M2 / SIG2
    DEF_TIM(TIM8, CH3, PE5, TIM_USE_MOTOR, 0, 2),  // M3 / SIG3
    DEF_TIM(TIM8, CH4, PE6, TIM_USE_MOTOR, 0, 3),  // M4 / SIG4
    DEF_TIM(TIM2, CH4, PB11, TIM_USE_SERVO, 0, 5), // SERVO1 (H2 header)
    DEF_TIM(TIM11, CH1, PD3, TIM_USE_LED, 0, 4),   // LED_STRIP (WS2812)
};