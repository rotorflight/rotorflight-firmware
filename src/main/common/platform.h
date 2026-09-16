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

#define NOINLINE __attribute__((noinline))

#if defined(STM32G474xx)
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "system_stm32g4xx.h"

#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_system.h"
#include "drivers/stm32g4xx_ll_ex.h"

// Chip Unique ID on G4
#define U_ID_0 (*(uint32_t*)UID_BASE)
#define U_ID_1 (*(uint32_t*)(UID_BASE + 4))
#define U_ID_2 (*(uint32_t*)(UID_BASE + 8))

#ifndef STM32G4
#define STM32G4
#endif

#elif defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H7A3xx) || defined(STM32H7A3xxQ) || defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx)
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "system_stm32h7xx.h"

#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_system.h"
#include "drivers/stm32h7xx_ll_ex.h"

// Chip Unique ID on H7
#define U_ID_0 (*(uint32_t*)UID_BASE)
#define U_ID_1 (*(uint32_t*)(UID_BASE + 4))
#define U_ID_2 (*(uint32_t*)(UID_BASE + 8))

#ifndef STM32H7
#define STM32H7
#endif

#elif defined(STM32F722xx) || defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F765xx)
#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "system_stm32f7xx.h"

#include "stm32f7xx_ll_spi.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_tim.h"
#include "stm32f7xx_ll_system.h"
#include "drivers/stm32f7xx_ll_ex.h"

// Chip Unique ID on F7
#define U_ID_0 (*(uint32_t*)UID_BASE)
#define U_ID_1 (*(uint32_t*)(UID_BASE + 4))
#define U_ID_2 (*(uint32_t*)(UID_BASE + 8))

#ifndef STM32F7
#define STM32F7
#endif

#elif defined(STM32F40_41xxx) || defined (STM32F411xE) || defined (STM32F446xx)

#include "stm32f4xx.h"

// Chip Unique ID on F405
#ifndef UID_BASE
#define UID_BASE 0x1FFF7A10UL
#endif

#define U_ID_0 (*(uint32_t*)UID_BASE)
#define U_ID_1 (*(uint32_t*)(UID_BASE + 4))
#define U_ID_2 (*(uint32_t*)(UID_BASE + 8))

#ifndef STM32F4
#define STM32F4
#endif
#elif defined(CH32H41x) || defined(CH32H4)

#include "ch32h417.h"
#include "ch32_debug.h"
#include "ch32h417_dma.h"
#include "ch32h417_rcc.h"
#include "ch32h417_spi.h"
#include "ch32h417_gpio.h"
#include "ch32h417_tim.h"
#include "ch32h417_adc.h"
#include "ch32h417_exti.h"
#include "ch32h417_usart.h"

// Chip Unique ID on CH32H41x
#define U_ID_0 (*(uint32_t*)0x1ffff7e8)
#define U_ID_1 (*(uint32_t*)0x1ffff7ec)
#define U_ID_2 (*(uint32_t*)0x1ffff7f0)

#ifndef CH32H4
#define CH32H4
#endif

// Flash configuration
#define FLASH_CONFIG_STREAMER_BUFFER_SIZE   256     // fast program is 256 bytes
#define FLASH_CONFIG_BUFFER_TYPE            uint32_t

// WCH fast interrupt attribute
#define __FAST_INTERRUPT       __attribute__((interrupt("WCH-Interrupt-fast")))

// GPIO configuration macros for CH32H4
// Bit layout: [1:0]=DIR  [3:2]=MODE  [5:4]=SPEED  [7:6]=PULL
#define GPIO_PIN_RESET       0
#define DIR_OUT              0x03
#define DIR_IN               0x00

#define GPIO_MODE_IN_AN      0x00
#define GPIO_MODE_IN_FLOAT   0x01
#define GPIO_MODE_IN_PULL    0x02

#define GPIO_MODE_OUT_PP     0x00
#define GPIO_MODE_OUT_OD     0x01
#define GPIO_MODE_OUT_AF_PP  0x02
#define GPIO_MODE_OUT_AF_OD  0x03

#define GPIO_SPEED_LOW          0x00
#define GPIO_SPEED_Medium       0x01
#define GPIO_SPEED_HIGH         0x02
#define GPIO_SPEED_VERY_HIGH    0x03

#define GPIO_PULL_NONE       0x00
#define GPIO_PULL_DOWN       0x01
#define GPIO_PULL_UP         0x02

#define IO_CONFIG(dir, mode, speed, pupd) ((dir) | ((mode) << 2) | ((speed) << 4) | ((pupd) << 6))

#define IOCFG_OUT_PP         IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define IOCFG_OUT_PP_UP      IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)
#define IOCFG_OUT_PP_25      IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define IOCFG_OUT_OD         IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_OD, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)

#define IOCFG_AF_PP          IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define IOCFG_AF_PP_PD       IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_DOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)
#define IOCFG_AF_OD          IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_OD, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define IOCFG_AF_OD_UP       IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_OD, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)

#define IOCFG_IPD            IO_CONFIG(DIR_IN, GPIO_MODE_IN_PULL, GPIO_SPEED_VERY_HIGH, GPIO_PULL_DOWN)
#define IOCFG_IPU            IO_CONFIG(DIR_IN, GPIO_MODE_IN_PULL, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)
#define IOCFG_IN_FLOATING    IO_CONFIG(DIR_IN, GPIO_MODE_IN_FLOAT, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define IOCFG_IPU_25         IO_CONFIG(DIR_IN, GPIO_MODE_IN_PULL, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)

//redefined error
// #define IO_CONFIG_GET_MODE(cfg)  (((cfg) >> 2) & 0x03)
// #define IO_CONFIG_GET_SPEED(cfg) (((cfg) >> 4) & 0x03)
// #define IO_CONFIG_GET_OTYPE(cfg) (((cfg) >> 0) & 0x03)
// #define IO_CONFIG_GET_PULL(cfg)  (((cfg) >> 6) & 0x03)

// SPI pin config macros
#define SPI_IO_AF_CFG           IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define SPI_IO_AF_SCK_CFG       IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)
#define SPI_IO_AF_SDI_CFG       IO_CONFIG(DIR_IN, GPIO_MODE_IN_FLOAT, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define SPI_IO_CS_CFG           IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define SPI_IO_CS_HIGH_CFG      IO_CONFIG(DIR_IN, GPIO_MODE_IN_PULL, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)

// SPI device count
#define SPIDEV_COUNT       4
#define MAX_SPI_PIN_SEL    5

// SPI register access macros
// #define CHECK_SPI_RX_DATA_AVAILABLE(instance)  (READ_BIT(instance->STATR, SPI_I2S_FLAG_RXNE) == (SPI_I2S_FLAG_RXNE))
// #define SPI_RX_DATA_REGISTER(base) ((base)->DATAR)

// Bit manipulation macros
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

// UART macros
#define UART_TX_BUFFER_ATTRIBUTE
#define UART_RX_BUFFER_ATTRIBUTE
#define UART_REG_RXD(base) (((USART_TypeDef *)(base))->DATAR)
#define UART_REG_TXD(base) (((USART_TypeDef *)(base))->DATAR)

// I2C and UART hardware counts
#define I2CDEV_COUNT            4
#define UARTHARDWARE_MAX_PINS   5

// NVIC priority macros
#define NVIC_PRIORITY_GROUPING  0x500
#define NVIC_BUILD_PRIORITY(base,sub) (((((base)<<(4-(7-(NVIC_PRIORITY_GROUPING>>8))))|((sub)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8)))))<<4)&0xf0)
#define NVIC_PRIORITY_BASE(prio) (((prio)>>(4-(7-(NVIC_PRIORITY_GROUPING>>8))))>>4)
#define NVIC_PRIORITY_SUB(prio) (((prio)>>4)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8))))

// USB Data Pin
#define USB_DP_PIN PB8
#elif defined(SIMULATOR_BUILD)

// Nop

#elif defined(UNIT_TEST)

#include "unittest_platform.h"

#else
#error "Invalid chipset specified. Update platform.h"
#endif

#if defined(UNIT_TEST)
#else
#include "target/common_pre.h"
#include "target.h"
#include "target/common_deprecated_post.h"
#include "target/common_post.h"
#include "target/common_defaults_post.h"
#endif
