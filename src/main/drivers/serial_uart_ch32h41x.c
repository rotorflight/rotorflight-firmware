#include <stdbool.h>
#include <stdint.h>
#include "platform.h"

#ifdef USE_UART
#include "build/debug.h"
#include "build/build_config.h"
#include "build/atomic.h"
#include "common/utils.h"
#include "drivers/inverter.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/dma.h"
#include "drivers/rcc.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#ifndef UART1_TX_DMA_CHANNEL
#define UART1_TX_DMA_CHANNEL NULL
#endif
#ifndef UART1_RX_DMA_CHANNEL
#define UART1_RX_DMA_CHANNEL NULL
#endif
#ifndef UART2_TX_DMA_CHANNEL
#define UART2_TX_DMA_CHANNEL NULL
#endif
#ifndef UART2_RX_DMA_CHANNEL
#define UART2_RX_DMA_CHANNEL NULL
#endif
#ifndef UART3_TX_DMA_CHANNEL
#define UART3_TX_DMA_CHANNEL NULL
#endif
#ifndef UART3_RX_DMA_CHANNEL
#define UART3_RX_DMA_CHANNEL NULL
#endif
#ifndef UART4_TX_DMA_CHANNEL
#define UART4_TX_DMA_CHANNEL NULL
#endif
#ifndef UART4_RX_DMA_CHANNEL
#define UART4_RX_DMA_CHANNEL NULL
#endif
#ifndef UART5_TX_DMA_CHANNEL
#define UART5_TX_DMA_CHANNEL NULL
#endif
#ifndef UART5_RX_DMA_CHANNEL
#define UART5_RX_DMA_CHANNEL NULL
#endif
#ifndef UART6_RX_DMA_CHANNEL
#define UART6_RX_DMA_CHANNEL NULL
#endif
#ifndef UART6_TX_DMA_CHANNEL
#define UART6_TX_DMA_CHANNEL NULL
#endif
#ifndef UART7_RX_DMA_CHANNEL
#define UART7_RX_DMA_CHANNEL NULL
#endif
#ifndef UART7_TX_DMA_CHANNEL
#define UART7_TX_DMA_CHANNEL NULL
#endif
#ifndef UART8_TX_DMA_CHANNEL
#define UART8_TX_DMA_CHANNEL NULL
#endif
#ifndef UART8_RX_DMA_CHANNEL
#define UART8_RX_DMA_CHANNEL NULL
#endif

// // TX/RX buffers
// #ifdef USE_UART1
// static uint8_t uart1TxBuffer[UART_TX_BUFFER_SIZE];
// static uint8_t uart1RxBuffer[UART_RX_BUFFER_SIZE];
// #endif
// #ifdef USE_UART2
// static uint8_t uart2TxBuffer[UART_TX_BUFFER_SIZE];
// static uint8_t uart2RxBuffer[UART_RX_BUFFER_SIZE];
// #endif
// #ifdef USE_UART3
// static uint8_t uart3TxBuffer[UART_TX_BUFFER_SIZE];
// static uint8_t uart3RxBuffer[UART_RX_BUFFER_SIZE];
// #endif
// #ifdef USE_UART4
// static uint8_t uart4TxBuffer[UART_TX_BUFFER_SIZE];
// static uint8_t uart4RxBuffer[UART_RX_BUFFER_SIZE];
// #endif
// #ifdef USE_UART5
// static uint8_t uart5TxBuffer[UART_TX_BUFFER_SIZE];
// static uint8_t uart5RxBuffer[UART_RX_BUFFER_SIZE];
// #endif
// #ifdef USE_UART6
// static uint8_t uart6TxBuffer[UART_TX_BUFFER_SIZE];
// static uint8_t uart6RxBuffer[UART_RX_BUFFER_SIZE];
// #endif
// #ifdef USE_UART7
// static uint8_t uart7TxBuffer[UART_TX_BUFFER_SIZE];
// static uint8_t uart7RxBuffer[UART_RX_BUFFER_SIZE];
// #endif
// #ifdef USE_UART8
// static uint8_t uart8TxBuffer[UART_TX_BUFFER_SIZE];
// static uint8_t uart8RxBuffer[UART_RX_BUFFER_SIZE];
// #endif

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART1
    {
        .device = UARTDEV_1,
        .reg = USART1,
#ifdef USE_DMA
        .rxDMAResource = (dmaResource_t *)UART1_RX_DMA_CHANNEL,
        .txDMAResource = (dmaResource_t *)UART1_TX_DMA_CHANNEL,
        .rxDMAChannel = 0,
        .txDMAChannel = 0,
#endif
        .rxPins = {
            {DEFIO_TAG_E(PA10), GPIO_AF7},
            {DEFIO_TAG_E(PB7), GPIO_AF7},
            {DEFIO_TAG_E(PB15), GPIO_AF4},
            {DEFIO_TAG_E(PD12), GPIO_AF14},
        },
        .txPins = {
            {DEFIO_TAG_E(PA9), GPIO_AF7},
            {DEFIO_TAG_E(PB6), GPIO_AF7},
            {DEFIO_TAG_E(PB14), GPIO_AF4},
            {DEFIO_TAG_E(PD13), GPIO_AF14},
        },
        .rcc = RCC_HB2(USART1),
        .irqn = USART1_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART1,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
    },
#endif
#ifdef USE_UART2
    {
        .device = UARTDEV_2,
        .reg = USART2,
#ifdef USE_DMA
        .rxDMAResource = (dmaResource_t *)UART2_RX_DMA_CHANNEL,
        .txDMAResource = (dmaResource_t *)UART2_TX_DMA_CHANNEL,
        .rxDMAChannel = 0,
        .txDMAChannel = 0,
#endif
        .rxPins = {
            {DEFIO_TAG_E(PA3), GPIO_AF7},
            {DEFIO_TAG_E(PD6), GPIO_AF7},
        },
        .txPins = {
            {DEFIO_TAG_E(PA2), GPIO_AF7},
            {DEFIO_TAG_E(PD5), GPIO_AF7},
        },
        .rcc = RCC_HB1(USART2),
        .irqn = USART2_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART2_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART2,
        .txBuffer = uart2TxBuffer,
        .rxBuffer = uart2RxBuffer,
        .txBufferSize = sizeof(uart2TxBuffer),
        .rxBufferSize = sizeof(uart2RxBuffer),
    },
#endif
#ifdef USE_UART3
    {
        .device = UARTDEV_3,
        .reg = USART3,
#ifdef USE_DMA
        .rxDMAResource = (dmaResource_t *)UART3_RX_DMA_CHANNEL,
        .txDMAResource = (dmaResource_t *)UART3_TX_DMA_CHANNEL,
        .rxDMAChannel = 0,
        .txDMAChannel = 0,
#endif
        .rxPins = {
            {DEFIO_TAG_E(PB11), GPIO_AF7},
            {DEFIO_TAG_E(PC11), GPIO_AF7},
            {DEFIO_TAG_E(PD9), GPIO_AF7},
            {DEFIO_TAG_E(PA14), GPIO_AF4},
        },
        .txPins = {
            {DEFIO_TAG_E(PB10), GPIO_AF7},
            {DEFIO_TAG_E(PC10), GPIO_AF7},
            {DEFIO_TAG_E(PD8), GPIO_AF7},
            {DEFIO_TAG_E(PA13), GPIO_AF4},
        },
        .rcc = RCC_HB1(USART3),
        .irqn = USART3_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART3_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART3,
        .txBuffer = uart3TxBuffer,
        .rxBuffer = uart3RxBuffer,
        .txBufferSize = sizeof(uart3TxBuffer),
        .rxBufferSize = sizeof(uart3RxBuffer),
    },
#endif
#ifdef USE_UART4
    {
        .device = UARTDEV_4,
        .reg = USART4,
#ifdef USE_DMA
        .rxDMAResource = (dmaResource_t *)UART4_RX_DMA_CHANNEL,
        .txDMAResource = (dmaResource_t *)UART4_TX_DMA_CHANNEL,
        .rxDMAChannel = 0,
        .txDMAChannel = 0,
#endif
        .rxPins = {
            {DEFIO_TAG_E(PF3), GPIO_AF7},
            {DEFIO_TAG_E(PC7), GPIO_AF7},
        },
        .txPins = {
            {DEFIO_TAG_E(PF4), GPIO_AF7},
            {DEFIO_TAG_E(PC6), GPIO_AF7},
        },
        .rcc = RCC_HB1(USART4),
        .irqn = USART4_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART4_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART4,
        .txBuffer = uart4TxBuffer,
        .rxBuffer = uart4RxBuffer,
        .txBufferSize = sizeof(uart4TxBuffer),
        .rxBufferSize = sizeof(uart4RxBuffer),
    },
#endif
#ifdef USE_UART5
    {
        .device = UARTDEV_5,
        .reg = USART5,
#ifdef USE_DMA
        .rxDMAResource = (dmaResource_t *)UART5_RX_DMA_CHANNEL,
        .txDMAResource = (dmaResource_t *)UART5_TX_DMA_CHANNEL,
        .rxDMAChannel = 0,
        .txDMAChannel = 0,
#endif
        .rxPins = {
            {DEFIO_TAG_E(PE2), GPIO_AF4},
            {DEFIO_TAG_E(PF5), GPIO_AF4},
        },
        .txPins = {
            {DEFIO_TAG_E(PE3), GPIO_AF11},
            {DEFIO_TAG_E(PE0), GPIO_AF4},
        },
        .rcc = RCC_HB1(USART5),
        .irqn = USART5_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART5_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART5,
        .txBuffer = uart5TxBuffer,
        .rxBuffer = uart5RxBuffer,
        .txBufferSize = sizeof(uart5TxBuffer),
        .rxBufferSize = sizeof(uart5RxBuffer),
    },
#endif
#ifdef USE_UART6
    {
        .device = UARTDEV_6,
        .reg = USART6,
#ifdef USE_DMA
        .rxDMAResource = (dmaResource_t *)UART6_RX_DMA_CHANNEL,
        .txDMAResource = (dmaResource_t *)UART6_TX_DMA_CHANNEL,
        .rxDMAChannel = 0,
        .txDMAChannel = 0,
#endif
        .rxPins = {
            {DEFIO_TAG_E(PA1), GPIO_AF8},
            {DEFIO_TAG_E(PA11), GPIO_AF6},
            {DEFIO_TAG_E(PB8), GPIO_AF8},
            {DEFIO_TAG_E(PC11), GPIO_AF8},
            {DEFIO_TAG_E(PD0), GPIO_AF8},
        },
        .txPins = {
            {DEFIO_TAG_E(PA0), GPIO_AF8},
            {DEFIO_TAG_E(PA12), GPIO_AF6},
            {DEFIO_TAG_E(PB9), GPIO_AF8},
            {DEFIO_TAG_E(PC10), GPIO_AF8},
            {DEFIO_TAG_E(PD1), GPIO_AF8},
        },
        .rcc = RCC_HB1(USART6),
        .irqn = USART6_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART6_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART6,
        .txBuffer = uart6TxBuffer,
        .rxBuffer = uart6RxBuffer,
        .txBufferSize = sizeof(uart6TxBuffer),
        .rxBufferSize = sizeof(uart6RxBuffer),
    },
#endif
#ifdef USE_UART7
    {
        .device = UARTDEV_7,
        .reg = USART7,
#ifdef USE_DMA
        .rxDMAResource = (dmaResource_t *)UART7_RX_DMA_CHANNEL,
        .txDMAResource = (dmaResource_t *)UART7_TX_DMA_CHANNEL,
        .rxDMAChannel = 0,
        .txDMAChannel = 0,
#endif
        .rxPins = {
            {DEFIO_TAG_E(PB5), GPIO_AF14},
            {DEFIO_TAG_E(PB12), GPIO_AF14},
            {DEFIO_TAG_E(PD2), GPIO_AF8},
        },
        .txPins = {
            {DEFIO_TAG_E(PB6), GPIO_AF14},
            {DEFIO_TAG_E(PB13), GPIO_AF14},
            {DEFIO_TAG_E(PC12), GPIO_AF8},
        },
        .rcc = RCC_HB1(USART7),
        .irqn = USART7_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART7_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART7,
        .txBuffer = uart7TxBuffer,
        .rxBuffer = uart7RxBuffer,
        .txBufferSize = sizeof(uart7TxBuffer),
        .rxBufferSize = sizeof(uart7RxBuffer),
    },
#endif
#ifdef USE_UART8
    {
        .device = UARTDEV_8,
        .reg = USART8,
#ifdef USE_DMA
        .rxDMAResource = (dmaResource_t *)UART8_RX_DMA_CHANNEL,
        .txDMAResource = (dmaResource_t *)UART8_TX_DMA_CHANNEL,
        .rxDMAChannel = 0,
        .txDMAChannel = 0,
#endif
        .rxPins = {
            {DEFIO_TAG_E(PA8), GPIO_AF11},
            {DEFIO_TAG_E(PB3), GPIO_AF11},
            {DEFIO_TAG_E(PE7), GPIO_AF7},
            {DEFIO_TAG_E(PF6), GPIO_AF7},
        },
        .txPins = {
            {DEFIO_TAG_E(PA15), GPIO_AF11},
            {DEFIO_TAG_E(PB4), GPIO_AF11},
            {DEFIO_TAG_E(PE8), GPIO_AF7},
            {DEFIO_TAG_E(PF7), GPIO_AF7},
        },
        .rcc = RCC_HB1(USART8),
        .irqn = USART8_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART8_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART8,
        .txBuffer = uart8TxBuffer,
        .rxBuffer = uart8RxBuffer,
        .txBufferSize = sizeof(uart8TxBuffer),
        .rxBufferSize = sizeof(uart8RxBuffer),
    },
#endif
};

uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartDevice_t *uart = uartDevmap[device];
    if (!uart)
        return NULL;

    const uartHardware_t *hardware = uart->hardware;
    if (!hardware)
        return NULL;

    uartPort_t *s = &(uart->port);
    s->port.vTable = uartVTable;
    s->port.baudRate = baudRate;
    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;
    s->USARTx = hardware->reg;

#ifdef USE_DMA
    uartConfigureDma(uart);
#endif

    IO_t txIO = IOGetByTag(uart->tx.pin);
    IO_t rxIO = IOGetByTag(uart->rx.pin);

    if (hardware->rcc)
    {
        RCC_ClockCmd(hardware->rcc, ENABLE);
    }

    if ((options & SERIAL_BIDIR) && txIO)
    {
        IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        IOConfigGPIOAF(txIO, ((options & SERIAL_BIDIR_PP) || (options & SERIAL_BIDIR_PP_PD)) ? IOCFG_AF_PP : IOCFG_AF_OD_UP, uart->tx.af);
    }
    else
    {
        if ((mode & MODE_TX) && txIO)
        {
            IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(txIO, IOCFG_AF_PP_UP, uart->tx.af);
        }

        if ((mode & MODE_RX) && rxIO)
        {
            IOInit(rxIO, OWNER_SERIAL_RX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(rxIO, IOCFG_AF_PP_UP, uart->rx.af);
        }
    }

#ifdef USE_DMA
    if (!s->rxDMAResource)
#endif
    {
      NVIC_SetPriority(hardware->irqn, hardware->rxPriority);
      NVIC_EnableIRQ(hardware->irqn);
      }

    return s;
}



FAST_IRQ_HANDLER void uartIrqHandler(uartPort_t *s)
{
    USART_TypeDef *USARTx = (USART_TypeDef *)s->USARTx;
    if (!s->rxDMAResource && (USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == SET))
    {
        if (s->port.rxCallback)
        {
            s->port.rxCallback(USARTx->DATAR, s->port.rxCallbackData);
        }
        else
        {
            s->port.rxBuffer[s->port.rxBufferHead] = USARTx->DATAR;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
    }
    if ((USART_GetFlagStatus(USARTx, USART_FLAG_TC) != RESET))
    {
        USART_ClearFlag(USARTx, USART_FLAG_TC);
        
    }
    if (!s->txDMAResource && (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == SET))
    {
        if (s->port.txBufferTail != s->port.txBufferHead)
        {
            USART_SendData(USARTx, s->port.txBuffer[s->port.txBufferTail]);
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        }
        else
        {
            USART_ITConfig(USARTx, USART_IT_TXE, DISABLE);
        }
    }
    if (USART_GetFlagStatus(USARTx, USART_FLAG_ORE) == SET)
    {
        USART_ClearFlag(USARTx, USART_FLAG_ORE);
        (void)USARTx->STATR;
        (void)USARTx->DATAR;
    }
    if (USART_GetFlagStatus(USARTx, USART_FLAG_IDLE) == SET)
    {
        if (s->port.idleCallback)
        {
            s->port.idleCallback();
        }
        (void)USARTx->STATR;
        (void)USARTx->DATAR;
    }
}

#endif // USE_UART
