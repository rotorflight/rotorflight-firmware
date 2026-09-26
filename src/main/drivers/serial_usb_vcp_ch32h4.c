/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_VCP

#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/usb_io.h"

#include "pg/usb.h"

#include "ch32_debug.h"
#include "cdc_vcp_ch32h41x.h"
#include "usbd_core.h"
#include "usbd_cdc_acm.h"
#include "usb_ch32h41x_usbhs_reg.h"

#include "drivers/time.h"
#include "drivers/serial.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/nvic.h"

#define USB_TIMEOUT  50

static vcpPort_t vcpPort = {0};

// #define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048

#define APP_TX_BLOCK_SIZE 512

// volatile uint8_t UserRxBuffer[APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
volatile uint8_t UserTxBuffer[APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */
uint32_t BuffLength;

/* Increment this pointer or roll it back to start address when data are received over USART */
volatile uint32_t UserTxBufPtrIn = 0;
/* Increment this pointer or roll it back to start address when data are sent over USB */
volatile uint32_t UserTxBufPtrOut = 0;

// volatile uint32_t APP_Rx_ptr_out = 0;
// volatile uint32_t APP_Rx_ptr_in = 0;
// static uint8_t APP_Rx_Buffer[APP_RX_DATA_SIZE];


// #define  CDC_POLLING_INTERVAL 5



void (*ctrlLineStateCb)(void* context, uint16_t ctrlLineState) = NULL;
void *ctrlLineStateCbContext = NULL;
void (*baudRateCb)(void *context, uint32_t baud) = NULL;
void *baudRateCbContext = NULL;

void CDC_SetBaudRateCb(void (*cb)(void *context, uint32_t baud), void *context)
{
    baudRateCbContext = context;
    baudRateCb = cb;
}

void CDC_SetCtrlLineStateCb(void (*cb)(void *context, uint16_t ctrlLineState), void *context)
{
    ctrlLineStateCbContext = context;
    ctrlLineStateCb = cb;
}



uint32_t CDC_Send_FreeBytes(void)
{
    uint32_t freeBytes;

    ATOMIC_BLOCK(NVIC_BUILD_PRIORITY(6, 0)) {
        freeBytes = ((UserTxBufPtrOut - UserTxBufPtrIn) + (-((int)(UserTxBufPtrOut <= UserTxBufPtrIn)) & APP_TX_DATA_SIZE)) - 1;
    }

    return freeBytes;
}

static volatile uint32_t lastBuffsize = 0;

void usb_vcp_tx_flush(void)
{
    if (ep_tx_busy_flag != 0) {
        return;
    }

    if (lastBuffsize) {
        bool needZeroLengthPacket = (lastBuffsize % CDC_MAX_MPS == 0);
        UserTxBufPtrOut = (UserTxBufPtrOut + lastBuffsize) % APP_TX_DATA_SIZE;
        lastBuffsize = 0;
        if (needZeroLengthPacket) {
            usb_vcp_send_data(0, (uint8_t*)&UserTxBuffer[UserTxBufPtrOut], 0);
            return;
        }
    }

    if (UserTxBufPtrOut != UserTxBufPtrIn) {
        uint32_t buffsize;
        if (UserTxBufPtrOut > UserTxBufPtrIn) {
            buffsize = APP_TX_DATA_SIZE - UserTxBufPtrOut;
        } else {
            buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
        }
        if (buffsize > APP_TX_BLOCK_SIZE) {
            buffsize = APP_TX_BLOCK_SIZE;
        }
        uint32_t txed = usb_vcp_send_data(0, (uint8_t*)&UserTxBuffer[UserTxBufPtrOut], buffsize);
        if (txed == 0) {
            lastBuffsize = buffsize;
        }
    }
}

uint32_t CDC_Send_DATA(const uint8_t *ptrBuffer, uint32_t sendLength)
{
    for (uint32_t i = 0; i < sendLength; i++) {
        while (CDC_Send_FreeBytes() == 0) {
            delay(1);
        }
        ATOMIC_BLOCK(NVIC_BUILD_PRIORITY(6, 0)) {
            UserTxBuffer[UserTxBufPtrIn] = ptrBuffer[i];
            UserTxBufPtrIn = (UserTxBufPtrIn + 1) % APP_TX_DATA_SIZE;
        }
    }
    /* Do NOT call usb_vcp_tx_flush() here — matching betaflight.
     * The SOF handler is solely responsible for flushing TX data.
     * Calling flush from both main context and SOF interrupt creates
     * a race condition on lastBuffsize/UserTxBufPtrOut shared state. */
    return sendLength;
}

void usb_int_rxsof_handler(void)
{
    /* Flush TX buffer every 16th SOF frame (~16ms) to match betaflight.
     * Flushing on every SOF (1ms) overwhelms the USB TX state machine
     * and can keep ep_tx_busy_flag permanently set, blocking MSP responses. */
    static uint8_t FrameCount = 0;
    if (FrameCount++ == 16) {
        FrameCount = 0;
        usb_vcp_tx_flush();
    }
}

uint8_t usbIsConnected(void)
{
    return ((usbd_cdc_info != USBD_EVENT_DISCONNECTED) && (usbd_cdc_info != USBD_EVENT_UNKNOWN));
}

uint8_t usbIsConfigured(void)
{
    return (usbd_cdc_info == USBD_EVENT_CONFIGURED);
}

uint8_t usbVcpIsConnected(void)
{
    return usbIsConnected();
}



static void usbVcpSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    UNUSED(instance);
    UNUSED(baudRate);
}

static void usbVcpSetMode(serialPort_t *instance, portMode_e mode)
{
    UNUSED(instance);
    UNUSED(mode);
}

static void usbVcpSetCtrlLineStateCb(serialPort_t *instance, void (*cb)(void *context, uint16_t ctrlLineState), void *context)
{
    UNUSED(instance);
    CDC_SetCtrlLineStateCb((void (*)(void *context, uint16_t ctrlLineState))cb, context);
}

static void usbVcpSetBaudRateCb(serialPort_t *instance, void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context)
{
    UNUSED(instance);
    CDC_SetBaudRateCb((void (*)(void *context, uint32_t baud))cb, (void *)context);
}

static bool isUsbVcpTransmitBufferEmpty(const serialPort_t *instance)
{
    UNUSED(instance);
    return true;
}

static uint32_t usbVcpAvailable(const serialPort_t *instance)
{
    UNUSED(instance);

//     uint32_t available = APP_Rx_ptr_in-APP_Rx_ptr_out;

//     // Return the sum of the bytes in the APP_Rx_Buffer buffer and those received by the VCP driver
//   if(ep_rx_finish == 1)
//     {
//         available += ep_rx_length;
//     }

//     return available;

    return usb_vcp_rx_available( ); 
}

// static uint8_t usbVcpRead(serialPort_t *instance)
// {
//     UNUSED(instance);
//     uint8_t buf[1] = {0};
//     usb_vcp_get_rx_data(0, buf, 1);
//     return buf[0];
// }
static uint8_t usbVcpRead(serialPort_t *instance)
{
    UNUSED(instance);
    uint8_t buf[1];
    uint32_t start = millis();
    while (true)
    {
        if (usb_vcp_get_rx_data(0, buf, 1))
            return buf[0];
        /* Safety timeout: if usbVcpAvailable() reported data but a race
         * condition consumed it before we could read, don't deadlock the
         * scheduler forever. 10ms is generous for a single byte. */
        if (millis() - start > 10)
            return 0;
    }
}
static void usbVcpWriteBuf(serialPort_t *instance, const void *data, int count)
{
    UNUSED(instance);

    uint32_t start = millis();
    const uint8_t *p = data;
    while (count > 0) {
        uint32_t txed = CDC_Send_DATA(p, count);
        count -= txed;
        p += txed;

        if (millis() - start > USB_TIMEOUT) {
            break;
        }
    }
}

static bool usbVcpFlush(vcpPort_t *port)
{
    uint32_t count = port->txAt;

    if (count == 0) {
        return true;
    }

    if (!usbIsConnected() || !usbIsConfigured()) {
        return false;
    }

    port->txAt = 0;

    uint32_t start = millis();
    uint8_t *p = port->txBuf;
    while (count > 0) {
        uint32_t txed = CDC_Send_DATA(p, count);
        count -= txed;
        p += txed;

        if (millis() - start > USB_TIMEOUT) {
            break;
        }
    }
    return count == 0;
}
static void usbVcpWrite(serialPort_t *instance, uint8_t c)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);

    port->txBuf[port->txAt++] = c;
    if (!port->buffering || port->txAt >= ARRAYLEN(port->txBuf)) {
        usbVcpFlush(port);
    }
}

static void usbVcpBeginWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = true;
}

static uint32_t usbTxBytesFree(const serialPort_t *instance)
{
    UNUSED(instance);
    return CDC_Send_FreeBytes();
}

static void usbVcpEndWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = false;
    usbVcpFlush(port);
}

static const struct serialPortVTable usbVTable[] = {
    {
        .serialWrite = usbVcpWrite,
        .serialTotalRxWaiting = usbVcpAvailable,
        .serialTotalTxFree = usbTxBytesFree,
        .serialRead = usbVcpRead,
        .serialSetBaudRate = usbVcpSetBaudRate,
        .isSerialTransmitBufferEmpty = isUsbVcpTransmitBufferEmpty,
        .setMode = usbVcpSetMode,
        .setCtrlLineStateCb = usbVcpSetCtrlLineStateCb,
        .setBaudRateCb = usbVcpSetBaudRateCb,
        .writeBuf =  usbVcpWriteBuf,
        .beginWrite = usbVcpBeginWrite,
        .endWrite = usbVcpEndWrite
    }
};

/* usb_dc_low_level_init() is called internally by usb_dc_init() inside
 * the CherryUSB stack (usbd_initialize → usb_dc_init → usb_dc_low_level_init).
 * All RCC/PLL/NVIC setup is now done in usbVcpInit() before cdc_acm_init()
 * is called, matching betaflight's proven init sequence. This stub satisfies
 * the CherryUSB callback without duplicating setup. */
void usb_dc_low_level_init(void)
{
    /* Intentionally empty — all low-level init is done in usbVcpInit() */
}

void usbVcpInit(void)
{
    static bool isUsbHwInitialized = false;
    if (isUsbHwInitialized) {
        return;
    }
    isUsbHwInitialized = true;

    /* Enable GPIO clocks and disable SWJ remap (matches betaflight) */
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOB, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

    IOInit(IOGetByTag(IO_TAG(PB8)), OWNER_USB, 0);
    IOInit(IOGetByTag(IO_TAG(PB9)), OWNER_USB, 0);

    usbGenerateDisconnectPulse();

    /* Configure USBHS 480MHz PLL (matches betaflight exactly) */
    if ((RCC->PLLCFGR & RCC_SYSPLL_SEL) != RCC_SYSPLL_USBHS)
    {
        RCC_USBHS_PLLCmd(DISABLE);
        RCC_USBHSPLLCLKConfig(RCC_USBHSPLLSource_HSI);
        RCC_USBHSPLLReferConfig(RCC_USBHSPLLRefer_25M);
        RCC_USBHSPLLClockSourceDivConfig(RCC_USBHSPLL_IN_Div1);
        RCC_USBHS_PLLCmd(ENABLE);
        while (!(RCC->CTLR & RCC_USBHS_PLLRDY));
    }
    /* Enable UTMI Clock */
    RCC_UTMIcmd(ENABLE);
    /* Enable USBHS Clock */
    RCC_HBPeriphClockCmd(RCC_HBPeriph_USBHS, ENABLE);

    usb_rxsof_handler = usb_int_rxsof_handler;

    /* Initialize CherryUSB CDC ACM stack */
    cdc_acm_init(0, 0);

    /* Enable USBHS IRQ AFTER cdc_acm_init() to avoid spurious interrupts
     * during hardware configuration (matches betaflight order) */
    NVIC_SetPriority(USBHS_IRQn, NVIC_PRIO_USB);
    NVIC_EnableIRQ(USBHS_IRQn);
}


serialPort_t *usbVcpOpen(void)
{
    /* usbVcpInit() has its own guard, so this is safe even if called
     * before the early init in init.c or after it. */
    usbVcpInit();

    vcpPort_t *s = &vcpPort;
    s->port.vTable = usbVTable;
    return &s->port;
}

uint32_t usbVcpGetBaudRate(serialPort_t *instance)
{
    UNUSED(instance);
    return cdc_vcp_line_coding.dwDTERate;
}

#endif
