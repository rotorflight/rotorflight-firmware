/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
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

/*
 * CH32H417 SPI driver for Rotorflight
 * Based on betaflight X32 bus_spi_x32.c by Temperslee
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SPI

#include "common/maths.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"
#include "drivers/dma.h"
#include "drivers/nvic.h"

// CH32H417 uses CTLR1 instead of CR1, Mode-based prescalers
#define IS_CCM(p) (((uint32_t)p & 0xffff0000) == 0x10000000)

static SPI_InitTypeDef defaultInit = {
    .SPI_Direction = SPI_Direction_2Lines_FullDuplex,
    .SPI_Mode = SPI_Mode_Master,
    .SPI_DataSize = SPI_DataSize_8b,
    .SPI_CPOL = SPI_CPOL_High,
    .SPI_CPHA = SPI_CPHA_2Edge,
    .SPI_NSS = SPI_NSS_Soft,
    .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_Mode3, // ~fPCLK/8 equivalent
    .SPI_FirstBit = SPI_FirstBit_MSB,
    .SPI_CRCPolynomial = 7,
};

// BR bits mask in CTLR1 register
#define BR_BITS (SPI_BaudRatePrescaler_Mode7)

// Map STM32-style divisor values to CH32 prescaler modes
static uint16_t spiDivisorToBRbits(SPI_TypeDef *instance, uint16_t divisor)
{
    UNUSED(instance);
    // CH32H417 uses Mode0-Mode7 for prescaler
    // Mode0=2, Mode1=4, Mode2=8, Mode3=16, Mode4=32, Mode5=64, Mode6=128, Mode7=256
    divisor = constrain(divisor, 2, 256);

    if (divisor <= 2)
        return SPI_BaudRatePrescaler_Mode0;
    if (divisor <= 4)
        return SPI_BaudRatePrescaler_Mode1;
    if (divisor <= 8)
        return SPI_BaudRatePrescaler_Mode2;
    if (divisor <= 16)
        return SPI_BaudRatePrescaler_Mode3;
    if (divisor <= 32)
        return SPI_BaudRatePrescaler_Mode4;
    if (divisor <= 64)
        return SPI_BaudRatePrescaler_Mode5;
    if (divisor <= 128)
        return SPI_BaudRatePrescaler_Mode6;
    return SPI_BaudRatePrescaler_Mode7;
}

void spiSetDivisorBRreg(SPI_TypeDef *instance, uint16_t divisor)
{
    // CH32H417 uses CTLR1 instead of CR1
    const uint16_t tempRegister = (instance->CTLR1 & ~BR_BITS);
    instance->CTLR1 = tempRegister | spiDivisorToBRbits(instance, divisor);
}

void spiInitDevice(SPIDevice device)
{
    spiDevice_t *spi = &(spiDevice[device]);

    if (!spi->dev)
    {
        return;
    }

    // Enable SPI clock
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);

    IOInit(IOGetByTag(spi->sck), OWNER_SPI_SCK, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_MISO, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_MOSI, RESOURCE_INDEX(device));

    IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG, spi->sckAF);
    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_SDI_CFG, spi->misoAF);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->mosiAF);

    // Init SPI hardware
    SPI_I2S_DeInit(spi->dev);
    SPI_Init(spi->dev, &defaultInit);
    SPI_Cmd(spi->dev, ENABLE);

    // Drive NSS high to signal slave deselect
    SPI_NSSInternalSoftwareConfig(spi->dev, SPI_NSSInternalSoft_Set);
}



// Internal DMA stream helpers (minimal stubs - full DMA SPI not yet implemented)
void spiInternalInitStream(const extDevice_t *dev, bool preInit)
{
    UNUSED(dev);
    UNUSED(preInit);
    // TODO: Implement CH32H4 SPI DMA stream initialization
}

void spiInternalStartDMA(const extDevice_t *dev)
{
    UNUSED(dev);
    // TODO: Implement CH32H4 SPI DMA start
}

void spiInternalStopDMA(const extDevice_t *dev)
{
    UNUSED(dev);
    // TODO: Implement CH32H4 SPI DMA stop
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    UNUSED(descriptor);
    // TODO: Implement CH32H4 SPI DMA stream reset
}

void spiInternalResetDescriptors(busDevice_t *bus)
{
    UNUSED(bus);
    // TODO: Implement CH32H4 SPI DMA descriptor reset
}

// Polled SPI transfer for a single segment
static bool spiTransferSegmentPolled(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
    for (int i = 0; i < len; i++)
    {
        uint8_t b = txData ? txData[i] : 0xFF;

        /* Wait for TX empty — timeout after ~100k cycles to avoid infinite hang */
        uint32_t timeout = 100000;
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET) {
            if (--timeout == 0) return false;
        }
        SPI_I2S_SendData(instance, b);

        /* Wait for RX not-empty — timeout after ~100k cycles */
        timeout = 100000;
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET) {
            if (--timeout == 0) return false;
        }
        b = SPI_I2S_ReceiveData(instance);

        if (rxData)
        {
            rxData[i] = b;
        }
    }

    return true;
}

void spiSequenceStart(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    SPI_TypeDef *instance = bus->busType_u.spi.instance;

    // Process segments in polled mode (DMA to be implemented later)
    volatile busSegment_t *lastSegment = NULL;
    while (bus->curSegment->len > 0)
    {
        // Assert CS
        if (!lastSegment || lastSegment->negateCS)
        {
            IOLo(dev->busType_u.spi.csnPin);
        }

        spiTransferSegmentPolled(
            instance,
            bus->curSegment->u.buffers.txData,
            bus->curSegment->u.buffers.rxData,
            bus->curSegment->len);

        if (bus->curSegment->negateCS)
        {
            IOHi(dev->busType_u.spi.csnPin);
        }

        if (bus->curSegment->callback)
        {
            switch (bus->curSegment->callback(dev->callbackArg))
            {
            case BUS_BUSY:
                bus->curSegment--;
                break;
            case BUS_ABORT:
                bus->curSegment = (volatile busSegment_t *)BUS_SPI_FREE;
                return;
            case BUS_READY:
            default:
                break;
            }
        }
        lastSegment = bus->curSegment;
        bus->curSegment++;
    }
    if (bus->curSegment->u.link.dev)
    {
        const extDevice_t *nextDev = bus->curSegment->u.link.dev;
        volatile busSegment_t *nextSegments = bus->curSegment->u.link.segments;
        volatile busSegment_t *endSegment = bus->curSegment;
        bus->curSegment = nextSegments;
        endSegment->u.link.dev = NULL;
        spiSequenceStart(nextDev);
    }
    else
    {
        bus->curSegment = (volatile busSegment_t *)BUS_SPI_FREE;
    }
}

#endif // USE_SPI
