/*
 * AT32 SDIO1 backend for the Rotorflight SD card state machine.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "platform.h"

#ifdef USE_SDCARD_SDIO

#include "at32f435_437_sdio.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/sdmmc_sdio.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "pg/sdio.h"

#define SDIO_COMMAND_FLAGS (SDIO_CMDFAIL_FLAG | SDIO_CMDTIMEOUT_FLAG | SDIO_CMDRSPCMPL_FLAG | SDIO_CMDCMPL_FLAG)
#define SDIO_DATA_ERRORS (SDIO_DTFAIL_FLAG | SDIO_DTTIMEOUT_FLAG | SDIO_TXERRU_FLAG | SDIO_RXERRO_FLAG | SDIO_SBITERR_FLAG)
#define SDIO_DATA_FLAGS (SDIO_DATA_ERRORS | SDIO_DTCMPL_FLAG | SDIO_DTBLKCMPL_FLAG)
#define SD_R1_ERRORS 0xFDFFE008U

SD_CardInfo_t SD_CardInfo;
SD_CardType_t SD_CardType;
static dmaChannelDescriptor_t *sdDma;
static uint32_t cardAddress;
static bool cardReady;
static bool multipleBlocks;
static bool transferActive;
static uint32_t transferStart;

static bool elapsed(uint32_t start, uint32_t us)
{
    return (uint32_t)clockCyclesToMicros(getCycleCounter() - start) >= us;
}

static SD_Error_t sendCommand(uint8_t cmd, uint32_t arg, sdio_reponse_type response, bool hasCrc)
{
    sdio_flag_clear(SDIO1, SDIO_COMMAND_FLAGS);
    sdio_command_struct_type request = {
        .argument = arg, .cmd_index = cmd, .rsp_type = response, .wait_type = SDIO_WAIT_FOR_NO
    };
    sdio_command_config(SDIO1, &request);
    sdio_command_state_machine_enable(SDIO1, TRUE);
    const uint32_t start = getCycleCounter();
    uint32_t status;
    do {
        status = SDIO1->sts & SDIO_COMMAND_FLAGS;
        if (elapsed(start, cardReady ? 100U : 3000U)) {
            sdio_command_state_machine_enable(SDIO1, FALSE);
            return SD_CMD_RSP_TIMEOUT;
        }
    } while (!status);

    if (status & SDIO_CMDTIMEOUT_FLAG) {
        return SD_CMD_RSP_TIMEOUT;
    }
    if (hasCrc && (status & SDIO_CMDFAIL_FLAG)) {
        return SD_CMD_CRC_FAIL;
    }
    return SD_OK;
}

static uint32_t response1(void)
{
    return sdio_response_get(SDIO1, SDIO_RSP1_INDEX);
}

static SD_Error_t commandR1(uint8_t cmd, uint32_t arg)
{
    const SD_Error_t status = sendCommand(cmd, arg, SDIO_RESPONSE_SHORT, true);
    if (status != SD_OK) {
        return status;
    }
    return sdio_command_response_get(SDIO1) == cmd && !(response1() & SD_R1_ERRORS) ? SD_OK : SD_ERROR;
}

static void setClock(uint32_t limit)
{
    const uint32_t divisor = (system_core_clock + limit - 1U) / limit;
    sdio_clock_config(SDIO1, divisor - 2U, SDIO_CLOCK_EDGE_FALLING);
}

bool SD_Initialize_LL(dma_channel_type *dma)
{
    static const ioTag_t pins[] = {
        IO_TAG(PC12), IO_TAG(PD2), IO_TAG(PC8), IO_TAG(PC9), IO_TAG(PC10), IO_TAG(PC11)
    };
    static const resourceOwner_e owners[] = {
        OWNER_SDIO_CK, OWNER_SDIO_CMD, OWNER_SDIO_D0, OWNER_SDIO_D1, OWNER_SDIO_D2, OWNER_SDIO_D3
    };
    const unsigned count = sdioConfig()->use4BitWidth ? 6 : 3;
    for (unsigned i = 0; i < count; i++) {
        const IO_t io = IOGetByTag(pins[i]);
        if (!io || IOGetOwner(io) != OWNER_FREE) {
            return false;
        }
    }
    const dmaIdentifier_e identifier = dmaGetIdentifier((dmaResource_t *)dma);
    if (!dmaAllocate(identifier, OWNER_SDCARD, 0)) {
        return false;
    }

    sdDma = dmaGetDescriptorByIdentifier(identifier);
    dmaMuxEnable(identifier, DMAMUX_DMAREQ_ID_SDIO1);
    crm_periph_clock_enable(CRM_SDIO1_PERIPH_CLOCK, TRUE);
    for (unsigned i = 0; i < count; i++) {
        const IO_t io = IOGetByTag(pins[i]);
        IOInit(io, owners[i], 0);
        IOConfigGPIOAF(io, IO_CONFIG(GPIO_MODE_MUX, GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL,
            i == 0 ? GPIO_PULL_NONE : GPIO_PULL_UP), GPIO_MUX_12);
    }
    return true;
}

static void stopData(void)
{
    sdio_data_state_machine_enable(SDIO1, FALSE);
    sdio_dma_enable(SDIO1, FALSE);
    xDMA_Cmd(sdDma->ref, FALSE);
    sdio_flag_clear(SDIO1, SDIO_DATA_FLAGS);
    transferActive = false;
}

void SD_Abort(void)
{
    if (sdDma) {
        stopData();
        sdio_power_set(SDIO1, SDIO_POWER_OFF);
    }
    cardReady = false;
}

static bool switchToHighSpeed(void)
{
    sdio_data_struct_type config = {
        .timeout = 3000U, .data_length = 64,
        .block_size = SDIO_DATA_BLOCK_SIZE_64B, .transfer_mode = SDIO_DATA_BLOCK_TRANSFER,
        .transfer_direction = SDIO_DATA_TRANSFER_TO_CONTROLLER,
    };
    union {
        uint32_t words[16];
        uint8_t bytes[64];
    } result;
    sdio_flag_clear(SDIO1, SDIO_DATA_FLAGS);
    sdio_data_config(SDIO1, &config);
    sdio_data_state_machine_enable(SDIO1, TRUE);
    bool ok = commandR1(6, 0x80FFFFF1U) == SD_OK;
    unsigned words = 0;
    const uint32_t start = getCycleCounter();
    while (ok) {
        const uint32_t status = SDIO1->sts;
        if ((status & SDIO_DATA_ERRORS) || elapsed(start, 10000U)) {
            ok = false;
        } else if (words < 16 && (status & SDIO_RXBUF_FLAG)) {
            result.words[words++] = sdio_data_read(SDIO1);
        } else if (words == 16 && (status & SDIO_DTCMPL_FLAG)) {
            break;
        }
    }
    stopData();
    return ok && words == 16 && (result.bytes[16] & 0xFU) == 1;
}

SD_Error_t SD_Init(void)
{
    if (!sdDma) {
        return SD_NOT_CONFIGURED;
    }
    cardReady = false;
    cardAddress = 0;
    stopData();
    sdio_reset(SDIO1);
    setClock(300000U);
    sdio_power_set(SDIO1, SDIO_POWER_ON);
    sdio_clock_enable(SDIO1, TRUE);
    delayMicroseconds(1000);

    if (sendCommand(0, 0, SDIO_RESPONSE_NO, false) != SD_OK) {
        return SD_CMD_RSP_TIMEOUT;
    }
    const SD_Error_t interfaceResult = sendCommand(8, 0x1AA, SDIO_RESPONSE_SHORT, true);
    if (interfaceResult != SD_CMD_RSP_TIMEOUT &&
        (interfaceResult != SD_OK || (response1() & 0xFFFU) != 0x1AAU)) {
        return SD_INVALID_VOLTRANGE;
    }
    const bool version2 = interfaceResult == SD_OK;
    const uint32_t start = getCycleCounter();
    uint32_t ocr;
    do {
        if (commandR1(55, 0) != SD_OK ||
            sendCommand(41, 0x00FF8000U | (version2 ? (1U << 30) : 0), SDIO_RESPONSE_SHORT, false) != SD_OK) {
            return SD_INVALID_VOLTRANGE;
        }
        ocr = response1();
        if (elapsed(start, 1000000)) {
            return SD_CMD_RSP_TIMEOUT;
        }
    } while (!(ocr & (1U << 31)));

    SD_CardType = ocr & (1U << 30) ? SD_HIGH_CAPACITY : (version2 ? SD_STD_CAPACITY_V2_0 : SD_STD_CAPACITY_V1_1);
    if (sendCommand(2, 0, SDIO_RESPONSE_LONG, true) != SD_OK) {
        return SD_ERROR;
    }
    const uint32_t cid0 = response1();
    const uint32_t cid1 = sdio_response_get(SDIO1, SDIO_RSP2_INDEX);
    const uint32_t cid2 = sdio_response_get(SDIO1, SDIO_RSP3_INDEX);
    const uint32_t cid3 = sdio_response_get(SDIO1, SDIO_RSP4_INDEX);
    SD_CardInfo.SD_cid.ManufacturerID = cid0 >> 24;
    SD_CardInfo.SD_cid.OEM_AppliID = cid0 >> 8;
    SD_CardInfo.SD_cid.ProdName1 = (cid0 << 24) | (cid1 >> 8);
    SD_CardInfo.SD_cid.ProdName2 = cid1;
    SD_CardInfo.SD_cid.ProdRev = cid2 >> 24;
    SD_CardInfo.SD_cid.ProdSN = (cid2 << 8) | (cid3 >> 24);
    SD_CardInfo.SD_cid.ManufactDate = (cid3 >> 8) & 0xFFFU;

    if (sendCommand(3, 0, SDIO_RESPONSE_SHORT, true) != SD_OK || (response1() & 0xE000U)) {
        return SD_ERROR;
    }
    cardAddress = response1() & 0xFFFF0000U;
    if (!cardAddress || sendCommand(9, cardAddress, SDIO_RESPONSE_LONG, true) != SD_OK) {
        return SD_ERROR;
    }

    const uint32_t csd0 = response1();
    const uint32_t csd1 = sdio_response_get(SDIO1, SDIO_RSP2_INDEX);
    const uint32_t csd2 = sdio_response_get(SDIO1, SDIO_RSP3_INDEX);
    if (csd0 >> 30 == 1) {
        SD_CardInfo.CardCapacity = (((((uint64_t)csd1 & 0x3FU) << 16) | (csd2 >> 16)) + 1U) * 1024U;
    } else if (csd0 >> 30 == 0) {
        const uint32_t size = ((csd1 & 0x3FFU) << 2) | (csd2 >> 30);
        const unsigned multiplier = (csd2 >> 15) & 7U;
        const unsigned readBlockLength = (csd1 >> 16) & 15U;
        SD_CardInfo.CardCapacity = (((uint64_t)size + 1U) << (multiplier + 2U + readBlockLength)) / 512U;
    } else {
        return SD_UNSUPPORTED_FEATURE;
    }
    if (!SD_CardInfo.CardCapacity || SD_CardInfo.CardCapacity > UINT32_MAX) {
        return SD_OUT_OF_BOUND;
    }
    SD_CardInfo.CardBlockSize = 512;
    if (commandR1(7, cardAddress) != SD_OK ||
        (SD_CardType != SD_HIGH_CAPACITY && commandR1(16, 512) != SD_OK)) {
        return SD_ERROR;
    }
    if (sdioConfig()->use4BitWidth) {
        if (commandR1(55, cardAddress) != SD_OK || commandR1(6, 2) != SD_OK) {
            return SD_ERROR;
        }
        sdio_bus_width_config(SDIO1, SDIO_BUS_WIDTH_D4);
    }
    const bool highSpeed = sdioConfig()->clockBypass && switchToHighSpeed();
    setClock(highSpeed ? 48000000U : 24000000U);
    cardReady = true;
    return SD_OK;
}

SD_Error_t SD_GetCardInfo(void)
{
    return cardReady ? SD_OK : SD_NOT_CONFIGURED;
}

bool SD_GetState(void)
{
    return cardReady && !transferActive && commandR1(13, cardAddress) == SD_OK &&
        (response1() & 0x1F00U) == 0x0900U;
}

static SD_Error_t startTransfer(uint64_t block, uint32_t *buffer, uint32_t blockSize, uint32_t count, bool write)
{
    if (!cardReady || transferActive) {
        return SD_BUSY;
    }
    if (!buffer || (uintptr_t)buffer % 4 || blockSize != 512 || !count || count > UINT16_MAX / 128U ||
        block >= SD_CardInfo.CardCapacity || count > SD_CardInfo.CardCapacity - block) {
        return SD_INVALID_PARAMETER;
    }

    const uint32_t arg = (uint32_t)(SD_CardType == SD_HIGH_CAPACITY ? block : block * 512U);
    multipleBlocks = count > 1;
    const uint8_t cmd = write ? (multipleBlocks ? 25 : 24) : (multipleBlocks ? 18 : 17);
    sdio_data_struct_type config = {
        .timeout = 2400000U, .data_length = count * 512U,
        .block_size = SDIO_DATA_BLOCK_SIZE_512B, .transfer_mode = SDIO_DATA_BLOCK_TRANSFER,
        .transfer_direction = write ? SDIO_DATA_TRANSFER_TO_CARD : SDIO_DATA_TRANSFER_TO_CONTROLLER,
    };
    dma_init_type dma = {
        .peripheral_base_addr = (uint32_t)&SDIO1->buf,
        .memory_base_addr = (uint32_t)buffer,
        .direction = write ? DMA_DIR_MEMORY_TO_PERIPHERAL : DMA_DIR_PERIPHERAL_TO_MEMORY,
        .buffer_size = count * 128U, .memory_inc_enable = TRUE,
        .peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD,
        .memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD,
        .priority = DMA_PRIORITY_HIGH,
    };
    DMA_CLEAR_FLAG(sdDma, DMA_IT_GLOB);
    dma_init((dma_channel_type *)sdDma->ref, &dma);
    sdio_flag_clear(SDIO1, SDIO_DATA_FLAGS);
    sdio_data_config(SDIO1, &config);
    sdio_dma_enable(SDIO1, TRUE);
    xDMA_Cmd(sdDma->ref, TRUE);
    if (!write) {
        sdio_data_state_machine_enable(SDIO1, TRUE);
    }
    const SD_Error_t status = commandR1(cmd, arg);
    if (status != SD_OK) {
        stopData();
        return status;
    }
    if (write) {
        sdio_data_state_machine_enable(SDIO1, TRUE);
    }
    transferStart = getCycleCounter();
    transferActive = true;
    return SD_OK;
}

static SD_Error_t checkTransfer(void)
{
    if (!transferActive) {
        return SD_ERROR;
    }
    const uint32_t status = SDIO1->sts;
    const bool failed = (status & SDIO_DATA_ERRORS) || DMA_GET_FLAG_STATUS(sdDma, DMA_IT_TEIF) || elapsed(transferStart, 1000000U);
    if (!failed && (!(status & SDIO_DTCMPL_FLAG) || !DMA_GET_FLAG_STATUS(sdDma, DMA_IT_TCIF))) {
        return SD_BUSY;
    }
    stopData();
    const SD_Error_t stop = multipleBlocks ? commandR1(12, 0) : SD_OK;
    return failed ? SD_ERROR : stop;
}

SD_Error_t SD_ReadBlocks_DMA(uint64_t block, uint32_t *buffer, uint32_t size, uint32_t count)
{
    return startTransfer(block, buffer, size, count, false);
}

SD_Error_t SD_WriteBlocks_DMA(uint64_t block, uint32_t *buffer, uint32_t size, uint32_t count)
{
    return startTransfer(block, buffer, size, count, true);
}

SD_Error_t SD_CheckRead(void)
{
    return checkTransfer();
}

SD_Error_t SD_CheckWrite(void)
{
    return checkTransfer();
}

#endif
