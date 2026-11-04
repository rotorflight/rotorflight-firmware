/* Based on the ATBetaflight AT32 MSC storage adapter. */
#include "platform.h"
#include <string.h>
#include "at32_msc_diskio.h"
#include "common/utils.h"
#include "drivers/sdcard.h"
#include "drivers/system.h"
#include "drivers/usb_msc.h"
#include "msc_bot_scsi.h"

#define BLOCK_SIZE 512
#define SD_TIMEOUT_US 1000000

static bool sdSelected;
static bool sdFailed;
static bool transferDone;
static bool transferOk;
static uint8_t sdBlock[BLOCK_SIZE] __attribute__((aligned(4)));

static const uint8_t scsi_inquiry[MSC_SUPPORT_MAX_LUN][SCSI_INQUIRY_DATA_LENGTH] =
{
  /* lun = 0 */
  {
    0x00,         /* peripheral device type (direct-access device) */
    0x80,         /* removable media bit */
    0x00,         /* ansi version, ecma version, iso version */
    0x01,         /* respond data format */
    SCSI_INQUIRY_DATA_LENGTH - 5, /* additional length */
    0x00, 0x00, 0x00, /* reserved */
	'R', 'O', 'T', 'O', 'R', 'F', 'L', 'T', // Manufacturer : 8 bytes
	'O', 'n', 'b', 'o', 'a', 'r', 'd', ' ', // Product      : 16 Bytes
	'F', 'l', 'a', 's', 'h', ' ', ' ', ' ', //
	' ', ' ', ' ' ,' ',                     // Version      : 4 Bytes
  }
};

void msc_disk_select_sd(bool selected)
{
    sdSelected = selected;
}

bool msc_disk_is_readonly(void)
{
    return !sdSelected;
}

static bool timedOut(uint32_t start)
{
    // Use the cycle counter because this code can run in a USB interrupt.
    return clockCyclesToMicros(getCycleCounter() - start) >= SD_TIMEOUT_US;
}

usb_sts_type msc_disk_capacity(uint8_t lun, uint32_t *blocks, uint32_t *blockSize)
{
    if (lun != 0) {
        return USB_FAIL;
    }
    if (sdSelected) {
        const uint32_t start = getCycleCounter();
        while (!sdFailed && sdcard_isInserted() && !sdcard_isInitialized() && !timedOut(start)) {
            sdcard_poll();
        }
        if (sdFailed || !sdcard_isInserted() || !sdcard_isInitialized()) {
            return USB_FAIL;
        }
    }
    *blocks = sdSelected ? sdcard_getMetadata()->numBlocks : emfat.disk_sectors;
    *blockSize = BLOCK_SIZE;
    return *blocks ? USB_OK : USB_FAIL;
}

static void onTransferDone(sdcardBlockOperation_e operation, uint32_t block, uint8_t *buffer, uint32_t data)
{
    UNUSED(operation);
    UNUSED(block);
    UNUSED(data);
    transferOk = buffer != NULL;
    transferDone = true;
}

static bool transferSdBlock(uint32_t block, bool write)
{
    const uint32_t start = getCycleCounter();
    transferDone = false;
    if (write) {
        const sdcardOperationStatus_e result = sdcard_writeBlock(block, sdBlock, onTransferDone, 0);
        if (result == SDCARD_OPERATION_SUCCESS) {
            return true;
        }
        if (result != SDCARD_OPERATION_IN_PROGRESS) {
            return false;
        }
    } else if (!sdcard_readBlock(block, sdBlock, onTransferDone, 0)) {
        return false;
    }
    while (sdcard_isInserted() && !timedOut(start)) {
        const bool ready = sdcard_poll();
        if (transferDone && (ready || !transferOk)) {
            return transferOk;
        }
    }
    // DMA may still own sdBlock. Keep it isolated until reboot.
    sdFailed = true;
    return false;
}

static bool validRange(uint8_t lun, uint32_t block, uint32_t len)
{
    uint32_t blocks, blockSize;
    return msc_disk_capacity(lun, &blocks, &blockSize) == USB_OK && len % blockSize == 0 &&
        block <= blocks && len / blockSize <= blocks - block;
}

usb_sts_type msc_disk_read(uint8_t lun, uint32_t block, uint8_t *buffer, uint32_t len)
{
    if (!validRange(lun, block, len)) {
        return USB_FAIL;
    }
    mscSetActive();
    if (!sdSelected) {
        emfat_read(&emfat, buffer, block, len / BLOCK_SIZE);
        return USB_OK;
    }
    for (uint32_t offset = 0; offset < len; offset += BLOCK_SIZE) {
        if (!transferSdBlock(block++, false)) {
            return USB_FAIL;
        }
        memcpy(buffer + offset, sdBlock, BLOCK_SIZE);
    }
    return USB_OK;
}

usb_sts_type msc_disk_write(uint8_t lun, uint32_t block, uint8_t *buffer, uint32_t len)
{
    if (!sdSelected || !validRange(lun, block, len)) {
        return USB_FAIL;
    }
    mscSetActive();
    for (uint32_t offset = 0; offset < len; offset += BLOCK_SIZE) {
        memcpy(sdBlock, buffer + offset, BLOCK_SIZE);
        if (!transferSdBlock(block++, true)) {
            return USB_FAIL;
        }
    }
    return USB_OK;
}

uint8_t *get_inquiry(uint8_t lun)
{
    return lun == 0 ? (uint8_t *)scsi_inquiry[0] : NULL;
}
