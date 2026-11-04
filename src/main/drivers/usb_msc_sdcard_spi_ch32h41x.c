#include <stdint.h>
#include "platform.h"

#if defined(USE_USB_MSC) && defined(USE_SDCARD_SPI)
#include "blackbox/blackbox.h"
#include "drivers/light_led.h"
#include "drivers/sdcard.h"
#include "drivers/usb_msc.h"
#include "msc/usbd_storage.h"
#include "pg/sdcard.h"

#define STORAGE_LUN_NBR 1

static int8_t storageInit(uint8_t lun)
{
    UNUSED(lun);
    LED0_OFF;
    sdcard_init(sdcardConfig());
    while (sdcard_poll() == 0)
        ;
    mscSetActive();
    return 0;
}
static int8_t storageGetCapacity(uint8_t lun, uint32_t *blockNum, uint32_t *blockSize)
{
    UNUSED(lun);
    *blockNum = sdcard_getMetadata()->numBlocks;
    *blockSize = 512;
    return 0;
}
static int8_t storageIsReady(uint8_t lun)
{
    UNUSED(lun);
    return sdcard_poll() ? 0 : -1;
}
static int8_t storageIsWriteProtected(uint8_t lun)
{
    UNUSED(lun);
    return 0;
}
static int8_t storageRead(uint8_t lun, uint8_t *buffer, uint32_t blockAddress, uint16_t blockLength)
{
    UNUSED(lun);
    for (int index = 0; index < blockLength; index++)
    {
        while (sdcard_readBlock(blockAddress + index, buffer + (512 * index), NULL, 0) == 0)
            ;
        while (sdcard_poll() == 0)
            ;
    }
    mscSetActive();
    return 0;
}
static int8_t storageWrite(uint8_t lun, uint8_t *buffer, uint32_t blockAddress, uint16_t blockLength)
{
    UNUSED(lun);
    for (int index = 0; index < blockLength; index++)
    {
        while (sdcard_writeBlock(blockAddress + index, buffer + (index * 512), NULL, 0) != SDCARD_OPERATION_IN_PROGRESS)
            sdcard_poll();
        while (sdcard_poll() == 0)
            ;
    }
    mscSetActive();
    return 0;
}
static int8_t storageGetMaxLun(void) { return STORAGE_LUN_NBR - 1; }

static uint8_t storageInquiryData[] = {
    0x00, 0x80, 0x02, 0x02, USBD_STD_INQUIRY_LENGTH - 5, 0x00, 0x00, 0x00,
    'W', 'C', 'H', ' ', ' ', ' ', ' ', ' ', 'P', 'r', 'o', 'd', 'u', 't', ' ', ' ',
    ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '0', '.', '0', '1'};

USBD_STORAGE_cb_TypeDef USBD_MSC_MICRO_SD_SPI_fops = {
    storageInit, storageGetCapacity, storageIsReady, storageIsWriteProtected,
    storageRead, storageWrite, storageGetMaxLun, (int8_t *)storageInquiryData};
#endif
