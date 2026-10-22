/* CH32H41x USB mass-storage implementation adapted from the reference port. */
#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#if defined(USE_USB_MSC)
#include "build/build_config.h"
#include "common/utils.h"
#include "blackbox/blackbox.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/usb_msc.h"
#include "drivers/usb_io.h"
#include "msc/usbd_storage.h"
#include "msc/usbd_storage_emfat.h"
#include "pg/sdcard.h"
#include "usbd_core.h"
#include "usbd_msc.h"
#include "usb_ch32h41x_usbhs_reg.h"

#define MSC_IN_EP 0x81
#define MSC_OUT_EP 0x01
#define USBD_VID 0x1A86
#define USBD_PID 0xFE10
#define USBD_MAX_POWER 100
#define USB_CONFIG_SIZE (9 + MSC_DESCRIPTOR_LEN)
#define MSC_MAX_MPS 64

static const uint8_t deviceDescriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0200, 0x01)};
static const uint8_t configDescriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_SELF_POWERED, USBD_MAX_POWER),
    MSC_DESCRIPTOR_INIT(0x00, MSC_OUT_EP, MSC_IN_EP, MSC_MAX_MPS, 0x05)};
static const uint8_t deviceQualityDescriptor[] = {
    0x0a, USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00};
static const char *stringDescriptors[] = {
    (const char[]){0x09, 0x04}, "Rotorflight", "Rotorflight FC Mass Storage (FS Mode)",
    "2025123456", "Rotorflight CH32H417", "Rotorflight CH32H417"};

static const uint8_t *deviceDescriptorCallback(uint8_t speed)
{
    UNUSED(speed);
    return deviceDescriptor;
}
static const uint8_t *configDescriptorCallback(uint8_t speed)
{
    UNUSED(speed);
    return configDescriptor;
}
static const uint8_t *deviceQualityDescriptorCallback(uint8_t speed)
{
    UNUSED(speed);
    return deviceQualityDescriptor;
}
static const char *stringDescriptorCallback(uint8_t speed, uint8_t index)
{
    UNUSED(speed);
    return index <= 5 ? stringDescriptors[index] : NULL;
}

const struct usb_descriptor msc_ram_descriptor = {
    .device_descriptor_callback = deviceDescriptorCallback,
    .config_descriptor_callback = configDescriptorCallback,
    .device_quality_descriptor_callback = deviceQualityDescriptorCallback,
    .string_descriptor_callback = stringDescriptorCallback};

static void usbdEventHandler(uint8_t busid, uint8_t event)
{
    UNUSED(busid);
    UNUSED(event);
}
static struct usbd_interface intf0;

void msc_ram_init(uint8_t busid, uintptr_t regBase)
{
    USBD_STORAGE_fops->Init(0);
    usbd_desc_register(busid, &msc_ram_descriptor);
    usbd_add_interface(busid, usbd_msc_init_intf(busid, &intf0, MSC_OUT_EP, MSC_IN_EP));
    usbd_initialize(busid, regBase, usbdEventHandler);
}

static void mscUsbGpioConfig(void)
{
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOB, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
}

static void mscUsbClockConfig(void)
{
    RCC_HBPeriphClockCmd(RCC_HBPeriph_USBHS, DISABLE);
    RCC_UTMIcmd(DISABLE);
    if ((RCC->PLLCFGR & RCC_SYSPLL_SEL) != RCC_SYSPLL_USBHS)
    {
        RCC_USBHS_PLLCmd(DISABLE);
        RCC_USBHSPLLCLKConfig(RCC_USBHSPLLSource_HSE);
        RCC_USBHSPLLReferConfig(RCC_USBHSPLLRefer_25M);
        RCC_USBHSPLLClockSourceDivConfig(RCC_USBHSPLL_IN_Div1);
        RCC_USBHS_PLLCmd(ENABLE);
    }
    RCC_UTMIcmd(ENABLE);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_USBHS, ENABLE);
}

uint8_t mscStart(void)
{
    mscUsbGpioConfig();
    usbGenerateDisconnectPulse();
    IOInit(IOGetByTag(IO_TAG(PB8)), OWNER_USB, 0);
    IOInit(IOGetByTag(IO_TAG(PB9)), OWNER_USB, 0);

    switch (blackboxConfig()->device)
    {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
#ifdef USE_SDCARD_SPI
        if (sdcardConfig()->mode != SDCARD_MODE_SPI)
            return 1;
        USBD_STORAGE_fops = &USBD_MSC_MICRO_SD_SPI_fops;
#else
        return 1;
#endif
        break;
#endif
#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        USBD_STORAGE_fops = &USBD_MSC_EMFAT_fops;
        break;
#endif
    default:
        return 1;
    }

    mscUsbClockConfig();
    usb_rxsof_handler = NULL;
    msc_ram_init(0, 0);
    NVIC_SetPriority(USBHS_IRQn, NVIC_PRIO_USB);
    NVIC_EnableIRQ(USBHS_IRQn);
    NVIC_DisableIRQ(SysTick1_IRQn);
    asm("fence.i");
    NVIC_SetPriority(SysTick1_IRQn, 0);
    NVIC_EnableIRQ(SysTick1_IRQn);
    return 0;
}

static uint32_t blockSize;
void usbd_msc_get_cap(uint8_t busid, uint8_t lun, uint32_t *blockNum, uint32_t *blockSizeOut)
{
    UNUSED(busid);
    USBD_STORAGE_fops->GetCapacity(lun, blockNum, blockSizeOut);
    blockSize = *blockSizeOut;
}
int usbd_msc_sector_read(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length)
{
    UNUSED(busid);
    return USBD_STORAGE_fops->Read(lun, buffer, sector, length / blockSize);
}
int usbd_msc_sector_write(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length)
{
    UNUSED(busid);
    return USBD_STORAGE_fops->Write(lun, buffer, sector, length / blockSize);
}
#endif
