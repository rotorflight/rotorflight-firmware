#pragma once

#define USBD_STD_INQUIRY_LENGTH 36

typedef struct _USBD_STORAGE {
    int8_t (*Init)(uint8_t lun);
    int8_t (*GetCapacity)(uint8_t lun, uint32_t *block_num, uint32_t *block_size);
    int8_t (*IsReady)(uint8_t lun);
    int8_t (*IsWriteProtected)(uint8_t lun);
    int8_t (*Read)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
    int8_t (*Write)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
    int8_t (*GetMaxLun)(void);
    int8_t *pInquiry;
} USBD_STORAGE_cb_TypeDef;

extern USBD_STORAGE_cb_TypeDef *USBD_STORAGE_fops;
