/**
  **************************************************************************
  * @file     cdc_class.c
  * @version  v2.0.5
  * @date     2022-02-11
  * @brief    usb cdc class type
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to 
  * download from Artery official website is the copyrighted work of Artery. 
  * Artery authorizes customers to use, copy, and distribute the BSP 
  * software and its related documentation for the purpose of design and 
  * development in conjunction with Artery microcontrollers. Use of the 
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
#include "platform.h"
#include "usbd_core.h"
#include "cdc_class.h"
#include "pg/usb.h"

#include <string.h>
#include "cdc_desc.h"

/** @addtogroup AT32F435_437_middlewares_usbd_class
  * @{
  */
  
/** @defgroup USB_cdc_class
  * @brief usb device class cdc demo
  * @{
  */  

/** @defgroup USB_cdc_class_private_functions
  * @{
  */

static usb_sts_type class_init_handler(void *udev);
static usb_sts_type class_clear_handler(void *udev);
static usb_sts_type class_setup_handler(void *udev, usb_setup_type *setup);
static usb_sts_type class_ept0_tx_handler(void *udev);
static usb_sts_type class_ept0_rx_handler(void *udev);
static usb_sts_type class_in_handler(void *udev, uint8_t ept_num);
static usb_sts_type class_out_handler(void *udev, uint8_t ept_num);
static usb_sts_type class_sof_handler(void *udev);
static usb_sts_type class_event_handler(void *udev, usbd_event_type event);

static usb_sts_type cdc_struct_init(cdc_struct_type *pcdc);
extern void usb_usart_config( linecoding_type linecoding);
static void usb_vcp_cmd_process(void *udev, uint8_t cmd, uint8_t *buff, uint16_t len);

linecoding_type linecoding = 
{
  115200,
  0,
  0,
  8
};
/** function pointers used in vTable
  * @}
  */

static void (*ctrlLineStateCb)(void *context, uint16_t ctrlLineState);
static void *ctrlLineStateCbContext;
static void (*baudRateCb)(void *context, uint32_t baud);
static void *baudRateCbContext;


#ifdef USE_USB_CDC_HID
static volatile bool hidBusy;
static uint8_t hidIdle;
static uint8_t hidReport[USBD_HID_REPORT_SIZE];

void cdc_hid_send_report(void *udev, const uint8_t *report, uint16_t size)
{
    usbd_core_type *device = udev;
    if (usbDevConfig()->type == COMPOSITE && device->conn_state == USB_CONN_STATE_CONFIGURED && !hidBusy && size == sizeof(hidReport)) {
        memcpy(hidReport, report, size);
        hidBusy = true;
        usbd_ept_send(device, USBD_HID_IN_EPT, hidReport, size);
    }
}
#endif

/* cdc data struct */
cdc_struct_type cdc_struct;

/* usb device class handler */
usbd_class_handler cdc_class_handler = 
{
  class_init_handler,
  class_clear_handler,
  class_setup_handler,
  class_ept0_tx_handler,
  class_ept0_rx_handler,
  class_in_handler,
  class_out_handler,
  class_sof_handler,
  class_event_handler,
  &cdc_struct
};
/**
  * @brief  initialize usb custom hid endpoint
  * @param  udev: to the structure of usbd_core_type
  * @retval status of usb_sts_type                            
  */
static usb_sts_type class_init_handler(void *udev)
{
  usb_sts_type status = USB_OK;
  usbd_core_type *pudev = (usbd_core_type *)udev;
  cdc_struct_type *pcdc = (cdc_struct_type *)pudev->class_handler->pdata;
  
#ifdef USE_USB_CDC_HID
  if (usbDevConfig()->type == COMPOSITE) {
    hidBusy = false;
    hidIdle = 0;
    usbd_ept_open(pudev, USBD_HID_IN_EPT, EPT_INT_TYPE, sizeof(hidReport));
  }
#endif
  /* init cdc struct */
  cdc_struct_init(pcdc);
  
  /* open in endpoint */
  usbd_ept_open(pudev, USBD_CDC_INT_EPT, EPT_INT_TYPE, USBD_CDC_CMD_MAXPACKET_SIZE);
  
  /* open in endpoint */
  usbd_ept_open(pudev, USBD_CDC_BULK_IN_EPT, EPT_BULK_TYPE, USBD_CDC_IN_MAXPACKET_SIZE);
  
  /* open out endpoint */
  usbd_ept_open(pudev, USBD_CDC_BULK_OUT_EPT, EPT_BULK_TYPE, USBD_CDC_OUT_MAXPACKET_SIZE);
  
  /* set out endpoint to receive status */
  usbd_ept_recv(pudev, USBD_CDC_BULK_OUT_EPT, pcdc->g_rx_buff, USBD_CDC_OUT_MAXPACKET_SIZE);
  


  return status;
}

/**
  * @brief  clear endpoint or other state
  * @param  udev: to the structure of usbd_core_type
  * @retval status of usb_sts_type                            
  */
static usb_sts_type class_clear_handler(void *udev)
{
  usb_sts_type status = USB_OK;
  usbd_core_type *pudev = (usbd_core_type *)udev;
  
#ifdef USE_USB_CDC_HID
  if (usbDevConfig()->type == COMPOSITE) {
    usbd_ept_close(pudev, USBD_HID_IN_EPT);
    hidBusy = false;
  }
#endif
  /* close in endpoint */
  usbd_ept_close(pudev, USBD_CDC_INT_EPT);
  
  /* close in endpoint */
  usbd_ept_close(pudev, USBD_CDC_BULK_IN_EPT);
  
  /* close out endpoint */
  usbd_ept_close(pudev, USBD_CDC_BULK_OUT_EPT);

  
  return status;
}

/**
  * @brief  usb device class setup request handler
  * @param  udev: to the structure of usbd_core_type
  * @param  setup: setup packet
  * @retval status of usb_sts_type                            
  */
static usb_sts_type class_setup_handler(void *udev, usb_setup_type *setup)
{
  usb_sts_type status = USB_OK;
  usbd_core_type *pudev = (usbd_core_type *)udev;
  cdc_struct_type *pcdc = (cdc_struct_type *)pudev->class_handler->pdata;

#ifdef USE_USB_CDC_HID
  if (usbDevConfig()->type == COMPOSITE && setup->wIndex == USBD_HID_INTERFACE) {
    const uint8_t requestType = setup->bmRequestType & USB_REQ_TYPE_RESERVED;
    if (requestType == USB_REQ_TYPE_STANDARD) {
      switch (setup->bRequest) {
      case USB_STD_REQ_GET_DESCRIPTOR: {
        uint16_t size;
        const uint8_t *descriptor = cdc_hid_descriptor(setup->wValue >> 8, &size);
        if (descriptor && setup->wLength) {
          usbd_ctrl_send(pudev, (uint8_t *)descriptor, size < setup->wLength ? size : setup->wLength);
          return USB_OK;
        }
        break;
      }
      case USB_STD_REQ_GET_INTERFACE: {
        static uint8_t alternateSetting = 0;
        if (setup->wLength == 1) {
          usbd_ctrl_send(pudev, &alternateSetting, 1);
          return USB_OK;
        }
        break;
      }
      case USB_STD_REQ_SET_INTERFACE:
        if (setup->wValue == 0 && setup->wLength == 0) {
          return USB_OK;
        }
        break;
      }
    } else if (requestType == USB_REQ_TYPE_CLASS) {
      switch (setup->bRequest) {
      case 0x01: // GET_REPORT
        if (setup->wLength) {
          usbd_ctrl_send(pudev, hidReport, sizeof(hidReport) < setup->wLength ? sizeof(hidReport) : setup->wLength);
          return USB_OK;
        }
        break;
      case 0x02: // GET_IDLE
        if (setup->wLength == 1) {
          usbd_ctrl_send(pudev, &hidIdle, 1);
          return USB_OK;
        }
        break;
      case 0x0A: // SET_IDLE
        if (setup->wLength == 0) {
          hidIdle = setup->wValue >> 8;
          return USB_OK;
        }
        break;
      }
    }
    usbd_ctrl_unsupport(pudev);
    return USB_FAIL;
  }
#endif

  switch(setup->bmRequestType & USB_REQ_TYPE_RESERVED)
  {
    /* class request */
    case USB_REQ_TYPE_CLASS:
      pcdc->g_req = 0xFF;
      pcdc->g_len = 0;
      if (setup->wIndex > 1 || setup->wLength > sizeof(pcdc->g_cmd) ||
          ((setup->bRequest == SET_LINE_CODING || setup->bRequest == GET_LINE_CODING) && setup->wLength != 7) ||
          (setup->bRequest == SET_LINE_CODING && (setup->bmRequestType & USB_REQ_DIR_DTH)) ||
          (setup->bRequest == GET_LINE_CODING && !(setup->bmRequestType & USB_REQ_DIR_DTH)) ||
          (setup->bRequest == SET_CONTROL_LINE_STATE && (setup->wLength || (setup->bmRequestType & USB_REQ_DIR_DTH)))) {
        usbd_ctrl_unsupport(pudev);
        return USB_FAIL;
      }
      if (setup->bRequest == SET_CONTROL_LINE_STATE) {
        if (ctrlLineStateCb) {
          ctrlLineStateCb(ctrlLineStateCbContext, setup->wValue);
        }
        return USB_OK;
      }
      if(setup->wLength)
      {
        if(setup->bmRequestType & USB_REQ_DIR_DTH)
        {
          usb_vcp_cmd_process(udev, setup->bRequest, pcdc->g_cmd, setup->wLength);
          usbd_ctrl_send(pudev, pcdc->g_cmd, setup->wLength);
        }
        else
        {
          pcdc->g_req = setup->bRequest;
          pcdc->g_len = setup->wLength;
          usbd_ctrl_recv(pudev, pcdc->g_cmd, pcdc->g_len);
          
        }
      }
      break;
    /* standard request */
    case USB_REQ_TYPE_STANDARD:
      switch(setup->bRequest)
      {
        case USB_STD_REQ_GET_DESCRIPTOR:
          usbd_ctrl_unsupport(pudev);
          break;
        case USB_STD_REQ_GET_INTERFACE:
          usbd_ctrl_send(pudev, (uint8_t *)&pcdc->alt_setting, 1);
          break;
        case USB_STD_REQ_SET_INTERFACE:
          pcdc->alt_setting = setup->wValue;
          break;
        default:
          break;
      }
      break;
    default:
      usbd_ctrl_unsupport(pudev);
      break;
  }
  return status;
}

/**
  * @brief  usb device endpoint 0 in status stage complete
  * @param  udev: to the structure of usbd_core_type
  * @retval status of usb_sts_type                            
  */
static usb_sts_type class_ept0_tx_handler(void *udev)
{
  usb_sts_type status = USB_OK;
  
  /* ...user code... */
  UNUSED(udev);
  
  return status;
}

/**
  * @brief  usb device endpoint 0 out status stage complete
  * @param  udev: usb device core handler type
  * @retval status of usb_sts_type                            
  */
static usb_sts_type class_ept0_rx_handler(void *udev)
{
  usb_sts_type status = USB_OK;
  usbd_core_type *pudev = (usbd_core_type *)udev;
  cdc_struct_type *pcdc = (cdc_struct_type *)pudev->class_handler->pdata;
  uint32_t recv_len = usbd_get_recv_len(pudev, 0);
  if (pcdc->g_req == SET_LINE_CODING) {
    if (recv_len == 7 && pcdc->g_len == 7) {
      usb_vcp_cmd_process(udev, pcdc->g_req, pcdc->g_cmd, recv_len);
    } else {
      usbd_ctrl_unsupport(pudev);
      status = USB_FAIL;
    }
  }
  pcdc->g_req = 0xFF;
  pcdc->g_len = 0;

  return status;
}

/**
  * @brief  usb device transmision complete handler
  * @param  udev: to the structure of usbd_core_type
  * @param  ept_num: endpoint number
  * @retval status of usb_sts_type                            
  */
static usb_sts_type class_in_handler(void *udev, uint8_t ept_num)
{
  usbd_core_type *pudev = (usbd_core_type *)udev;
  cdc_struct_type *pcdc = (cdc_struct_type *)pudev->class_handler->pdata;
  usb_sts_type status = USB_OK;
  
  /* ...user code...
    trans next packet data
  */
#ifdef USE_USB_CDC_HID
  if ((ept_num & 0x7F) == (USBD_HID_IN_EPT & 0x7F)) {
    hidBusy = false;
    return USB_OK;
  }
#endif
  usbd_flush_tx_fifo(pudev, ept_num);
  pcdc->g_tx_completed = 1;
  
  return status;
}

/**
  * @brief  usb device endpoint receive data
  * @param  udev: to the structure of usbd_core_type
  * @param  ept_num: endpoint number
  * @retval status of usb_sts_type                            
  */
static usb_sts_type class_out_handler(void *udev, uint8_t ept_num)
{
  usb_sts_type status = USB_OK;
  usbd_core_type *pudev = (usbd_core_type *)udev;
  cdc_struct_type *pcdc = (cdc_struct_type *)pudev->class_handler->pdata;
  
  /* get endpoint receive data length  */
  pcdc->g_rxlen = usbd_get_recv_len(pudev, ept_num);
  
  /*set recv flag*/
  pcdc->g_rx_completed = 1;
  
  return status;
}

/**
  * @brief  usb device sof handler
  * @param  udev: to the structure of usbd_core_type
  * @retval status of usb_sts_type                            
  */
static usb_sts_type class_sof_handler(void *udev)
{
  usb_sts_type status = USB_OK;
  
  /* ...user code... */
  UNUSED(udev);
  
  return status;
}

/**
  * @brief  usb device event handler
  * @param  udev: to the structure of usbd_core_type
  * @param  event: usb device event
  * @retval status of usb_sts_type                            
  */
static usb_sts_type class_event_handler(void *udev, usbd_event_type event)
{
  UNUSED(udev);
  usb_sts_type status = USB_OK;
  switch(event)
  {
    case USBD_RESET_EVENT:
      
      /* ...user code... */
    
      break;
    case USBD_SUSPEND_EVENT:
      
      /* ...user code... */
    
      break;
    case USBD_WAKEUP_EVENT:
      /* ...user code... */
    
      break;
    case USBD_INISOINCOM_EVENT:
      break;
    case USBD_OUTISOINCOM_EVENT:
      break;
    
    default:
      break;
  }
  return status;
}

/**
  * @brief  usb device cdc init
  * @param  pcdc: to the structure of cdc_struct
  * @retval status of usb_sts_type                             
  */
static usb_sts_type cdc_struct_init(cdc_struct_type *pcdc)
{
  pcdc->g_tx_completed = 1;
  pcdc->g_rx_completed = 0;
  pcdc->alt_setting = 0;
  pcdc->linecoding.bitrate = linecoding.bitrate;
  pcdc->linecoding.data = linecoding.data;
  pcdc->linecoding.format = linecoding.format;
  pcdc->linecoding.parity = linecoding.parity;
  return USB_OK;
}

/**
  * @brief  usb device class rx data process
  * @param  udev: to the structure of usbd_core_type
  * @param  recv_data: receive buffer
  * @retval receive data len                            
  */
uint16_t usb_vcp_get_rxdata(void *udev, uint8_t *recv_data)
{
  uint16_t i_index = 0;
  uint16_t tmp_len = 0;
  usbd_core_type *pudev = (usbd_core_type *)udev;
  cdc_struct_type *pcdc = (cdc_struct_type *)pudev->class_handler->pdata;
  
  if(pcdc->g_rx_completed == 0)
  {
    return 0;
  }
  pcdc->g_rx_completed = 0;
  tmp_len = pcdc->g_rxlen;
  for(i_index = 0; i_index < pcdc->g_rxlen; i_index ++)
  {
    recv_data[i_index] = pcdc->g_rx_buff[i_index];
  }
  
  usbd_ept_recv(pudev, USBD_CDC_BULK_OUT_EPT, pcdc->g_rx_buff, USBD_CDC_OUT_MAXPACKET_SIZE);
  
  return tmp_len;
}

/**
  * @brief  usb device class send data
  * @param  udev: to the structure of usbd_core_type
  * @param  send_data: send data buffer
  * @param  len: send length
  * @retval error status                            
  */
error_status usb_vcp_send_data(void *udev, uint8_t *send_data, uint16_t len)
{
  error_status status = SUCCESS;
  usbd_core_type *pudev = (usbd_core_type *)udev;
  cdc_struct_type *pcdc = (cdc_struct_type *)pudev->class_handler->pdata;
  static uint8_t txBuffer[USBD_CDC_TX_BUFFER_SIZE];
  if(pcdc->g_tx_completed && len <= sizeof(txBuffer) && (send_data || len == 0))
  {
    if (len) {
      memcpy(txBuffer, send_data, len);
    }
    pcdc->g_tx_completed = 0;
    usbd_ept_send(pudev, USBD_CDC_BULK_IN_EPT, txBuffer, len);
  }
  else
  {
    status = ERROR;
  }
  return status;
}

/**
  * @brief  usb device function
  * @param  udev: to the structure of usbd_core_type
  * @param  cmd: request number
  * @param  buff: request buffer
  * @param  len: buffer length
  * @retval none                            
  */
static void usb_vcp_cmd_process(void *udev, uint8_t cmd, uint8_t *buff, uint16_t len)
{
  if ((cmd == SET_LINE_CODING || cmd == GET_LINE_CODING) && len != 7) {
    return;
  }
  usbd_core_type *pudev = (usbd_core_type *)udev;
  cdc_struct_type *pcdc = (cdc_struct_type *)pudev->class_handler->pdata;
  switch(cmd)
  {
    case SET_LINE_CODING:
      pcdc->linecoding.bitrate = ((uint32_t)buff[0] | ((uint32_t)buff[1] << 8) | ((uint32_t)buff[2] << 16) | ((uint32_t)buff[3] << 24));
      pcdc->linecoding.format = buff[4];
      pcdc->linecoding.parity = buff[5];
      pcdc->linecoding.data = buff[6];
#ifdef USB_VIRTUAL_COMPORT
      /* set hardware usart */
      usb_usart_config(pcdc->linecoding);
#endif
      //call set line coding change callback
      if (baudRateCb) {
          baudRateCb(baudRateCbContext,pcdc->linecoding.bitrate);
         }

      break;
    
    case GET_LINE_CODING:
      buff[0] = (uint8_t)pcdc->linecoding.bitrate;
      buff[1] = (uint8_t)(pcdc->linecoding.bitrate >> 8);
      buff[2] = (uint8_t)(pcdc->linecoding.bitrate >> 16);
      buff[3] = (uint8_t)(pcdc->linecoding.bitrate >> 24);
      buff[4] = (uint8_t)(pcdc->linecoding.format);
      buff[5] = (uint8_t)(pcdc->linecoding.parity);
      buff[6] = (uint8_t)(pcdc->linecoding.data);
      break;
    
    default:
      break;
  }
}

/**
  * @}
  */ 

/*******************************************************************************
 * Function Name  : CDC_SetBaudRateCb
 * Description    : Set a callback to call when baud rate changes
 * Input          : callback function and context.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void CDC_SetBaudRateCb(void (*cb)(void *context, uint32_t baud), void *context)
{
    baudRateCbContext = context;
    baudRateCb = cb;
}

/*******************************************************************************
 * Function Name  : CDC_SetCtrlLineStateCb
 * Description    : Set a callback to call when control line state changes
 * Input          : callback function and context.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void CDC_SetCtrlLineStateCb(void (*cb)(void *context, uint16_t ctrlLineState), void *context)
{
    ctrlLineStateCbContext = context;
    ctrlLineStateCb = cb;
}
/**
  * @}
  */

/**
  * @}
  */

