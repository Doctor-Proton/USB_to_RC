#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "hardware/uart.h"
#include "bsp/board.h"
#include "tusb.h"
#include "usb_control_decode.h"
#include "FreeRTOS.h"
#include "task.h"
#include "output.h"
#include "usb_task.h"

// English
#define LANGUAGE_ID 0x0409
#define BUF_COUNT   4


tusb_desc_device_t desc_device;

uint8_t buf_pool[BUF_COUNT][64];
uint8_t buf_owner[BUF_COUNT] = { 0 }; // device address that owns buffer

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

static void print_utf16(uint16_t *temp_buf, size_t buf_len, char *global_buf,int global_buf_len);
void print_device_descriptor(tuh_xfer_t* xfer);
void parse_config_descriptor(uint8_t dev_addr, tusb_desc_configuration_t const* desc_cfg);

uint8_t* get_xid_buf(uint8_t daddr);
void free_xid_buf(uint8_t daddr);

uint8_t XID_active=false;
uint32_t XID_printf_interval=0;


tuh_xfer_t xid_endp_xfer={0};
uint32_t xid_endp_interval=10;

uint8_t mfg_string[128]; //this is probably not the most efficient way to do this, may need to implement a more complicated asyncronous mechanism
uint8_t prod_string[128]; //do we want to worry about unicode?
uint8_t serial_string[128];


void usb_host_task(void *p)
{
  tusb_init();
  init_usb_control_decode();
  uint32_t last_endp_time=0;
  while (1)
  {
    tuh_task();
    #if CFG_TUH_HID
    hid_app_task();
    #endif
    if(XID_active && (xTaskGetTickCount()-last_endp_time)>=xid_endp_interval)
    {
      last_endp_time=xTaskGetTickCount();
      tuh_edpt_xfer(&xid_endp_xfer);
    }
    vTaskDelay(1);
  }
}

/*------------- TinyUSB Callbacks -------------*/

// Invoked when device is mounted (configured)
void tuh_mount_cb (uint8_t daddr)
{
  printf("Device attached, address = %d\r\n", daddr);

  // Get Device Descriptor
  // TODO: invoking control transfer now has issue with mounting hub with multiple devices attached, fix later
  tuh_descriptor_get_device(daddr, &desc_device, 18, print_device_descriptor, 0);
}

/// Invoked when device is unmounted (bus reset/unplugged)
void tuh_umount_cb(uint8_t daddr)
{
  printf("Device removed, address = %d\r\n", daddr);
  memset(&desc_device,0,sizeof(desc_device));
  XID_active=0;
  free_xid_buf(daddr);
  signal_device_gone();
}

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+

void print_device_descriptor(tuh_xfer_t* xfer)
{
  if ( XFER_RESULT_SUCCESS != xfer->result )
  {
    printf("Failed to get device descriptor\r\n");
    return;
  }

  uint8_t const daddr = xfer->daddr;

  printf("Device %u: ID %04x:%04x\r\n", daddr, desc_device.idVendor, desc_device.idProduct);
  printf("Device Descriptor:\r\n");
  printf("  bLength             %u\r\n"     , desc_device.bLength);
  printf("  bDescriptorType     %u\r\n"     , desc_device.bDescriptorType);
  printf("  bcdUSB              %04x\r\n"   , desc_device.bcdUSB);
  printf("  bDeviceClass        %u\r\n"     , desc_device.bDeviceClass);
  printf("  bDeviceSubClass     %u\r\n"     , desc_device.bDeviceSubClass);
  printf("  bDeviceProtocol     %u\r\n"     , desc_device.bDeviceProtocol);
  printf("  bMaxPacketSize0     %u\r\n"     , desc_device.bMaxPacketSize0);
  printf("  idVendor            0x%04x\r\n" , desc_device.idVendor);
  printf("  idProduct           0x%04x\r\n" , desc_device.idProduct);
  printf("  bcdDevice           %04x\r\n"   , desc_device.bcdDevice);

  // Get String descriptor using Sync API
  uint16_t temp_buf[128];

  printf("  iManufacturer       %u     "     , desc_device.iManufacturer);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_manufacturer_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf)) )
  {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf),mfg_string,sizeof(mfg_string));
  }
  printf("\r\n");

  printf("  iProduct            %u     "     , desc_device.iProduct);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_product_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf)))
  {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf),prod_string,sizeof(prod_string));
  }
  printf("\r\n");

  printf("  iSerialNumber       %u     "     , desc_device.iSerialNumber);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_serial_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf)))
  {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf),serial_string,sizeof(serial_string));
  }
  printf("\r\n");

  printf("  bNumConfigurations  %u\r\n"     , desc_device.bNumConfigurations);

  // Get configuration descriptor with sync API
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_configuration_sync(daddr, 0, temp_buf, sizeof(temp_buf)))
  {
    parse_config_descriptor(daddr, (tusb_desc_configuration_t*) temp_buf);
  }
}

int print_device_descriptor_ssi(char *buffer, int Len) //this should maybe be mutex, but only reads desc_device and is only for display
{
  int printed_len=0;
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"VID:PID %04x:%04x<br>", desc_device.idVendor, desc_device.idProduct);
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"Device Descriptor:<br>");
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"  bLength             %u<br>"     , desc_device.bLength);
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"  bDescriptorType     %u<br>"     , desc_device.bDescriptorType);
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"  bcdUSB              %04x<br>"   , desc_device.bcdUSB);
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"  bDeviceClass        %u<br>"     , desc_device.bDeviceClass);
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"  bDeviceSubClass     %u<br>"     , desc_device.bDeviceSubClass);
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"  bDeviceProtocol     %u<br>"     , desc_device.bDeviceProtocol);
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"  bMaxPacketSize0     %u<br>"     , desc_device.bMaxPacketSize0);
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"  idVendor            0x%04x<br>" , desc_device.idVendor);
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"  idProduct           0x%04x<br>" , desc_device.idProduct);
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"  bcdDevice           %04x<br>"   , desc_device.bcdDevice);

  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"  iManufacturer       %u     "     , desc_device.iManufacturer);
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,mfg_string);
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"<br>");

  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"  iProduct            %u     "     , desc_device.iProduct);
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,prod_string);
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"<br>");

  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"  iSerialNumber       %u     "     , desc_device.iSerialNumber);
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,serial_string);
  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"<br>");

  printed_len+=snprintf(&buffer[printed_len],Len-printed_len,"  bNumConfigurations  %u<br>"     , desc_device.bNumConfigurations);
  return printed_len;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

// count total length of an interface
uint16_t count_interface_total_len(tusb_desc_interface_t const* desc_itf, uint8_t itf_count, uint16_t max_len);

void open_xid_interface(uint8_t daddr, tusb_desc_interface_t const *desc_itf, uint16_t max_len);

// simple configuration parser to open and listen to HID Endpoint IN
void parse_config_descriptor(uint8_t dev_addr, tusb_desc_configuration_t const* desc_cfg)
{
  uint8_t const* desc_end = ((uint8_t const*) desc_cfg) + tu_le16toh(desc_cfg->wTotalLength);
  uint8_t const* p_desc   = tu_desc_next(desc_cfg);

  // parse each interfaces
  while( p_desc < desc_end )
  {
    uint8_t assoc_itf_count = 1;

    // Class will always starts with Interface Association (if any) and then Interface descriptor
    if ( TUSB_DESC_INTERFACE_ASSOCIATION == tu_desc_type(p_desc) )
    {
      tusb_desc_interface_assoc_t const * desc_iad = (tusb_desc_interface_assoc_t const *) p_desc;
      assoc_itf_count = desc_iad->bInterfaceCount;

      p_desc = tu_desc_next(p_desc); // next to Interface
    }

    // must be interface from now
    if( TUSB_DESC_INTERFACE != tu_desc_type(p_desc) ) return;
    tusb_desc_interface_t const* desc_itf = (tusb_desc_interface_t const*) p_desc;

    uint16_t const drv_len = count_interface_total_len(desc_itf, assoc_itf_count, (uint16_t) (desc_end-p_desc));

    // probably corrupted descriptor
    if(drv_len < sizeof(tusb_desc_interface_t)) return;

    // only open and listen to HID endpoint IN
    if (desc_itf->bInterfaceClass == 0xFF && desc_itf->bInterfaceSubClass==0x5D && desc_itf->bInterfaceProtocol==0x01)
    {
      //open_hid_interface(dev_addr, desc_itf, drv_len);
      open_xid_interface(dev_addr, desc_itf, drv_len);
    }

    // next Interface or IAD descriptor
    p_desc += drv_len;
  }
}

uint16_t count_interface_total_len(tusb_desc_interface_t const* desc_itf, uint8_t itf_count, uint16_t max_len)
{
  uint8_t const* p_desc = (uint8_t const*) desc_itf;
  uint16_t len = 0;

  while (itf_count--)
  {
    // Next on interface desc
    len += tu_desc_len(desc_itf);
    p_desc = tu_desc_next(p_desc);

    while (len < max_len)
    {
      // return on IAD regardless of itf count
      if ( tu_desc_type(p_desc) == TUSB_DESC_INTERFACE_ASSOCIATION ) return len;

      if ( (tu_desc_type(p_desc) == TUSB_DESC_INTERFACE) &&
           ((tusb_desc_interface_t const*) p_desc)->bAlternateSetting == 0 )
      {
        break;
      }

      len += tu_desc_len(p_desc);
      p_desc = tu_desc_next(p_desc);
    }
  }

  return len;
}


//--------------------------------------------------------------------+
// XID Interface
//--------------------------------------------------------------------+

void xid_report_received(tuh_xfer_t* xfer);
void xid_endp_callback(tuh_xfer_t* xfer);

void open_xid_interface(uint8_t daddr, tusb_desc_interface_t const *desc_itf, uint16_t max_len)
{
  // len = interface + hid + n*endpoints
  uint16_t const drv_len = (uint16_t) (sizeof(tusb_desc_interface_t) + sizeof(tusb_hid_descriptor_hid_t) +
                                       desc_itf->bNumEndpoints * sizeof(tusb_desc_endpoint_t));

  // corrupted descriptor
  if (max_len < drv_len) return;

    uint8_t* buf = get_xid_buf(daddr);
      if (!buf) return; // out of memory

      const tusb_control_request_t xid_req =
      {
        .bmRequestType=0xc1,
        .bRequest=1,
        .wValue=0x0100,
        .wIndex=0,
        .wLength=20,
        
      };

      tuh_xfer_t xfer =
      {
        .daddr       = daddr,
        .ep_addr     = 0x00,
        .setup      = &xid_req,
        .buffer      = buf,
        .complete_cb = xid_report_received,
        .user_data   = (uintptr_t) buf, // since buffer is not available in callback, use user data to store the buffer
      };

      // submit transfer for this EP
      tuh_control_xfer(&xfer);

      printf("Request XID descriptor\r\n");

  uint8_t const *p_desc = (uint8_t const *) desc_itf;

  // HID descriptor
  p_desc = tu_desc_next(p_desc);
  tusb_hid_descriptor_hid_t const *desc_hid = (tusb_hid_descriptor_hid_t const *) p_desc;
  if(HID_DESC_TYPE_HID != desc_hid->bDescriptorType) return;

  // Endpoint descriptor
  p_desc = tu_desc_next(p_desc);
  tusb_desc_endpoint_t const * desc_ep = (tusb_desc_endpoint_t const *) p_desc;

  for(int i = 0; i < desc_itf->bNumEndpoints; i++)
  {
    if (TUSB_DESC_ENDPOINT != desc_ep->bDescriptorType) return;

    if(tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN)
    {
      // skip if failed to open endpoint
      if ( ! tuh_edpt_open(daddr, desc_ep) ) return;

      uint8_t* buf = get_xid_buf(daddr);
      if (!buf) return; // out of memory



        xid_endp_xfer.daddr       = daddr;
        xid_endp_xfer.ep_addr     = desc_ep->bEndpointAddress;
        xid_endp_xfer.buflen      = desc_ep->wMaxPacketSize;
        xid_endp_xfer.buffer      = buf;
        xid_endp_xfer.complete_cb = xid_endp_callback;
        xid_endp_xfer.user_data   = (uintptr_t) buf; // since buffer is not available in callback, use user data to store the buffer
        xid_endp_interval=desc_ep->bInterval;

      // submit transfer for this EP
      //tuh_edpt_xfer(&xfer);

      printf("found endpoint [dev %u: ep %02x]\r\n", daddr, desc_ep->bEndpointAddress);
    }

    p_desc = tu_desc_next(p_desc);
    desc_ep = (tusb_desc_endpoint_t const *) p_desc;
  }
}

void xid_report_received(tuh_xfer_t* xfer)
{
  XID_active=true;
  // Note: not all field in xfer is available for use (i.e filled by tinyusb stack) in callback to save sram
  // For instance, xfer->buffer is NULL. We have used user_data to store buffer when submitted callback
  uint8_t* buf = (uint8_t*) xfer->user_data;

  if (xfer->result == XFER_RESULT_SUCCESS)
  {
    printf("[dev %u: ep %02x] XID Report:", xfer->daddr, xfer->ep_addr);
    for(uint32_t i=0; i<xfer->actual_len; i++)
    {
      if (i%16 == 0) printf("\r\n  ");
      printf("%02X ", buf[i]);
    }
    printf("\r\n");
  }
  free_xid_buf(buf);
  load_XID_report();
  uint16_t VID,PID;
  tuh_vid_pid_get(xfer->daddr,&VID,&PID);
  signal_new_device(VID,PID);
}

void xid_endp_callback(tuh_xfer_t* xfer)
{
  // Note: not all field in xfer is available for use (i.e filled by tinyusb stack) in callback to save sram
  // For instance, xfer->buffer is NULL. We have used user_data to store buffer when submitted callback
  uint8_t* buf = (uint8_t*) xfer->user_data;

  if (xfer->result == XFER_RESULT_SUCCESS && (xTaskGetTickCount()-XID_printf_interval)>500)
  {
    XID_printf_interval=xTaskGetTickCount();
    printf("[dev %u: ep %02x] XID Report:", xfer->daddr, xfer->ep_addr);
    for(uint32_t i=0; i<xfer->actual_len; i++)
    {
      if (i%16 == 0) printf("\r\n  ");
      printf("%02X ", buf[i]);
    }
    printf("\r\n");
  }

  xid_endp_xfer.buffer = buf;
  process_usb_packet(buf,xfer->actual_len);
}

//--------------------------------------------------------------------+
// Buffer helper
//--------------------------------------------------------------------+

// get an buffer from pool
uint8_t* get_xid_buf(uint8_t daddr)
{
  for(size_t i=0; i<BUF_COUNT; i++)
  {
    if (buf_owner[i] == 0)
    {
      buf_owner[i] = daddr;
      return buf_pool[i];
    }
  }

  // out of memory, increase BUF_COUNT
  return NULL;
}

// free all buffer owned by device
void free_xid_buf(uint8_t daddr)
{
  for(size_t i=0; i<BUF_COUNT; i++)
  {
    if (buf_owner[i] == daddr) buf_owner[i] = 0;
  }
}
//--------------------------------------------------------------------+
// String Descriptor Helper
//--------------------------------------------------------------------+

static void _convert_utf16le_to_utf8(const uint16_t *utf16, size_t utf16_len, uint8_t *utf8, size_t utf8_len) {
    // TODO: Check for runover.
    (void)utf8_len;
    // Get the UTF-16 length out of the data itself.

    for (size_t i = 0; i < utf16_len; i++) {
        uint16_t chr = utf16[i];
        if (chr < 0x80) {
            *utf8++ = chr & 0xffu;
        } else if (chr < 0x800) {
            *utf8++ = (uint8_t)(0xC0 | (chr >> 6 & 0x1F));
            *utf8++ = (uint8_t)(0x80 | (chr >> 0 & 0x3F));
        } else {
            // TODO: Verify surrogate.
            *utf8++ = (uint8_t)(0xE0 | (chr >> 12 & 0x0F));
            *utf8++ = (uint8_t)(0x80 | (chr >> 6 & 0x3F));
            *utf8++ = (uint8_t)(0x80 | (chr >> 0 & 0x3F));
        }
        // TODO: Handle UTF-16 code points that take two entries.
    }
}

// Count how many bytes a utf-16-le encoded string will take in utf-8.
static int _count_utf8_bytes(const uint16_t *buf, size_t len) {
    size_t total_bytes = 0;
    for (size_t i = 0; i < len; i++) {
        uint16_t chr = buf[i];
        if (chr < 0x80) {
            total_bytes += 1;
        } else if (chr < 0x800) {
            total_bytes += 2;
        } else {
            total_bytes += 3;
        }
        // TODO: Handle UTF-16 code points that take two entries.
    }
    return (int) total_bytes;
}

static void print_utf16(uint16_t *temp_buf, size_t buf_len, char *global_buf,int global_buf_len) {
    size_t utf16_len = ((temp_buf[0] & 0xff) - 2) / sizeof(uint16_t);
    size_t utf8_len = (size_t) _count_utf8_bytes(temp_buf + 1, utf16_len);
    _convert_utf16le_to_utf8(temp_buf + 1, utf16_len, (uint8_t *) temp_buf, sizeof(uint16_t) * buf_len);
    ((uint8_t*) temp_buf)[utf8_len] = '\0';

    printf((char*)temp_buf);
    if(global_buf!=0)
      {
        strncpy(global_buf,temp_buf,global_buf_len);
      }
}

