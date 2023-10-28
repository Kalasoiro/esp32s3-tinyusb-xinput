#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tusb.h"
#include "class/hid/hid_device.h"
#include "device/usbd_pvt.h"

/********* Descriptors ***************/

const tusb_desc_device_t xinputDeviceDescriptor = {
  .bLength = sizeof(tusb_desc_device_t),
  .bDescriptorType = TUSB_DESC_DEVICE,
  .bcdUSB = 0x0200,
  .bDeviceClass = 0xFF,
  .bDeviceSubClass = 0xFF,
  .bDeviceProtocol = 0xFF,
  .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

  .idVendor = 0x045E,
  .idProduct = 0x028E,
  .bcdDevice = 0x0572,

  .iManufacturer = 0x01,
  .iProduct = 0x02,
  .iSerialNumber = 0x03,

  .bNumConfigurations = 0x01
};

const uint8_t xinputConfigurationDescriptor[] = {
  //Configuration Descriptor:
  0x09,	//bLength
  0x02,	//bDescriptorType
  0x30,0x00, 	//wTotalLength   (48 bytes)
  0x01,	//bNumInterfaces
  0x01,	//bConfigurationValue
  0x00,	//iConfiguration
  0x80,	//bmAttributes   (Bus-powered Device)
  0xFA,	//bMaxPower      (500 mA)

  //Interface Descriptor:
  0x09,	//bLength
  0x04,	//bDescriptorType
  0x00,	//bInterfaceNumber
  0x00,	//bAlternateSetting
  0x02,	//bNumEndPoints
  0xFF,	//bInterfaceClass      (Vendor specific)
  0x5D,	//bInterfaceSubClass   
  0x01,	//bInterfaceProtocol   
  0x00,	//iInterface

  //Unknown Descriptor:
  0x10,
  0x21, 
  0x10, 
  0x01, 
  0x01, 
  0x24, 
  0x81, 
  0x14, 
  0x03, 
  0x00, 
  0x03,
  0x13, 
  0x02, 
  0x00, 
  0x03, 
  0x00, 

  //Endpoint Descriptor:
  0x07,	//bLength
  0x05,	//bDescriptorType
  0x81,	//bEndpointAddress  (IN endpoint 1)
  0x03,	//bmAttributes      (Transfer: Interrupt / Synch: None / Usage: Data)
  0x20,0x00, 	//wMaxPacketSize    (1 x 32 bytes)
  0x04,	//bInterval         (4 frames)

  //Endpoint Descriptor:

  0x07,	//bLength
  0x05,	//bDescriptorType
  0x02,	//bEndpointAddress  (OUT endpoint 2)
  0x03,	//bmAttributes      (Transfer: Interrupt / Synch: None / Usage: Data)
  0x20,0x00, 	//wMaxPacketSize    (1 x 32 bytes)
  0x08,	//bInterval         (8 frames)
};

char const *string_desc_arr_xinput[] = {
  (const char[]){0x09, 0x04},   // 0: is supported language is English (0x0409)
  "GENERIC",                    // 1: Manufacturer
  "XINPUT CONTROLLER",          // 2: Product
  "1.0"                         // 3: Serials
};


/********* TinyUSB HID callbacks ***************/
const uint8_t *tud_descriptor_device_cb(void) //
{
  return (const uint8_t*)(&xinputDeviceDescriptor);
}

const uint8_t *tud_descriptor_configuration_cb(uint8_t index) //
{
  return xinputConfigurationDescriptor;
}

const uint8_t *tud_hid_descriptor_report_cb(uint8_t instance) //
{
  return (const uint8_t*)(&xinputDeviceDescriptor);
}

static uint16_t _desc_str[32];
const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) //
{
  uint8_t chr_count;

  if (index == 0) {
    memcpy(&_desc_str[1], string_desc_arr_xinput[0], 2);
    chr_count = 1;
  } else {
    // Convert ASCII string into UTF-16

    if (!(index < sizeof(string_desc_arr_xinput) / sizeof(string_desc_arr_xinput[0])))
      return NULL;

    const char *str = string_desc_arr_xinput[index];

    // Cap at max char
    chr_count = strlen(str);
    if (chr_count > 31)
      chr_count = 31;

    for (uint8_t i = 0; i < chr_count; ++i) 
    {
      _desc_str[1 + i] = str[i];
    }
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

  return _desc_str;
}


/********* Report ***************/

typedef struct __attribute__((packed, aligned(1))) {
	uint8_t rid;
	uint8_t rsize;
	uint8_t digital_buttons_1;
	uint8_t digital_buttons_2;
	uint8_t lt;
	uint8_t rt;
	int16_t l_x;
	int16_t l_y;
	int16_t r_x;
	int16_t r_y;
	uint8_t reserved_1[6];
} ReportDataXinput;
ReportDataXinput XboxButtonData;      // data to send
ReportDataXinput prevXboxButtonData;  // data sent last time


/********* Code ***************/

bool inputChanged()
{
  if(XboxButtonData.digital_buttons_1 != prevXboxButtonData.digital_buttons_1)
    return true;
  if(XboxButtonData.digital_buttons_2 != prevXboxButtonData.digital_buttons_2)
    return true;
  if(XboxButtonData.lt != prevXboxButtonData.lt)
    return true;
  if(XboxButtonData.rt != prevXboxButtonData.rt)
    return true;
  if(XboxButtonData.l_x != prevXboxButtonData.l_x)
    return true;
  if(XboxButtonData.l_y != prevXboxButtonData.l_y)
    return true;
  if(XboxButtonData.r_x != prevXboxButtonData.r_x)
    return true;
  if(XboxButtonData.r_y != prevXboxButtonData.r_y)
    return true;

  return false;
}

const uint8_t endpoint_in = 0x81;
void sendReportData() 
{
  if(endpoint_in == 0)
    return;

  if(!inputChanged())
    return;

  // Poll every 4ms
  const unsigned long interval_ms = 4;
  static unsigned long start_ms = 0;

  if (millis() - start_ms < interval_ms) 
    return;
  start_ms += interval_ms;

  // Remote wakeup
  if (tud_suspended()) 
  {
    tud_remote_wakeup();
  }

  XboxButtonData.rid = 0;
  XboxButtonData.rsize = 20;
  for(int8_t i = 0; i < 6; ++i)
    XboxButtonData.reserved_1[i] = 0;

  if (tud_ready()  && !usbd_edpt_busy(0, endpoint_in))
  {
    usbd_edpt_claim(0, endpoint_in);
    usbd_edpt_xfer(0, endpoint_in, (uint8_t*)&XboxButtonData, 20);
    usbd_edpt_release(0, endpoint_in);

    prevXboxButtonData = XboxButtonData;
  }
}

static void xinput_init() 
{
}

static void xinput_reset(uint8_t rhport) 
{
}

static uint16_t xinput_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) 
{
  //+16 is for the unknown descriptor 
  const uint16_t drv_len = sizeof(tusb_desc_interface_t) + itf_desc->bNumEndpoints*sizeof(tusb_desc_endpoint_t) + 16;
  TU_VERIFY(max_len >= drv_len, 0);

  const uint8_t* p_desc = tu_desc_next(itf_desc);
  uint8_t found_endpoints = 0;
  while (found_endpoints < itf_desc->bNumEndpoints)
  {
    const tusb_desc_endpoint_t* desc_ep = (const tusb_desc_endpoint_t*) p_desc;
    if ( TUSB_DESC_ENDPOINT == tu_desc_type(desc_ep) )
    {
      TU_ASSERT(usbd_edpt_open(rhport, desc_ep));
      found_endpoints += 1;
    }
    p_desc = tu_desc_next(p_desc);
  }

  return drv_len;
}

static bool xinput_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
    return true;
}

static bool xinput_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) 
{
  return true;
}

static const usbd_class_driver_t xinput_driver = {
#if CFG_TUSB_DEBUG >= 2
  .name = "XINPUT",
#endif
  .init             = xinput_init,
  .reset            = xinput_reset,
  .open             = xinput_open,
  .control_xfer_cb  = xinput_control_xfer_cb,
  .xfer_cb          = xinput_xfer_cb,
  .sof              = NULL
};

const usbd_class_driver_t *usbd_app_driver_get_cb(uint8_t *driver_count) 
{
  *driver_count = 1;
  return &xinput_driver;
}

void resetReportStruct()
{
  XboxButtonData.rid = 0;
  XboxButtonData.rsize = 20;
  XboxButtonData.digital_buttons_1 = 0;
  XboxButtonData.digital_buttons_2 = 0;
  XboxButtonData.lt = 0;
  XboxButtonData.rt = 0;
  XboxButtonData.l_x = 0;
  XboxButtonData.l_y = 0;
  XboxButtonData.r_x = 0;
  XboxButtonData.r_y = 0;
}

void setup() 
{
  pinMode(0, INPUT_PULLUP);

  resetReportStruct();
  prevXboxButtonData = XboxButtonData;

  tusb_init();
  delay(1000);
}

void loop() 
{
  // tinyusb task handler
  tud_task_ext(0, false);

  //inputs
  resetReportStruct();

  // spin left joystick when BOOT button pressed
  if(digitalRead(0) == LOW)
  {
    float t = float(millis()) / 1000.0f;
    int16_t x = (int16_t)(cos(t) * 32767.0f);
    int16_t y = (int16_t)(sin(t) * 32767.0f);
    XboxButtonData.l_x = x;
    XboxButtonData.l_y = y;
  }

  // send inputs
  sendReportData();
  
  delay(1);
}