/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "tusb.h"
#include "usb_descriptors.h"



#if OPT_CMSIS_DAPV1
    static uint8_t const desc_hid_report[] =
    {
        TUD_HID_REPORT_DESC_GENERIC_INOUT(CFG_TUD_HID_EP_BUFSIZE)
    };

    uint8_t const * tud_hid_descriptor_report_cb(uint8_t itf)
    {
        (void)itf;
        return desc_hid_report;
    }
#endif



size_t board_get_unique_id(uint8_t id[], size_t max_len) {
  (void) max_len;
  volatile uint32_t* ch32_uuid = ((volatile uint32_t*) 0x1FFFF7E8UL);
  uint32_t* serial_32 = (uint32_t*) (uintptr_t) id;
  serial_32[0] = ch32_uuid[0];
  serial_32[1] = ch32_uuid[1];
  serial_32[2] = ch32_uuid[2];

  return 12;
}

// Get USB Serial number string from unique ID if available. Return number of character.
// Input is string descriptor from index 1 (index 0 is type + len)

static uint8_t uid[16];
static inline size_t board_usb_get_serial(uint16_t desc_str1[], size_t max_chars) {
   TU_ATTR_ALIGNED(4);
  size_t uid_len;

  // TODO work with make, but not working with esp32s3 cmake
  if ( board_get_unique_id ) {
    uid_len = board_get_unique_id(uid, sizeof(uid));
  }else {
    // fixed serial string is 01234567889ABCDEF
    uint32_t* uid32 = (uint32_t*) (uintptr_t) uid;
    uid32[0] = 0x67452301;
    uid32[1] = 0xEFCDAB89;
    uid_len = 8;
  }

  if ( uid_len > max_chars / 2 ) uid_len = max_chars / 2;
  for ( size_t i = 0; i < uid_len; i++ ) {
    for ( size_t j = 0; j < 2; j++ ) {
      const char nibble_to_hex[16] = {
          '0', '1', '2', '3', '4', '5', '6', '7',
          '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
      };
      uint8_t const nibble = (uid[i] >> (j * 4)) & 0xf;
      desc_str1[i * 2 + (1 - j)] = nibble_to_hex[nibble]; // UTF-16-LE
    }
  }

  return 2 * uid_len;
}



/* A combination of interfaces must have a unique product id, since PC will save device driver after the first plug.
 * Same VID/PID with different interface e.g MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]       MIDI | HID | MSC | CDC          [LSB]
 */
#define _PID_MAP(itf, n)  ( (CFG_TUD_##itf) << (n) )
#define USB_PID           (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
                           _PID_MAP(MIDI, 3) | _PID_MAP(VENDOR, 4) )

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0210, // at least 2.1 or 3.x for BOS & webUSB

    // Use Interface Association Descriptor (IAD) for CDC
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0x2E8A,
    .idProduct          = 0x000c,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}



//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

enum
{
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_UDF,
  STRID_INTERFACE_DAP1,
  STRID_INTERFACE_DAP2,
  STRID_INTERFACE_MSC,
  STRID_INTERFACE_CDC_UART,
  STRID_INTERFACE_CDC_I2C,
  STRID_INTERFACE_CDC_SPI,
};


//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

enum
{
    ITF_NUM_VENDOR,
  #if CFG_TUD_CDC >2
    ITF_NUM_CDC ,
    ITF_NUM_CDC_DATA,

    ITF_NUM_CDC_2 ,
    ITF_NUM_CDC_DATA_2,
    
    ITF_NUM_CDC_3 ,
    ITF_NUM_CDC_DATA_3,
  #elif CFG_TUD_CDC >1
    ITF_NUM_CDC ,
    ITF_NUM_CDC_DATA,

    ITF_NUM_CDC_2 ,
    ITF_NUM_CDC_DATA_2,
  #else
    ITF_NUM_CDC,
    ITF_NUM_CDC_DATA,
    #if CFG_TUD_DFU
      ITF_NUM_DFU_MODE,
    #endif
  #endif
    ITF_NUM_TOTAL
};


// don't know if consecutive numbering is required.  Let's do it anyway
enum
{
    DUMMY_CNT = 0,
    PROBE_VENDOR_OUT_EP_CNT,
    PROBE_VENDOR_IN_EP_CNT,
#if CFG_TUD_CDC >2
    CDC_UART_NOTIFICATION_EP_CNT,
    CDC_UART_DATA_EP_CNT,

    CDC_UART_NOTIFICATION_EP_CNT_2,
    CDC_UART_DATA_EP_CNT_2,
    
    CDC_UART_NOTIFICATION_EP_CNT_3,
    CDC_UART_DATA_EP_CNT_3,
    
  #elif CFG_TUD_CDC >1
    CDC_UART_NOTIFICATION_EP_CNT,
    CDC_UART_DATA_EP_CNT,
    
    CDC_UART_NOTIFICATION_EP_CNT_2,
    CDC_UART_DATA_EP_CNT_2,
  #else
    CDC_UART_NOTIFICATION_EP_CNT,
    CDC_UART_DATA_EP_CNT,
  #endif
};


#define ALT_COUNT   1
#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + CFG_TUD_CDC*TUD_CDC_DESC_LEN \
                              + CFG_TUD_HID*TUD_HID_INOUT_DESC_LEN \
                              + CFG_TUD_VENDOR*TUD_VENDOR_DESC_LEN \
                              + CFG_TUD_DFU *TUD_DFU_DESC_LEN(ALT_COUNT))


#define EPNUM_VENDOR_OUT  (PROBE_VENDOR_OUT_EP_CNT+ 0x00) //7
#define EPNUM_VENDOR_IN   (PROBE_VENDOR_IN_EP_CNT + 0x80) //0x81

#if CFG_TUD_CDC >2
  #define EPNUM_CDC3_NOTIF   (CDC_UART_NOTIFICATION_EP_CNT_3+0x80)//   0x85
  #define EPNUM_CDC3_OUT     (CDC_UART_DATA_EP_CNT_3+ 0x00)       // 0x06
  #define EPNUM_CDC3_IN      (CDC_UART_DATA_EP_CNT_3+ 0x80)       //0x87
#endif

#if CFG_TUD_CDC >1
  #define EPNUM_CDC2_NOTIF   (CDC_UART_NOTIFICATION_EP_CNT_2+0x80)//0x83
  #define EPNUM_CDC2_OUT     (CDC_UART_DATA_EP_CNT_2+ 0x00)       //0x04
  #define EPNUM_CDC2_IN      (CDC_UART_DATA_EP_CNT_2+ 0x80)       //0x84
#endif

#if CFG_TUD_CDC >0
  #define EPNUM_CDC_NOTIF  (CDC_UART_NOTIFICATION_EP_CNT+0x80)//0x82
  #define EPNUM_CDC_OUT    (CDC_UART_DATA_EP_CNT+ 0x00)       //0x03
  #define EPNUM_CDC_IN     (CDC_UART_DATA_EP_CNT+ 0x80)       //0x83
#endif

#define FUNC_ATTRS (DFU_ATTR_CAN_UPLOAD | DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_MANIFESTATION_TOLERANT)


uint8_t const desc_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 200),

#if CFG_TUD_DFU  
  TUD_DFU_DESCRIPTOR(ITF_NUM_DFU_MODE, ALT_COUNT, STRID_UDF, FUNC_ATTRS, 1000, CFG_TUD_DFU_XFER_BUFSIZE),
#endif

#if OPT_CMSIS_DAPV1
    TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_VENDOR, STRID_INTERFACE_DAP1, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report), EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN, CFG_TUD_HID_EP_BUFSIZE, 1),
#else
  TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, STRID_INTERFACE_DAP2, EPNUM_VENDOR_OUT, 0x80 | EPNUM_VENDOR_IN, TUD_OPT_HIGH_SPEED ? 512 : 64),
#endif

  // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC,   STRID_INTERFACE_CDC_UART, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, 0x80 | EPNUM_CDC_IN, TUD_OPT_HIGH_SPEED ? 512 : 64),
#if CFG_TUD_CDC >1  
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_2, STRID_INTERFACE_CDC_I2C, EPNUM_CDC2_NOTIF, 8, EPNUM_CDC2_OUT, 0x80 | EPNUM_CDC2_IN, TUD_OPT_HIGH_SPEED ? 512 : 64),
#endif
#if CFG_TUD_CDC >2  
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_3, STRID_INTERFACE_CDC_SPI, EPNUM_CDC3_NOTIF, 8, EPNUM_CDC3_OUT, 0x80 | EPNUM_CDC3_IN, TUD_OPT_HIGH_SPEED ? 512 : 64),
#endif  
  // Interface number, string index, EP Out & IN address, EP size
  
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index; // for multiple configurations
  return desc_configuration;
}

//--------------------------------------------------------------------+
// BOS Descriptor
//--------------------------------------------------------------------+

/* Microsoft OS 2.0 registry property descriptor
Per MS requirements https://msdn.microsoft.com/en-us/library/windows/hardware/hh450799(v=vs.85).aspx
device should create DeviceInterfaceGUIDs. It can be done by driver and
in case of real PnP solution device should expose MS "Microsoft OS 2.0
registry property descriptor". Such descriptor can insert any record
into Windows registry per device/configuration/interface. In our case it
will insert "DeviceInterfaceGUIDs" multistring property.

GUID is freshly generated and should be OK to use.

https://developers.google.com/web/fundamentals/native-hardware/build-for-webusb/
(Section Microsoft OS compatibility descriptors)
*/

#if 0

#define BOS_TOTAL_LEN      (TUD_BOS_DESC_LEN + TUD_BOS_WEBUSB_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

#define MS_OS_20_DESC_LEN  0xB2

// BOS Descriptor is required for webUSB
uint8_t const desc_bos[] =
{
  // total length, number of device caps
  TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 2),

  // Vendor Code, iLandingPage
  TUD_BOS_WEBUSB_DESCRIPTOR(VENDOR_REQUEST_WEBUSB, 1),

  // Microsoft OS 2.0 descriptor
  TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT)
};

uint8_t const * tud_descriptor_bos_cb(void)
{
  return desc_bos;
}


uint8_t const desc_ms_os_20[] =
{
  // Set header: length, type, windows version, total length
  U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

  // Configuration subset header: length, type, configuration index, reserved, configuration total length
  U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A),

  // Function Subset header: length, type, first interface, reserved, subset length
  U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), ITF_NUM_VENDOR, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A-0x08),

  // MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID
  U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sub-compatible

  // MS OS 2.0 Registry property descriptor: length, type
  U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A-0x08-0x08-0x14), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
  U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A), // wPropertyDataType, wPropertyNameLength and PropertyName "DeviceInterfaceGUIDs\0" in UTF-16
  'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00,
  'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, 0x00, 0x00,
  U16_TO_U8S_LE(0x0050), // wPropertyDataLength
	//bPropertyData: “{975F44D9-0D08-43FD-8B3E-127CA8AFFF9D}”.
  '{', 0x00, '9', 0x00, '7', 0x00, '5', 0x00, 'F', 0x00, '4', 0x00, '4', 0x00, 'D', 0x00, '9', 0x00, '-', 0x00,
  '0', 0x00, 'D', 0x00, '0', 0x00, '8', 0x00, '-', 0x00, '4', 0x00, '3', 0x00, 'F', 0x00, 'D', 0x00, '-', 0x00,
  '8', 0x00, 'B', 0x00, '3', 0x00, 'E', 0x00, '-', 0x00, '1', 0x00, '2', 0x00, '7', 0x00, 'C', 0x00, 'A', 0x00,
  '8', 0x00, 'A', 0x00, 'F', 0x00, 'F', 0x00, 'F', 0x00, '9', 0x00, 'D', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00
};

TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "Incorrect size");

#else
#define MS_OS_20_DESC_LEN  0xB2
#define BOS_TOTAL_LEN      (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)
uint8_t const desc_bos[] = {
    // total length, number of device caps
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),

    // Microsoft OS 2.0 descriptor
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, 1)
};

uint8_t const desc_ms_os_20[] = {
    // Set header: length, type, windows version, total length
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),
    // Configuration subset header: length, type, configuration index, reserved, configuration total length
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A),
    // Function Subset header: length, type, first interface, reserved, subset length
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), ITF_NUM_VENDOR, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08),
    // MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sub-compatible

    // MS OS 2.0 Registry property descriptor: length, type
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08 - 0x08 - 0x14), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
    U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A), // wPropertyDataType, wPropertyNameLength and PropertyName "DeviceInterfaceGUIDs\0" in UTF-16
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00,
    'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, 0x00, 0x00,
    U16_TO_U8S_LE(0x0050), // wPropertyDataLength
                           // bPropertyData "{CDB3B5AD-293B-4663-AA36-1AAE46463776}" as a UTF-16 string (b doesn't mean bytes)
    '{', 0x00, 'C', 0x00, 'D', 0x00, 'B', 0x00, '3', 0x00, 'B', 0x00, '5', 0x00, 'A', 0x00, 'D', 0x00, '-', 0x00,
    '2', 0x00, '9', 0x00, '3', 0x00, 'B', 0x00, '-', 0x00, '4', 0x00, '6', 0x00, '6', 0x00, '3', 0x00, '-', 0x00,
    'A', 0x00, 'A', 0x00, '3', 0x00, '6', 0x00, '-', 0x00, '1', 0x00, 'A', 0x00, 'A', 0x00, 'E', 0x00, '4', 0x00,
    '6', 0x00, '4', 0x00, '6', 0x00, '3', 0x00, '7', 0x00, '7', 0x00, '6', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00
};

TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "Incorrect size");

uint8_t const * tud_descriptor_bos_cb(void)
{
    // printf("tud_descriptor_bos_cb\r\n");
    return desc_bos;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
  // nothing to with DATA & ACK stage
  if (stage != CONTROL_STAGE_SETUP) return true;
  switch (request->bmRequestType_bit.type)
  {
    case TUSB_REQ_TYPE_VENDOR:
      switch (request->bRequest)
      {
        case 1:
          if (request->wIndex == 7 ){
            // Get Microsoft OS 2.0 compatible descriptor
            uint16_t total_len;
            memcpy(&total_len, desc_ms_os_20+8, 2);
            // printf("tud_vendor_control_xfer_cb  total_len:%d\r\n",total_len);
            return tud_control_xfer(rhport, request, (void*) &desc_ms_os_20, total_len);
          }else {
            return false;
          }
          default: break;
      }
      break;
    default: break;
  }
  // stall unknown request
  return false;
}


#endif



// array of pointer to string descriptors


static char const* string_desc_arr [] =
{
  [STRID_LANGID]             = (const char[]) { 0x09, 0x04 }, // supported language is English (0x0409)
  [STRID_MANUFACTURER]       = "PPVision",          // Manufacturer
  [STRID_PRODUCT]            = "WCH-Link@18",       // Product
  [STRID_SERIAL]             = "0000",              // Serial
  [STRID_INTERFACE_DAP2]     = "CMSIS-DAP v2",      // Interface descriptor for Bulk transport, **MUST** contain "CMSIS-DAP" to enable "CMSIS-DAP v2"
  [STRID_INTERFACE_DAP1]     = "CMSIS-DAP v1",      // Interface descriptor for HID transport
  [STRID_INTERFACE_CDC_I2C]  = "CDC-I2C",           // Interface descriptor for MSC interface
  [STRID_INTERFACE_CDC_SPI]  = "CDC-SPI",           // Interface descriptor for MSC interface
  [STRID_INTERFACE_CDC_UART] = "CDC-UART",     
  [STRID_UDF] = "UDF",     
  // STRID_MAC index is handled separately
};



static uint16_t _desc_str[32 + 1];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void) langid;
  size_t chr_count;

  switch ( index ) {
    case STRID_LANGID:
      memcpy(&_desc_str[1], string_desc_arr[0], 2);
      chr_count = 1;
      break;

    case STRID_SERIAL:
      chr_count = board_usb_get_serial(_desc_str + 1, 32);
      break;

    default:
      // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
      // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

      if ( !(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) ) return NULL;

      const char *str = string_desc_arr[index];

      // Cap at max char
      chr_count = strlen(str);
      size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1; // -1 for string type
      if ( chr_count > max_count ) chr_count = max_count;

      // Convert ASCII string into UTF-16
      for ( size_t i = 0; i < chr_count; i++ ) {
        _desc_str[1 + i] = str[i];
      }
      break;
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

  return _desc_str;
}
















