#include "app_config.h"
#if APP_USB_RNDIS_ENABLED
#include "tusb.h"
#include "main.h"

#include <string.h>

/* TinyUSB development identifiers; assign a product VID/PID before distribution. */
static const tusb_desc_device_t device = {
    .bLength = sizeof(tusb_desc_device_t), .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200, .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON, .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE, .idVendor = 0xCAFE, .idProduct = 0x4021,
    .bcdDevice = 0x0100, .iManufacturer = 1, .iProduct = 2, .iSerialNumber = 3,
    .bNumConfigurations = 1
};
static const uint8_t configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUD_CONFIG_DESC_LEN + TUD_RNDIS_DESC_LEN, TUSB_DESC_CONFIG_ATT_SELF_POWERED, 100),
    TUD_RNDIS_DESCRIPTOR(0, 4, 0x81, 8, 0x02, 0x82, 64)
};
static const uint8_t ms_string[] = {18, TUSB_DESC_STRING, 'M',0,'S',0,'F',0,'T',0,'1',0,'0',0,'0',0,0x20,0};
static const uint8_t ms_compatible[] = {
    40,0,0,0, 0,1, 4,0, 1, 0,0,0,0,0,0,0,
    0,1, 'R','N','D','I','S',0,0,0, '5','1','6','2','0','0','1',0, 0,0,0,0,0,0
};

const uint8_t* tud_descriptor_device_cb(void) { return (const uint8_t*)&device; }
const uint8_t* tud_descriptor_configuration_cb(uint8_t index) { return index == 0U ? configuration : NULL; }

const uint16_t* tud_descriptor_string_cb(uint8_t index, uint16_t language)
{
    (void)language;
    static uint16_t buffer[64];
    if(index == 0xEEU) { return (const uint16_t*)(const void*)ms_string; }
    if(index == 0U) { buffer[0] = 0x0304; buffer[1] = 0x0409; return buffer; }
    static const char* const strings[] = {"", "AirQualitySensors", "AirQualitySensors USB Ethernet", "", "RNDIS Network"};
    char serial[25];
    const char* value;
    if(index == 3U)
    {
        static const char hex[] = "0123456789ABCDEF";
        const uint32_t uid[] = {HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()};
        for(unsigned int i = 0; i < 24U; ++i) { serial[i] = hex[(uid[i / 8U] >> ((7U - i % 8U) * 4U)) & 15U]; }
        serial[24] = 0;
        value = serial;
    }
    else
    {
        if(index >= sizeof(strings) / sizeof(strings[0])) { return NULL; }
        value = strings[index];
    }
    size_t length = strlen(value);
    if(length > 63U) { length = 63U; }
    for(size_t i = 0; i < length; ++i) { buffer[1U + i] = (uint8_t)value[i]; }
    buffer[0] = (uint16_t)(0x0300U | (2U * length + 2U));
    return buffer;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request)
{
    if(stage != CONTROL_STAGE_SETUP) { return true; }
    if(request->bmRequestType == 0xC0U && request->bRequest == 0x20U && request->wIndex == 4U)
    {
        return tud_control_xfer(rhport, request, (void*)(uintptr_t)ms_compatible, sizeof(ms_compatible));
    }
    return false;
}
#endif
