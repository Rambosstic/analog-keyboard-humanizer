#include "tusb.h"
#include <string.h>

// Lean XInput Device Descriptor
static const uint8_t desc_device[] = {
    0x12,       // bLength
    0x01,       // bDescriptorType (Device)
    0x00, 0x02, // bcdUSB 2.00
    0xFF,       // bDeviceClass (Vendor Specific)
    0xFF,       // bDeviceSubClass
    0xFF,       // bDeviceProtocol
    0x08,       // bMaxPacketSize0 (Strictly 8 bytes for Xbox 360)
    0x5E, 0x04, // idVendor 0x045E (Microsoft)
    0x91, 0x02, // idProduct 0x0291 (Fresh PID to reset Windows driver registry)
    0x14, 0x01, // bcdDevice 0x0114
    0x01,       // iManufacturer
    0x02,       // iProduct
    0x03,       // iSerialNumber
    0x01,       // bNumConfigurations
};

uint8_t const * tud_descriptor_device_cb(void)
{
    return desc_device;
}

// Clean Configuration Descriptor matching strict Gamepad endpoints
static const uint8_t desc_configuration[] = {
    // Configuration Descriptor
    0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0xA0, 0xFA,

    // Interface 0: Gamepad Controls Only
    0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0x5D, 0x01, 0x00,
    0x11, 0x21, 0x00, 0x01, 0x01, 0x25, 0x81, 0x14,
    0x00, 0x00, 0x00, 0x00, 0x13, 0x02, 0x08, 0x00, 0x00,
    
    // Endpoint 1: IN (Data to PC)
    0x07, 0x05, 0x81, 0x03, 0x20, 0x00, 0x04,
    // Endpoint 2: OUT (Rumble commands from PC)
    0x07, 0x05, 0x02, 0x03, 0x20, 0x00, 0x08
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_configuration;
}

// Strings
static const char* string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },
    "Microsoft Corporation",
    "Xbox 360 Controller",
    "000000000001"
};

static uint16_t _desc_str[128];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) return NULL;
        const char* str = string_desc_arr[index];
        chr_count = (uint8_t) strlen(str);
        if (chr_count > 127) chr_count = 127;
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1+i] = str[i];
        }
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2*chr_count + 2));
    return _desc_str;
}
