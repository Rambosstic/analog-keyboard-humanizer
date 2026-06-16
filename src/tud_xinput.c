#include "tusb.h"
#include "device/usbd_pvt.h"
#include "tud_xinput.h"
#include "hardware/watchdog.h"
#include "pico/time.h" 
#include <string.h>

#define ENDPOINT_SIZE 20  

static bool is_config_mode = false;

void tud_set_config_mode(bool enable) {
    is_config_mode = enable;
}

bool tud_in_config_mode(void) {
    return is_config_mode;
}

// ====================================================================
// 1. ISOLATED DEVICE DESCRIPTORS
// ====================================================================

// Mode A: Official Xbox 360 Controller (Endpoint 0: 8 Bytes)
static const uint8_t desc_device_xinput[] = {
    0x12, 0x01, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x08, // Max Packet Size 0 = 8
    0x5E, 0x04, 0x8E, 0x02, 0x14, 0x01, 0x01, 0x02, 0x03, 0x01
};

// Mode B: Raspberry Pi CDC Serial Port (Endpoint 0: 8 Bytes - MATCHES CONFIG)
static const uint8_t desc_device_cdc[] = {
    0x12, 0x01, 0x00, 0x02, 0xEF, 0x02, 0x01, 0x08, // Max Packet Size 0 = 8 (CRITICAL FIX)
    0x8A, 0x2E, 0x0A, 0x00, 0x14, 0x01, 0x01, 0x02, 0x03, 0x01 // VID/PID: 0x2E8A / 0x000A
};

uint8_t const * tud_descriptor_device_cb(void) {
    return is_config_mode ? desc_device_cdc : desc_device_xinput;
}

// ====================================================================
// 2. ISOLATED CONFIGURATION DESCRIPTORS
// ====================================================================

// Mode A: Xbox 360 Layout (49 Bytes)
static const uint8_t desc_config_xinput[] = {
    0x09, 0x02, 0x31, 0x00, 0x01, 0x01, 0x00, 0xA0, 0xFA, 
    0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0x5D, 0x01, 0x00, 
    0x11, 0x21, 0x00, 0x01, 0x01, 0x25, 0x81, 0x14,       
    0x00, 0x00, 0x00, 0x00, 0x13, 0x02, 0x08, 0x00, 0x00,
    0x07, 0x05, 0x81, 0x03, 0x20, 0x00, 0x04,             
    0x07, 0x05, 0x02, 0x03, 0x20, 0x00, 0x08              
};

// Mode B: Standard CDC Serial Layout (75 Bytes)
static const uint8_t desc_config_cdc[] = {
    0x09, 0x02, 0x4B, 0x00, 0x02, 0x01, 0x00, 0xC0, 0x32, 
    0x08, 0x0B, 0x00, 0x02, 0x02, 0x02, 0x01, 0x00,       
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00, 
    0x05, 0x24, 0x00, 0x10, 0x01,                         
    0x05, 0x24, 0x01, 0x00, 0x01,                         
    0x04, 0x24, 0x02, 0x02,                               
    0x05, 0x24, 0x06, 0x00, 0x01,                         
    0x07, 0x05, 0x83, 0x03, 0x08, 0x00, 0x10,             
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00, 
    0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00,             
    0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00              
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return is_config_mode ? desc_config_cdc : desc_config_xinput;
}

// ====================================================================
// 3. SEPARATED STRING DESCRIPTORS
// ====================================================================
static const char* string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 }, 
    "Microsoft Corporation",       
    "Xbox 360 Controller",         
    "000000000001"                 
};

static uint16_t _desc_str[128];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) return NULL;
        const char* str = string_desc_arr[index];
        
        // Morph product identification labels dynamically to look clean
        if (index == 2 && is_config_mode) {
            str = "Humanizer Configuration Port";
        }

        chr_count = (uint8_t) strlen(str);
        if (chr_count > 127) chr_count = 127;
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1+i] = str[i];
        }
    }
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2*chr_count + 2));
    return _desc_str;
}

// ====================================================================
// 5. APPLICATION CLASS DRIVER INTERNAL ROUTINES
// ====================================================================
static uint8_t endpoint_in  = 0xFF;
static uint8_t endpoint_out = 0xFF;
static uint8_t ep_in_buffer[ENDPOINT_SIZE];
static uint8_t ep_out_buffer[ENDPOINT_SIZE];

static void xinput_init(void) {
    endpoint_in  = 0xFF;
    endpoint_out = 0xFF;
}

static bool xinput_deinit(void) {
    xinput_init();
    return true;
}

static void xinput_reset(uint8_t rhport) {
    (void)rhport;
    xinput_init();
}

static uint16_t xinput_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_length) {
    if (is_config_mode) return 0;

    uint16_t driver_length = sizeof(tusb_desc_interface_t) + (itf_desc->bNumEndpoints * sizeof(tusb_desc_endpoint_t)) + 17;
    TU_VERIFY(max_length >= driver_length, 0);

    uint8_t const *cur_desc = tu_desc_next(itf_desc);
    uint8_t found = 0;

    while (found < itf_desc->bNumEndpoints && driver_length <= max_length) {
        tusb_desc_endpoint_t const *ep_desc = (tusb_desc_endpoint_t const *)cur_desc;
        if (TUSB_DESC_ENDPOINT == tu_desc_type(ep_desc)) {
            TU_ASSERT(usbd_edpt_open(rhport, ep_desc));
            if (tu_edpt_dir(ep_desc->bEndpointAddress) == TUSB_DIR_IN)
                endpoint_in = ep_desc->bEndpointAddress;
            else
                endpoint_out = ep_desc->bEndpointAddress;
            found++;
        }
        cur_desc = tu_desc_next(cur_desc);
    }

    if (endpoint_out != 0xFF)
        usbd_edpt_xfer(BOARD_TUD_RHPORT, endpoint_out, ep_out_buffer, ENDPOINT_SIZE);

    return driver_length;
}

static bool xinput_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
    (void)rhport; (void)stage; (void)request;
    return true;
}

static bool xinput_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
    (void)rhport; (void)result; (void)xferred_bytes;
    if (!is_config_mode && ep_addr == endpoint_out)
        usbd_edpt_xfer(BOARD_TUD_RHPORT, endpoint_out, ep_out_buffer, ENDPOINT_SIZE);
    return true;
}

static const usbd_class_driver_t xinput_driver = {
    .name = "XINPUT",
    .init             = xinput_init,
    .deinit           = xinput_deinit,
    .reset            = xinput_reset,
    .open             = xinput_open,
    .control_xfer_cb  = xinput_control_xfer_cb,
    .xfer_cb          = xinput_xfer_cb,
    .sof              = NULL
};

// CRITICAL FIX: Hide the XInput application driver from TinyUSB when in Serial Config Mode
usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t *driver_count) {
    if (is_config_mode) {
        *driver_count = 0;
        return NULL;
    }
    *driver_count = 1;
    return &xinput_driver;
}

bool tud_xinput_send_report(const uint8_t *report, uint16_t len) {
    if (is_config_mode || !tud_ready() || endpoint_in == 0xFF) return false;
    if (usbd_edpt_busy(BOARD_TUD_RHPORT, endpoint_in)) return false;
    if (len > ENDPOINT_SIZE) len = ENDPOINT_SIZE;
    memcpy(ep_in_buffer, report, len);
    usbd_edpt_claim(BOARD_TUD_RHPORT, endpoint_in);
    usbd_edpt_xfer(BOARD_TUD_RHPORT, endpoint_in, ep_in_buffer, len);
    usbd_edpt_release(BOARD_TUD_RHPORT, endpoint_in);
    return true;
}
