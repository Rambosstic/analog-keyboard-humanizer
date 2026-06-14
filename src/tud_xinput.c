#include "tusb.h"
#include "device/usbd_pvt.h"
#include "tud_xinput.h"
#include <string.h>

// Corrected buffer size to 64 to fully safely handle audio and gamepad layout pipelines
#define ENDPOINT_SIZE 64  

static uint8_t endpoint_in  = 0xFF;
static uint8_t endpoint_out = 0xFF;
static uint8_t ep_in_buffer[ENDPOINT_SIZE];
static uint8_t ep_out_buffer[ENDPOINT_SIZE];

static void xinput_init(void)
{
    endpoint_in  = 0xFF;
    endpoint_out = 0xFF;
    memset(ep_out_buffer, 0, ENDPOINT_SIZE);
    memset(ep_in_buffer,  0, ENDPOINT_SIZE);
}

static bool xinput_deinit(void)
{
    xinput_init();
    return true;
}

static void xinput_reset(uint8_t rhport)
{
    (void)rhport;
    xinput_init();
}

static uint16_t xinput_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_length)
{
    uint16_t driver_length = sizeof(tusb_desc_interface_t) +
                             (itf_desc->bNumEndpoints * sizeof(tusb_desc_endpoint_t)) + 16;

    TU_VERIFY(max_length >= driver_length, 0);

    uint8_t const *cur_desc = tu_desc_next(itf_desc);
    uint8_t found = 0;

    while (found < itf_desc->bNumEndpoints && driver_length <= max_length)
    {
        tusb_desc_endpoint_t const *ep_desc = (tusb_desc_endpoint_t const *)cur_desc;
        if (TUSB_DESC_ENDPOINT == tu_desc_type(ep_desc))
        {
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

static bool xinput_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    (void)rhport; (void)stage; (void)request;
    return true;
}

static bool xinput_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
    (void)rhport; (void)result; (void)xferred_bytes;
    if (ep_addr == endpoint_out)
        usbd_edpt_xfer(BOARD_TUD_RHPORT, endpoint_out, ep_out_buffer, ENDPOINT_SIZE);
    return true;
}

static const usbd_class_driver_t xinput_driver =
{
#if CFG_TUSB_DEBUG >= 2
    .name = "XINPUT",
#else
    .name = NULL,
#endif
    .init             = xinput_init,
    .deinit           = xinput_deinit,
    .reset            = xinput_reset,
    .open             = xinput_open,
    .control_xfer_cb  = xinput_control_xfer_cb,
    .xfer_cb          = xinput_xfer_cb,
    .sof              = NULL
};

usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t *driver_count)
{
    *driver_count = 1;
    return &xinput_driver;
}

bool tud_xinput_send_report(const uint8_t *report, uint16_t len)
{
    if (!tud_ready() || endpoint_in == 0xFF) return false;
    if (usbd_edpt_busy(BOARD_TUD_RHPORT, endpoint_in)) return false;
    if (len > ENDPOINT_SIZE) len = ENDPOINT_SIZE;
    memcpy(ep_in_buffer, report, len);
    usbd_edpt_claim(BOARD_TUD_RHPORT, endpoint_in);
    usbd_edpt_xfer(BOARD_TUD_RHPORT, endpoint_in, ep_in_buffer, len);
    usbd_edpt_release(BOARD_TUD_RHPORT, endpoint_in);
    return true;
}
