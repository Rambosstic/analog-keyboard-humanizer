#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"

#include "tusb.h"
#include "xinput_host.h"
#include "tud_xinput.h"
#include "humanizer.h"

#define USB_HOST_PWR_PIN 18

static uint8_t current_report[20] = {0};
static volatile bool report_ready = false; 
static Humanizer humanizer;

// ====================================================================
// CORE 1: EXCLUSIVE USB HOST CONTROLLER
// ====================================================================
void core1_main(void)
{
    // Turn on the 5V Boost Converter to power the USB Type-A Slot
    gpio_init(USB_HOST_PWR_PIN);
    gpio_set_dir(USB_HOST_PWR_PIN, GPIO_OUT);
    gpio_put(USB_HOST_PWR_PIN, 1);

    // Map Pico-PIO-USB to the exact hardware pins on the Adafruit Feather Host Board
    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp = 16; // Sets D+ to GPIO 16, which automatically maps D- to GPIO 17
    
    // Bind the pin configuration to the host stack before initializing it
    tuh_configure(BOARD_TUH_RHPORT, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);

    // Fire up the Host Engine
    tuh_init(BOARD_TUH_RHPORT);
    
    while (true) {
        tuh_task();
    }
}

usbh_class_driver_t const* usbh_app_driver_get_cb(uint8_t* driver_count)
{
    *driver_count = 1;
    return &usbh_xinput_driver;
}

// ====================================================================
// HOST DEVICE EVENT CALLBACKS
// ====================================================================
void tuh_xinput_mount_cb(uint8_t dev_addr, uint8_t instance,
                          const xinputh_interface_t* xinput_itf)
{
    (void)dev_addr; (void)instance; (void)xinput_itf;
    // Kickstart the data loop by asking the connected device for its first report
    tuh_xinput_receive_report(dev_addr, instance);
}

void tuh_xinput_umount_cb(uint8_t dev_addr, uint8_t instance)
{
    (void)dev_addr; (void)instance;
    report_ready = false;
    memset(current_report, 0, sizeof(current_report));
}

void tuh_xinput_report_received_cb(uint8_t dev_addr, uint8_t instance,
                                    xinputh_interface_t const* xid_itf,
                                    uint16_t len)
{
    (void)dev_addr; (void)instance; (void)len;
    const xinput_gamepad_t* p = &xid_itf->pad;
    int16_t lx = p->sThumbLX;
    int16_t ly = p->sThumbLY;
    int16_t rx = p->sThumbRX;
    int16_t ry = p->sThumbRY;
    
    // Run the joystick values through your custom humanizer logic
    humanizer_process(&humanizer, &lx, &ly, &rx, &ry);
    
    // Pack the modified controller report
    current_report[0]  = 0x00;
    current_report[1]  = 0x14;
    current_report[2]  = (p->wButtons) & 0xFF;
    current_report[3]  = (p->wButtons >> 8) & 0xFF;
    current_report[4]  = p->bLeftTrigger;
    current_report[5]  = p->bRightTrigger;
    current_report[6]  = lx & 0xFF;
    current_report[7]  = (lx >> 8) & 0xFF;
    current_report[8]  = ly & 0xFF;
    current_report[9]  = (ly >> 8) & 0xFF;
    current_report[10] = rx & 0xFF;
    current_report[11] = (rx >> 8) & 0xFF;
    current_report[12] = ry & 0xFF;
    current_report[13] = (ry >> 8) & 0xFF;
    report_ready = true;
    
    // Keep the loop cycling continuously
    tuh_xinput_receive_report(dev_addr, instance);
}

// ====================================================================
// CORE 0: PC DEVICE EMULATION LOOP
// ====================================================================
int main(void)
{
    // Set the system clock to exactly 120MHz (Mandatory for PIO USB bit-banging)
    set_sys_clock_khz(120000, true);

    stdio_init_all();
    humanizer_init(&humanizer);
    
    // Initialize the PC Device Emulation Stack
    tud_init(BOARD_TUD_RHPORT);
    
    // Spin up the Host Stack on Core 1
    multicore_launch_core1(core1_main);
    
    while (true) {
        tud_task();
        if (report_ready) {
            tud_xinput_send_report(current_report, sizeof(current_report));
            report_ready = false;
        }
    }
    return 0;
}
