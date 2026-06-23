#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h" 
#include "hardware/flash.h"    
#include "hardware/sync.h"
#include "hardware/adc.h" // Needed for thermal entropy

#include "pio_usb.h"         
#include "tusb.h"
#include "xinput_host.h"
#include "tud_xinput.h"
#include "humanizer.h"

void tud_set_config_mode(bool enable);
bool tud_in_config_mode(void);

#define USB_HOST_PWR_PIN 18
#define CONFIG_MAGIC_NUM 0x1A2B3C4D 
#define FLASH_MAGIC_KEY 0x48554D50   
#define FLASH_TARGET_OFFSET (1024 * 1024) 

typedef struct {
    uint32_t magic;           
    uint16_t circ_error;      
    uint16_t jitter_mag;      
    uint16_t jitter_inner;    
    uint16_t jitter_outer;    
    uint16_t smoothing_rate;  
    uint16_t gate_level;      
    uint16_t tilt_deg;        
    uint16_t landing_var;     
    uint16_t diagonal_feel;   
    uint16_t anti_deadzone;   
    uint16_t passthrough;     
} humanizer_config_t;

static humanizer_config_t active_config;
static const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

static uint8_t current_report[20] = {0};
static Humanizer humanizer;

static volatile uint16_t latest_buttons = 0;
static volatile int16_t raw_lx = 0, raw_ly = 0;
static volatile int16_t raw_rx = 0, raw_ry = 0;
static volatile uint8_t raw_lt = 0, raw_rt = 0;

static uint32_t combo_start_time = 0;
static bool pending_save_and_reboot = false;
static uint32_t save_trigger_time = 0;
static uint32_t last_math_tick_us = 0; 
static int16_t preview_lx = 0, preview_ly = 0;
static uint32_t last_preview_ms = 0;

void load_settings_from_flash(void) {
    humanizer_config_t *flash_profile = (humanizer_config_t *) flash_target_contents;
    if (flash_profile->magic == FLASH_MAGIC_KEY) {
        memcpy(&active_config, flash_profile, sizeof(humanizer_config_t));
    } else {
        active_config.magic = FLASH_MAGIC_KEY;
        active_config.circ_error     = 3; 
        active_config.jitter_mag     = 50;    
        active_config.jitter_inner   = 15; 
        active_config.jitter_outer   = 100;
        active_config.smoothing_rate = 20; 
        active_config.gate_level     = 0;
        active_config.tilt_deg       = 5;   
        active_config.landing_var    = 50;
        active_config.diagonal_feel  = 15; 
        active_config.anti_deadzone  = 0;
        active_config.passthrough    = 0;
    }
}

void save_settings_to_flash(humanizer_config_t *new_config) {
    new_config->magic = FLASH_MAGIC_KEY;
    uint8_t page_buffer[FLASH_PAGE_SIZE];
    memset(page_buffer, 0, FLASH_PAGE_SIZE); 
    memcpy(page_buffer, new_config, sizeof(humanizer_config_t)); 
    uint32_t saved_interrupts = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, page_buffer, FLASH_PAGE_SIZE);
    restore_interrupts(saved_interrupts);
}

void process_web_serial_commands(void) {
    if (tud_cdc_available()) {
        char buffer[64];
        uint32_t count = tud_cdc_read(buffer, sizeof(buffer) - 1);
        buffer[count] = '\0'; 
        
        // --- 1. INSTANT RAM UPDATE (No Reboot) ---
        if (strncmp(buffer, "LIVE:", 5) == 0) {
            int c, jm, ji, jo, s, g, t, l, df, ad, p;
            if (sscanf(buffer, "LIVE:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", 
                       &c, &jm, &ji, &jo, &s, &g, &t, &l, &df, &ad, &p) == 11) {
                // Update the active configuration in RAM instantly
                active_config.circ_error     = (uint16_t)c;
                active_config.jitter_mag     = (uint16_t)jm;
                active_config.jitter_inner   = (uint16_t)ji;
                active_config.jitter_outer   = (uint16_t)jo;
                active_config.smoothing_rate = (uint16_t)s;
                active_config.gate_level     = (uint16_t)g;
                active_config.tilt_deg       = (uint16_t)t;
                active_config.landing_var    = (uint16_t)l;
                active_config.diagonal_feel  = (uint16_t)df;
                active_config.anti_deadzone  = (uint16_t)ad;
                active_config.passthrough    = (uint16_t)p;
            }
        }
        // --- 2. PERMANENT FLASH COMMIT (Triggers Reboot) ---
        else if (strncmp(buffer, "SAVE", 4) == 0) {
            tud_cdc_write_str("COMMITTING_TO_FLASH\r\n");
            tud_cdc_write_flush();
            pending_save_and_reboot = true;
            save_trigger_time = to_ms_since_boot(get_absolute_time());
        }
        // --- 3. EXIT CONFIG MODE (Triggers Reboot) ---
        else if (strncmp(buffer, "EXIT", 4) == 0) {
            tud_cdc_write_str("EXITING_CONFIG_MODE\r\n");
            tud_cdc_write_flush();
            watchdog_hw->scratch[0] = 0; // Clear the magic number
            watchdog_reboot(0, 0, 10);   // Reboot into normal mode
            while(1);
        }
    }
}


void core1_main(void) {
    gpio_init(USB_HOST_PWR_PIN);
    gpio_set_dir(USB_HOST_PWR_PIN, GPIO_OUT);
    gpio_put(USB_HOST_PWR_PIN, 1);
    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp = 16; 
    tuh_configure(BOARD_TUH_RHPORT, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);
    tuh_init(BOARD_TUH_RHPORT);
 while (true) {
        tuh_task();
        if (!tud_in_config_mode()) {
            uint32_t now = to_ms_since_boot(get_absolute_time());
            uint32_t ints = save_and_disable_interrupts();
            uint16_t btns = latest_buttons;
            restore_interrupts(ints);
            if ((btns & 0x0310) == 0x0310) {
                if (combo_start_time == 0) { combo_start_time = now; }
                else if (now - combo_start_time >= 3000) {
                    watchdog_hw->scratch[0] = CONFIG_MAGIC_NUM;
                    watchdog_reboot(0, 0, 10);
                    while(1);
                }
            } else { combo_start_time = 0; }
        }
    }

void tuh_xinput_report_received_cb(uint8_t dev_addr, uint8_t instance, xinputh_interface_t const* xid_itf, uint16_t len) {
    (void)dev_addr; (void)instance; (void)len;
    const xinput_gamepad_t* p = &xid_itf->pad;
    uint32_t ints = save_and_disable_interrupts();
    latest_buttons = p->wButtons;
    raw_lx = p->sThumbLX; raw_ly = p->sThumbLY;
    raw_rx = p->sThumbRX; raw_ry = p->sThumbRY;
    raw_lt = p->bLeftTrigger; raw_rt = p->bRightTrigger;
    restore_interrupts(ints);
    tuh_xinput_receive_report(dev_addr, instance);
}

usbh_class_driver_t const* usbh_app_driver_get_cb(uint8_t* driver_count) {
    *driver_count = 1;
    return &usbh_xinput_driver;
}

void tuh_xinput_mount_cb(uint8_t dev_addr, uint8_t instance, const xinputh_interface_t* xinput_itf) {
    (void)xinput_itf;
    tuh_xinput_receive_report(dev_addr, instance);
}


void tuh_xinput_umount_cb(uint8_t dev_addr, uint8_t instance) {
    (void)dev_addr; (void)instance;
    uint32_t ints = save_and_disable_interrupts();
    latest_buttons = 0;
    raw_lx = raw_ly = raw_rx = raw_ry = 0;
    raw_lt = raw_rt = 0;
    restore_interrupts(ints);
}

int main(void) {
    set_sys_clock_khz(120000, true);
    watchdog_start_tick(12); 
    stdio_init_all();

    // --- FINAL ENTROPY SEED: Internal Thermal Noise ---
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4); // Select internal temperature sensor
    (void)adc_read();    // Dummy read to stabilize
    srand(adc_read());   // True entropy seed

    load_settings_from_flash();
    humanizer_init(&humanizer);
    
    if (watchdog_hw->scratch[0] == CONFIG_MAGIC_NUM) { tud_set_config_mode(true); } 
    else { tud_set_config_mode(false); }
    
    tud_init(BOARD_TUD_RHPORT);
    multicore_launch_core1(core1_main);
    last_math_tick_us = time_us_32();
    
    while (true) {
        tud_task(); 
 if (tud_in_config_mode()) {
            process_web_serial_commands();

            uint32_t now_us = time_us_32();
            if (now_us - last_math_tick_us >= 4000) {
                last_math_tick_us += 4000;
                uint32_t ints = save_and_disable_interrupts();
                int16_t lx = raw_lx, ly = raw_ly, rx = raw_rx, ry = raw_ry;
                restore_interrupts(ints);
                humanizer_process(&humanizer, &lx, &ly, &rx, &ry,
                                  active_config.circ_error,
                                  active_config.jitter_mag, active_config.jitter_inner, active_config.jitter_outer,
                                  active_config.smoothing_rate, active_config.gate_level,
                                  active_config.tilt_deg, active_config.landing_var,
                                  active_config.diagonal_feel, active_config.anti_deadzone,
                                  active_config.passthrough);
                preview_lx = lx; preview_ly = ly;
            }

            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            if (now_ms - last_preview_ms >= 33) {
                last_preview_ms = now_ms;
                char msg[24];
                int n = snprintf(msg, sizeof(msg), "P:%d,%d\n", preview_lx, preview_ly);
                if (n > 0) { tud_cdc_write((const uint8_t*)msg, (uint32_t)n); tud_cdc_write_flush(); }
            }

            if (pending_save_and_reboot) {
                if (to_ms_since_boot(get_absolute_time()) - save_trigger_time > 500) {
                    multicore_reset_core1();
                    save_settings_to_flash(&active_config);
                    watchdog_hw->scratch[0] = 0;
                    watchdog_reboot(0, 0, 10);
                    while(1);
            }
        }
        } else {
            uint32_t current_time_us = time_us_32();
            if (current_time_us - last_math_tick_us >= 4000) {
                last_math_tick_us += 4000;
                uint32_t ints = save_and_disable_interrupts();
                int16_t lx = raw_lx; int16_t ly = raw_ly;
                int16_t rx = raw_rx; int16_t ry = raw_ry;
                uint8_t lt = raw_lt; uint8_t rt = raw_rt;
                uint16_t btns = latest_buttons;
                restore_interrupts(ints);

                humanizer_process(&humanizer, &lx, &ly, &rx, &ry,
                                  active_config.circ_error, 
                                  active_config.jitter_mag, active_config.jitter_inner, active_config.jitter_outer,
                                  active_config.smoothing_rate, active_config.gate_level,
                                  active_config.tilt_deg, active_config.landing_var, 
                                  active_config.diagonal_feel, active_config.anti_deadzone,
                                  active_config.passthrough);

                current_report[0] = 0x00; current_report[1] = 0x14;
                current_report[2] = btns & 0xFF; current_report[3] = (btns >> 8) & 0xFF;
                current_report[4] = lt; current_report[5] = rt;
                current_report[6] = lx & 0xFF; current_report[7] = (lx >> 8) & 0xFF;
                current_report[8] = ly & 0xFF; current_report[9] = (ly >> 8) & 0xFF;
                current_report[10] = rx & 0xFF; current_report[11] = (rx >> 8) & 0xFF;
                current_report[12] = ry & 0xFF; current_report[13] = (ry >> 8) & 0xFF;
                tud_xinput_send_report(current_report, sizeof(current_report));
            }
        }
    }
    return 0;
}
