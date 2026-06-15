cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD adafruit_feather_rp2040_usb_host)

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(analog_keyboard_humanizer C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Pico PIO USB path
if(NOT PICO_PIO_USB_PATH)
    set(PICO_PIO_USB_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/Pico-PIO-USB)
endif()

# Tell tusb_xinput where TinyUSB lives
set(TINYUSB_PATH ${PICO_SDK_PATH}/lib/tinyusb)

# Add Pico PIO USB first
add_subdirectory(${PICO_PIO_USB_PATH} Pico-PIO-USB)

# Add tusb_xinput
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/tusb_xinput xinput_host)

add_executable(analog_keyboard_humanizer
    src/main.c
    src/humanizer.c
    src/tud_xinput.c
)

target_include_directories(analog_keyboard_humanizer PRIVATE
    src
    ${PICO_PIO_USB_PATH}/src
    ${TINYUSB_PATH}/src
)

target_compile_definitions(analog_keyboard_humanizer PRIVATE
    PIO_USB_DP_PIN_DEFAULT=16
)

target_link_libraries(analog_keyboard_humanizer PRIVATE
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_dma
    tinyusb_device
    tinyusb_board
    tinyusb_host
    tinyusb_pico_pio_usb
    xinput_host
)

add_library(tinyusb ALIAS tinyusb_host)

pico_add_extra_outputs(analog_keyboard_humanizer)
