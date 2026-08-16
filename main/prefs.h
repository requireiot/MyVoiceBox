/*
    Define installation-specific choices: which hardware?
 */
#pragma once

// select which hardware variant to support
#define USE_ESP_BOX_LITE    1
#define USE_MUMA            0
#define USE_CUSTOM_BOARD    0

// set to 1 if we have climate sensors
#define HAS_SENSORS 1

// set to 1 if we have an LVGL-controlled display
#define HAS_DISPLAY 1

// set to 1 if we have a common-anode RGB LED attached to GPIO pins  
#define HAS_RGB_LED 0

// set 1 to have a web interface
#define USE_WEBSERVER 1   

// if we have climate sensors
#define ENS160_ADDRESS 0x53     // or 0x52
#define BME280_ADDRESS 0x76     // or 0x77
#define AHT21_ADDRESS  0x38
