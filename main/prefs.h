/*
    Define installation-specific choices: which hardware?
 */
#pragma once

// select which hardware variant to support
#define USE_ESP_BOX_LITE    1
#define USE_MUMA            0
#define USE_CUSTOM_BOARD    0


// set 1 to have a web interface
#define USE_WEBSERVER 1   


#if USE_ESP_BOX_LITE
 #define HAS_SENSORS 1
 #define HAS_DISPLAY 1
 #define HAS_RGB_LED 0
#elif USE_MUMA
 #define HAS_SENSORS 0
 #define HAS_DISPLAY 1
 #define HAS_RGB_LED 0
#else // custom board
 #define HAS_SENSORS 0      // set to 1 if we have climate sensors
 #define HAS_DISPLAY 0      // set to 1 if we have an LVGL-controlled display
 #define HAS_RGB_LED 1      // set to 1 if we have a common-anode RGB LED
#endif