/*
    GPIO pins and other hardware constants
*/
#pragma once
#include "prefs.h"


#if USE_CUSTOM_BOARD // custom board with INMP441 and MAX98357A modules

 // if we have an RGB LED
 #define PIN_LED_R          4
 #define PIN_LED_A          5   // common anode
 #define PIN_LED_G          6
 #define PIN_LED_B          7

 // if we have sensors 
 #define I2C_PORT           0
 #define PIN_I2C_SDA        1
 #define PIN_I2C_SCL        2 

#elif USE_ESP_BOX_LITE

 #define PIN_LED_A   -1 // common anode
 #define PIN_LED_R   -1
 #define PIN_LED_G   -1
 #define PIN_LED_B   -1

 // if we have sensors 
 #define I2C_PORT           0      // On ESP-BOX_Lite, #1 is occupied
 #define PIN_I2C_SCL 40
 #define PIN_I2C_SDA 41

#elif USE_MUMA

 #define PIN_LED_A   -1 // common anode
 #define PIN_LED_R   -1
 #define PIN_LED_G   -1
 #define PIN_LED_B   -1

 // if we have sensors 
 #define I2C_PORT           0      // On ESP-BOX_Lite, #1 is occupied
 #define PIN_I2C_SCL 39
 #define PIN_I2C_SDA 40

#endif 
