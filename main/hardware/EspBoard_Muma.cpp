/**
 * @file        EspBoard_Muma.cpp
 * @project     voice control projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2026-08-03
 * tabsize      4
 * 
 * This Revision: $Id: EspBoard_Muma.cpp 2007 2026-08-13 14:15:03Z  $
 */

/*
   Copyright (C) 2026 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. 
   If a copy of the MPL was not distributed with this file, You can obtain one at 
   http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief Implementation of hardware and audio codec control specific to 
 * Spotpear "DeepSeek AI Chat Box ESP32-S3 1.54 inch LCD" device 
 * (https://spotpear.com/shop/ESP32-S3-AI-1.54-inch-LCD-Display-TouchScreen-N16R8-DeepSeek-Box.html)
 */

#include "EspBoard_Muma.h"
#include "AudioCodec_ES8311.h"

#define TAG "EspBoard_Muma"


const Config_AudioCodec_ES8311 AudioCodec_Muma::_config = {
    .i2c_num = 1,
    .scl = 14,
    .sda = 15, 
    .i2s_num = 0,
    .bclk = 9,
    .lrck = 45,
    .din = 10,
    .dout = 8,
    .mclk = 16,
    .pa_ctrl = 46,
    .mic_gain = 24.0,
    .volume = 80.0
};


class LcDisplay_Muma : public LcDisplay_ST7789 {
protected:
    static const Config_LcDisplay_ST7789 _config;
public:
    LcDisplay_Muma() : LcDisplay_ST7789(&_config) {}
};


const Config_LcDisplay_ST7789 LcDisplay_Muma::_config = {
    .width = 240,
    .height = 240,
    .miso = -1,
    .mosi = 2,
    .sclk = 4,
    .cs = 5,
    .dc = 47,
    .rst = 38,
    .pclk_hz = (40 * 1000 * 1000),
    .color_space = ESP_LCD_COLOR_SPACE_RGB,
    .color_invert = true,
    .swap_xy = false,
    .mirror_x = false,
    .mirror_y = false,
};


class LcBacklight_Muma : public LcBacklight {
public:
    LcBacklight_Muma() : LcBacklight(42,true) {}
};


EspBoard_Muma::EspBoard_Muma() { 
    _backlight = new LcBacklight_Muma(); 
    _display = new LcDisplay_Muma();
}
