/**
 * @file        EspBoard_Lite.cpp
 * @project     voice control projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2026-08-03
 * tabsize      4
 * 
 * This Revision: $Id: EspBoard_Lite.cpp 2006 2026-08-12 18:59:44Z  $
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
 * ESP32-S3-BOX-Lite device
 */

#include "esp_err.h"
#include "esp_log.h"
#include "EspBoard_Lite.h"

#define TAG "EspBoard_Lite"


class LcDisplay_Lite : public LcDisplay_ST7789 {
protected:
    static const Config_LcDisplay_ST7789 _config;
public:
    LcDisplay_Lite() : LcDisplay_ST7789(&_config) {}

};


const Config_LcDisplay_ST7789 LcDisplay_Lite::_config = {
    .width = 320,
    .height = 240,
    .miso = -1,
    .mosi = 6,
    .sclk = 7,
    .cs = 5,
    .dc = 4,
    .rst = 48,
    .pclk_hz = (40 * 1000 * 1000),
    .color_space = ESP_LCD_COLOR_SPACE_RGB,
    .color_invert = true,
    .swap_xy = true,
    .mirror_x = false,
    .mirror_y = true,
};


class LcBacklight_Lite : public LcBacklight {
public:
    LcBacklight_Lite() : LcBacklight(45,true) {}
};


EspBoard_BoxLite::EspBoard_BoxLite() { 
    _backlight = new LcBacklight_Lite(); 
    _display = new LcDisplay_Lite();
}
