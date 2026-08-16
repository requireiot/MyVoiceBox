/**
 * @file        EspBoard_Custom.cpp
 * @project     voice control projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2026-08-03
 * tabsize      4
 * 
 * This Revision: $Id: EspBoard_Custom.cpp 2013 2026-08-16 12:18:31Z  $
 */

/*
   Copyright (C) 2026 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. 
   If a copy of the MPL was not distributed with this file, You can obtain one at 
   http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief Implementation of hardware control specific to 
 * custom device with 2x INMP441-style digital mic, and MAX98357A mono DAC 
 */

#include "esp_err.h"
#include "esp_log.h"
#include "EspBoard_Custom.h"

#define TAG "EspBoard_Custom"

//------------------------------------------------------------------------------
// ST7789 based 320x240 LCD

class LcDisplay_Custom_ST7789 : public LcDisplay_ST7789 {
protected:
    static const Config_LcDisplay_ST7789 _config;
public:
    LcDisplay_Custom_ST7789() : LcDisplay_ST7789(&_config) {}

};


const Config_LcDisplay_ST7789 LcDisplay_Custom_ST7789::_config = {
    .width = 320,
    .height = 240,
    .miso = -1,
    .mosi = 12,
    .sclk = 13,
    .cs = 9,
    .dc = 11,
    .rst = 10,
    .pclk_hz = (40 * 1000 * 1000),
    .color_space = ESP_LCD_COLOR_SPACE_RGB,
    .color_invert = true,
    .swap_xy = true,
    .mirror_x = false,
    .mirror_y = true,
};


class LcBacklight_Custom_ST7789 : public LcBacklight {
public:
    LcBacklight_Custom_ST7789() : LcBacklight(14,false) {}
};


EspBoard_Custom_LCD::EspBoard_Custom_LCD() : EspBoard_Custom() { 
    _backlight = new LcBacklight_Custom_ST7789(); 
    _display = new LcDisplay_Custom_ST7789();
}


//------------------------------------------------------------------------------
// ST77916 based 360x360 round LCD, QSPI interface

class LcDisplay_Custom_ST77916 : public LcDisplay_ST77916 {
protected:
    static const Config_LcDisplay_ST77916 _config;
public:
    LcDisplay_Custom_ST77916() : LcDisplay_ST77916(&_config) {}

};


const Config_LcDisplay_ST77916 LcDisplay_Custom_ST77916::_config = {
    .width = 360,
    .height = 360,
    .sclk = 14,
    .data0 = 13,
    .data1 = 12,
    .data2 = 11,
    .data3 = 10,
    .cs = 9,
    .rst = 8,
    .color_space = ESP_LCD_COLOR_SPACE_RGB,
    .color_invert = false,
    .swap_xy = 0,
    .mirror_x = 0,
    .mirror_y = 0,
};


class LcBacklight_Custom_ST77916 : public LcBacklight {
public:
    LcBacklight_Custom_ST77916() : LcBacklight(18,false) {}
};


EspBoard_Custom_RoundLCD::EspBoard_Custom_RoundLCD() : EspBoard_Custom() { 
    _backlight = new LcBacklight_Custom_ST77916(); 
    _display = new LcDisplay_Custom_ST77916();
}
