/**
 * @file        EspBoard.h
 * @project     voice control projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2026-08-03
 * tabsize      4
 * 
 * This Revision: $Id: EspBoard.h 2014 2026-08-16 16:07:27Z  $
 */

/*
   Copyright (C) 2026 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, 
   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
   obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

#pragma once

#include "esp_log.h"
#include "esp_lvgl_port.h" 
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_types.h"
#include <Arduino.h>

#include "error_checks.h"

//==============================================================================
#pragma region LCD backlight


class LcBacklight {
protected:
    int _pin;
    bool _invert;
public:
    LcBacklight( int pin, bool invert) : _pin(pin), _invert(invert) {}
    void init()
        { ledcAttach(_pin,400,10); }
    void set_brightness(int brightness_percent)
        {
            brightness_percent = constrain(brightness_percent,0,100);
            uint32_t duty_cycle = map(brightness_percent,0,100,0,1023);
            if (_invert)
                duty_cycle = 1023 - duty_cycle;
            ledcWrite(_pin,duty_cycle);
        }
};


#pragma endregion
//==============================================================================
#pragma region LC Display


class LcDisplay {
protected:
public:
    virtual lv_disp_t* get_disp_handle() = 0;
};


struct Config_LcDisplay_ST7789 {
    unsigned width;
    unsigned height;
    int miso;
    int mosi;
    int sclk;
    int cs;
    int dc;
    int rst;
    unsigned pclk_hz;
    esp_lcd_color_space_t color_space;
    bool color_invert;
    bool swap_xy;
    bool mirror_x;
    bool mirror_y;
};


class LcDisplay_ST7789 : public LcDisplay {
protected:
    const Config_LcDisplay_ST7789* _config;
public:
    LcDisplay_ST7789( const Config_LcDisplay_ST7789* config ) : _config(config) {}
    virtual lv_disp_t* get_disp_handle() override;
};


struct Config_LcDisplay_ST77916 {
    unsigned width;
    unsigned height;
    int sclk;
    int data0;
    int data1;
    int data2;
    int data3;
    int cs;
    int rst;
    esp_lcd_color_space_t color_space;
    bool color_invert;
    bool swap_xy;
    bool mirror_x;
    bool mirror_y;
};


class LcDisplay_ST77916 : public LcDisplay {
protected:
    const Config_LcDisplay_ST77916* _config;
public:
    LcDisplay_ST77916( const Config_LcDisplay_ST77916* config ) : _config(config) {}
    virtual lv_disp_t* get_disp_handle() override;
};


#pragma endregion
//==============================================================================
#pragma region Top-level EspBoard classes

/**
 * @brief Abstract base class for ESP boards with common functionality
 * 
 */
class EspBoard {
protected:
    LcBacklight* _backlight = nullptr;
    LcDisplay* _display = nullptr;

public:

    EspBoard() {}

    /// initialize board-specific hardware
    virtual bool init() { 
        if (_backlight) _backlight->init(); 
        return true; 
    }
    bool hasDisplay()  { return _display != NULL; }

    /// set LCD backlight brightness, in percent
    virtual void set_brightness( int percent ) 
        { if (_backlight) _backlight->set_brightness(percent); }
    /// initialize LCD driver, return display hanlde for LVGL
    virtual lv_disp_t* get_disp_handle() 
        { if (_display) return _display->get_disp_handle(); else return NULL; }
};


#pragma endregion
