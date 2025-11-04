/**
 * @file        StatusLED.cpp
 * @project     (various)
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2025-10-28
 * tabsize  4
 * 
 * This Revision: $Id: $
 */

/*
   Copyright (C) 2025 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, 
   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
   obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief Analog RGB LED with common anode connected to 4 pins. 
 * Can select the color and brightness, and various blink and breathe modes
 * 
 */

#include <Arduino.h>
#include "StatusLED.h"
#include "cie1931.h"
#define CIE_SIZE (sizeof(cie)/sizeof(cie[0]))

#define LEDC_MAX_DUTY 4095

StatusLED::StatusLED( int pinA, int pinR, int pinG, int pinB )
        : _pinA(pinA), _pinR(pinR), _pinG(pinG), _pinB(pinB) 
{ 
    init(); 
}


/**
 * @brief Convert *brightness* [0..255] to PWM *duty cycle* [0..4095]
 * 
 * @param br 
 * @return uint16_t 
 */
uint16_t StatusLED::brightness_to_duty(uint8_t br) 
{ 
    return cie[br * 256 / CIE_SIZE]; 
}


/**
 * @brief Initialize RGB LED, set all 3 LEDs to off
 * 
 */
void StatusLED::init() {
    pinMode(_pinR,OUTPUT_OPEN_DRAIN);
    pinMode(_pinG,OUTPUT_OPEN_DRAIN);
    pinMode(_pinB,OUTPUT_OPEN_DRAIN);
    digitalWrite(_pinR,HIGH);
    digitalWrite(_pinG,HIGH);
    digitalWrite(_pinB,HIGH);
    ledcSetClockSource(LEDC_AUTO_CLK);
    ledcAttach(_pinA,400,12);
}


/**
 * @brief Set PWM duty cycle [0..4095] for all LEDs that are on
 * 
 * @param duty 
 */
void StatusLED::set_duty( uint16_t duty ) 
{ 
    _duty=duty; ledcWrite(_pinA,duty); 
}


/**
 * @brief Set brightness [0..255] for all LEDs that are on
 * 
 * @param br 
 */
void StatusLED::set_brightness( uint8_t br ) 
{ 
    _brightness = br;
    set_duty( brightness_to_duty(br) ); 
}


/**
 * @brief Set color for red, green and blue component. Colors are not independent,
 * `set_color(100,0,50)` is the same as `set_color(100,0,100)
 * 
 * @param r     red brightness [0..255]
 * @param g     green brightness [0..255]
 * @param b     blue brightness [0..255]
 */
void StatusLED::set_color( uint8_t r, uint8_t g, uint8_t b ) 
{
    digitalWrite(_pinR, r ? LOW : HIGH);
    digitalWrite(_pinG, g ? LOW : HIGH);
    digitalWrite(_pinB, b ? LOW : HIGH);
    uint8_t br = max(max(r,g),b);
    set_brightness( br ); 
    log_d("set_color(%" PRIu8 ",%" PRIu8 ",%" PRIu8 ") br=%" PRIu8, r, g, b, br );
}


/**
 * @brief Set blink mode like steady on, blink, flash, breathe
 * 
 * @param mode  one of MODE_OFF, MODE_ON, MODE_BREATHE, MODE_BLINK, MODE_FLASH
 */
void StatusLED::set_mode( mode_t mode ) { 
    _mode=mode; 
    _count=0; 
    _dir=1;
}


/**
 * @brief Update brightness according to set blink mode. Call this every
 * StatusLED::MS_PER_TICK milliseconds, from `loop()` or a task
 * 
 */
void StatusLED::tick()
{
    static constexpr unsigned TICK_PER_BLINK = MS_PER_BLINK / MS_PER_TICK;
    static constexpr unsigned TICK_PER_FLASH = MS_PER_FLASH / MS_PER_TICK;
    static constexpr unsigned TICK_PER_BREATHE = MS_PER_BREATHE / MS_PER_TICK;

    int index;

    switch (_mode) {
        case MODE_OFF:
            set_duty(0);
            break;
        case MODE_ON:
            set_duty( brightness_to_duty(_brightness));
            break;
        case MODE_BREATHE:
            _count += _dir;
            if (_count==0) {
                _dir = 1;
            } else if (_count >= TICK_PER_BREATHE) {
                _dir = -1;
            }
            index = _count - (TICK_PER_BREATHE - _brightness);
            set_duty( brightness_to_duty( (index < 0) ? 0 : index ));
            break;
        case MODE_BLINK:
            _count++;
            if (_count >= TICK_PER_BLINK) _count=0;
            set_duty( brightness_to_duty( (_count < TICK_PER_BLINK/2) ? _brightness : 0));
            break;
        case MODE_FLASH:
            _count++;
            if (_count >= TICK_PER_BLINK) _count=0;
            set_duty( brightness_to_duty( (_count < TICK_PER_FLASH) ? _brightness : 0));
            break;
    }
}
