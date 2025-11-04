#pragma once
/**
 * @file        StatusLED.h
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

#include <Arduino.h>

struct StatusLED {
    static const unsigned MS_PER_TICK = 10;
    static const unsigned MS_PER_BLINK = 1000;      /// blink period [ms]
    static const unsigned MS_PER_FLASH = 100;       /// flash on duration [ms]
    static const unsigned MS_PER_BREATHE = 2000;    /// breathe cycle duration [ms]

    enum mode_t { MODE_OFF, MODE_ON, MODE_BREATHE, MODE_BLINK, MODE_FLASH } _mode;
    int _pinA;              /// GPIO pin number for common anode
    int _pinR;              /// GPIO pin number for red LED cathode
    int _pinG;              /// GPIO pin number for green LED cathode
    int _pinB;              /// GPIO pin number for blue LED cathode
    uint8_t _brightness;    /// perceived brightness [0..255]
    uint16_t _duty;         /// current PWM duty cyle [0..4095]
    int _dir;
    int _count;             /// tick counter for blink, flash, breathe

    StatusLED( int pinA, int pinR, int pinG, int pinB );

    void init();
    void tick();

    uint16_t brightness_to_duty(uint8_t br);
    void set_duty( uint16_t duty );
    void set_brightness( uint8_t br );
    void set_color( uint8_t r, uint8_t g, uint8_t b );
    void set_mode( mode_t mode );
};