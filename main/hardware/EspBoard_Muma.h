/**
 * @file        EspBoard_Muma.h
 * @project     voice control projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2026-08-03
 * tabsize      4
 * 
 * This Revision: $Id: EspBoard_Muma.h 2006 2026-08-12 18:59:44Z  $
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

 #pragma once
#include "EspBoard.h"
#include "AudioCodec_ES8311.h"

/**
 * @brief Audio codec in Spotpear 1.54inch. Uses ES8311
 * 
 */
class AudioCodec_Muma : public AudioCodec_ES8311 {
protected:
    static const Config_AudioCodec_ES8311 _config;
public:
    AudioCodec_Muma() : AudioCodec_ES8311( &_config ) {}
};


/**
 * @brief Board features with Spotpear AI board with 1.54inch LCD
 * 
 */
class EspBoard_Muma : public EspBoard {
protected:
public:   
    EspBoard_Muma();
};
