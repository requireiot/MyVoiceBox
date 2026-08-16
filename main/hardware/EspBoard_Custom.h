/**
 * @file        EspBoard_Custom.h
 * @project     voice control projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2026-08-03
 * tabsize      4
 * 
 * This Revision: $Id: EspBoard_Custom.h 2006 2026-08-12 18:59:44Z  $
 */

/*
   Copyright (C) 2026 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. 
   If a copy of the MPL was not distributed with this file, You can obtain one at 
   http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

#pragma once
#include "EspBoard.h"
#include "AudioCodec_Direct.h"

/**
 * @brief Board features with custom I2S hardware
 * 
 */
class EspBoard_Custom : public EspBoard {
public:
    EspBoard_Custom() {}
};


/**
 * @brief Board features with custom I2S hardware and LCD
 * 
 */
class EspBoard_Custom_LCD : public EspBoard_Custom {
public:
    EspBoard_Custom_LCD();
};


/**
 * @brief Board features with custom I2S hardware and round LCD
 * 
 */
class EspBoard_Custom_RoundLCD : public EspBoard_Custom {
public:
    EspBoard_Custom_RoundLCD();
};
