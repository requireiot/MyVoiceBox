/**
 * @file        EspBoard_Lite.h
 * @project     voice control projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2026-08-03
 * tabsize      4
 * 
 * This Revision: $Id: EspBoard_Lite.h 2006 2026-08-12 18:59:44Z  $
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

 #pragma once
#include "EspBoard.h"
#include "AudioCodec_Lite.h"


/**
 * @brief Board features with ESP32-S3-BOX-Lite
 * 
 */
class EspBoard_BoxLite : public EspBoard {
protected:
public:
    EspBoard_BoxLite();
};
