/**
 * @file        AudioCodec_Direct.h
 * @project     voice control projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2026-08-03
 * tabsize      4
 * 
 * This Revision: $Id: AudioCodec_Direct.h 2011 2026-08-15 10:12:25Z  $
 */

/*
   Copyright (C) 2026 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. 
   If a copy of the MPL was not distributed with this file, You can obtain one at 
   http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief Implementation of audio codec control specific to 
 * custom device with 2x INMP441-style digital mic, and MAX98357A mono DAC 
 */

 #pragma once
#include "EspBoard.h"
#include "AudioCodec.h"

/**
 * @brief No audio codec, direct digital mic INMP441 and amplifier MAX98357A
 * 
 */
class AudioCodec_Direct : public AudioCodec {
protected:
    // using 2x INMP441 digital mic
    static const gpio_num_t _RX_DIN = (gpio_num_t) 15;
    static const gpio_num_t _RX_BCK = (gpio_num_t) 16;
    static const gpio_num_t _RX_LRCK = (gpio_num_t) 17;
    // using MAX98357A amplifier
    static const gpio_num_t _TX_DOUT = (gpio_num_t) 40;
    static const gpio_num_t _TX_BCK  = (gpio_num_t) 39;
    static const gpio_num_t _TX_LRCK = (gpio_num_t) 38;
    static const gpio_num_t _MCLK    = (gpio_num_t) -1;
    static const gpio_num_t _PA_CTRL = (gpio_num_t) -1;

public:
    AudioCodec_Direct()  {}
    /**
     * @brief Initialize RX and TX codec
     * 
     * @param mic_sample_rate    RX sample rate [Hz]
     * @param spk_sample_rate    TX sample rate [Hz] 
     * @return true if ok, false if init failed
     */   
    bool init( 
        unsigned mic_sample_rate,            
        unsigned spk_sample_rate             
    ) override;

    virtual void enableMic() override;
    virtual void enableSpk() override;

    virtual int audio_read(int16_t* dest, int samples) override;
    virtual int audio_write(const int16_t* data, int samples) override;
};

