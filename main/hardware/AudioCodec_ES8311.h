/**
 * @file        AudioCodec_ES8311.h
 * @project     voice control projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2026-08-03
 * tabsize      4
 * 
 * This Revision: $Id: AudioCodec_ES8311.h 2011 2026-08-15 10:12:25Z  $
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
 * devices based on ES8311, such as Spotpear Muma
 * 
 * Specific pin numbers are defined in derived classes
 */

 #pragma once
#include "AudioCodec.h"

/**
 * @brief Pin configuration for ES8311 codec, 
 * used in classes derived from `AudioCodec_ES8311`
 * 
 */
struct Config_AudioCodec_ES8311 {
    int i2c_num;
    int scl;
    int sda;
    int i2s_num;
    int bclk;
    int lrck;
    int din;
    int dout;
    int mclk;
    int pa_ctrl;
    float mic_gain;     // in dB
    float volume;       // 0=-95dB to 255=+32dB
};


/**
 * @brief Audio codec ES8311, mono, rx and tx
 * 
 */
class AudioCodec_ES8311 : public AudioCodec {
protected:
    const Config_AudioCodec_ES8311 *_config;
public:
    AudioCodec_ES8311( const Config_AudioCodec_ES8311 *config ) 
    : _config(config) {}

    /**
     * @brief Initialize RX and TX codec
     * 
     * @param i2c_handle        i2c handle from `EspBoard::get_codec_i2c()`
     * @param mic_sample_rate    RX sample rate [Hz]
     * @param spk_sample_rate    TX sample rate [Hz] 
     * @return true if ok, false if init failed
     */   
    bool init( 
        unsigned mic_sample_rate,            
        unsigned spk_sample_rate             
    ) override;
    esp_err_t init_I2C() override
    { return _init_I2C( 1, _config->sda, _config->scl ); }
    
    virtual void enableMic() override;
    virtual void enableSpk() override;

    virtual int audio_read(int16_t* dest, int samples) override;
    virtual int audio_write(const int16_t* data, int samples) override;
};
