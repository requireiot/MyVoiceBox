/**
 * @file        AudioCodec_Lite.h
 * @project     voice control projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2026-08-03
 * tabsize      4
 * 
 * This Revision: $Id: AudioCodec_Lite.h 2011 2026-08-15 10:12:25Z  $
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
 * ESP32-S3-BOX-Lite device
 */

#pragma once
#include "EspBoard.h"
#include "AudioCodec.h"

/**
 * @brief Audio codecs in ESP32-S3-BOX-Lite
 * 
 */
class AudioCodec_BoxLite : public AudioCodec {
protected:
    // I2C for codec control
    static const int _I2C_PORT_CODEC = I2C_NUM_1;
    static const gpio_num_t _PIN_I2C_SDA_CODEC = (gpio_num_t) 8;
    static const gpio_num_t _PIN_I2C_SCL_CODEC = (gpio_num_t) 18;
    // I2S
    static const int _I2S_PORT = I2S_NUM_1;
    // using ES7243E stereo ADC
    static const gpio_num_t _RX_DIN  = (gpio_num_t) 16;
    static const gpio_num_t _RX_BCK  = (gpio_num_t) 17;
    static const gpio_num_t _RX_LRCK = (gpio_num_t) 47;
    // using ES8156 stereo DAC
    static const gpio_num_t _TX_DOUT = (gpio_num_t) 15;
    static const gpio_num_t _TX_BCK  = (gpio_num_t) 17;
    static const gpio_num_t _TX_LRCK = (gpio_num_t) 47;
    static const gpio_num_t _MCLK    = (gpio_num_t) 2;
    static const gpio_num_t _PA_CTRL = (gpio_num_t) 46;

public:
    AudioCodec_BoxLite()  {}
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
    { return _init_I2C( _I2C_PORT_CODEC, _PIN_I2C_SDA_CODEC, _PIN_I2C_SCL_CODEC ); }

    virtual void enableMic() override;
    virtual void enableSpk() override;
};
