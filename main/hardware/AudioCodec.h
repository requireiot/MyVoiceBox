/**
 * @file        AudioCodec.h
 * @project     voice control projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2026-08-03
 * tabsize      4
 * 
 * This Revision: $Id: $
 */

/*
   Copyright (C) 2026 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. 
   If a copy of the MPL was not distributed with this file, You can obtain one at 
   http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief Implementation of audio codec control, generic base class
 */
#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include <driver/i2c_master.h>
#include <driver/i2s_std.h>
#include <esp_codec_dev.h>
#include <esp_codec_dev_defaults.h>
#include "error_checks.h"

/**
 * @brief Abstract base class for I2S audio codecs (rx and tx)
 * 
 */
class AudioCodec
{
protected:
    i2c_master_bus_handle_t _i2c_bus = nullptr;
    i2c_port_num_t _i2c_port;

    // handles to use with i2s_channel_read(), i2s_channel_write()
    i2s_chan_handle_t _mic_handle = nullptr; 
    i2s_chan_handle_t _spk_handle = nullptr;

    // handles to // use with esp_codec_dev_read(), esp_codec_dev_write()
    esp_codec_dev_handle_t _mic_dev = nullptr;   
    esp_codec_dev_handle_t _spk_dev = nullptr;

    int _mic_channels = 1;
    int _spk_channels = 1;

    unsigned _mic_sample_rate;
    unsigned _spk_sample_rate;

    esp_err_t _init_I2C( int port, int sda, int scl );
public:
    bool start();

    /**
     * @brief Initialize RX and TX codec
     * 
     * @param i2c_handle        i2c handle from `EspBoard::get_codec_i2c()`
     * @param mic_sample_rate    RX sample rate [Hz]
     * @param spk_sample_rate    TX sample rate [Hz] 
     * @return true if ok, false if init failed
     */   
    virtual bool init( 
        unsigned mic_sample_rate,
        unsigned spk_sample_rate 
    ) = 0;
    /// initialize I2C port for codec control
    virtual esp_err_t init_I2C() { return ESP_OK; }
    void* get_i2c_handle() { return _i2c_bus; }

    virtual void enableMic() = 0;
    virtual void enableSpk() = 0;

    /**
     * @brief Read samples from RX codec.
     * Default implementation calls esp_codec_dev_read(), but may be overridden
     * in derived classes
     * 
     * @param dest      destination buffer
     * @param samples   no of //samples// not //bytes//
     * @return int      no of samples read
     */
    virtual int audio_read(int16_t* dest, int samples);

    /**
     * @brief Write samples to TX codec.
     * Default implementation calls esp_codec_dev_write(), but may be overridden
     * in derived classes
     * 
     * @param data      source buffer
     * @param samples   no of //samples// not //bytes//
     * @return int      no of samples written
     */
    virtual int audio_write(const int16_t* data, int samples);
};
