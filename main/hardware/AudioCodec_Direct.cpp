/**
 * @file        AudioCodec_Direct.cpp
 * @project     voice control projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2026-08-03
 * tabsize      4
 * 
 * This Revision: $Id: AudioCodec_Direct.cpp 2011 2026-08-15 10:12:25Z  $
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
 * 
 * The INMP441 digital microphone _must_ be operated with 32 bit words, not 16!
 * See datasheet: "There must be 64 SCK cycles in each WS stereo frame"
 */

/*
    Gain calculation:
    Microphones are INMP441 digital MEMS
    - sensitivity is -26 dBFS dBV @ 94 dB SPL
    - max input is 120 dB SPL
    No ADC, no analog gain
    For desired max input of 90 dB SPL,
    - required total gain is 30 dB
    - 30 dB digital (bit shift of I2S input)
*/

#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "dummy_codec.h"

#include "AudioCodec_Direct.h"

#define RX_HW_BITS  I2S_DATA_BIT_WIDTH_32BIT    // word length on I2S bus
#define RX_SW_BITS  I2S_DATA_BIT_WIDTH_16BIT    // word length in app buffers

#define TAG "AudioCodec_Direct"
//------------------------------------------------------------------------------

bool AudioCodec_Direct::init( 
    unsigned mic_sample_rate, 
    unsigned spk_sample_rate 
)
{
    _mic_sample_rate = mic_sample_rate;
    _spk_sample_rate = spk_sample_rate;
    _mic_channels = 2;

    //----- Setup I2S channel to amp -------------------------------------------
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK_RETURN_FALSE(i2s_new_channel(&tx_chan_cfg, &_spk_handle, NULL));

    i2s_std_config_t tx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_spk_sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = GPIO_NUM_NC,
            .bclk = _TX_BCK,
            .ws = _TX_LRCK,
            .dout = _TX_DOUT,
            .din = GPIO_NUM_NC,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };
    ESP_ERROR_CHECK_RETURN_FALSE(i2s_channel_init_std_mode(_spk_handle, &tx_cfg));

    audio_codec_i2s_cfg_t tx_i2s_cfg = {
        .port = tx_chan_cfg.id,
        .rx_handle = NULL,
        .tx_handle = _spk_handle,
    };
    auto tx_data_if = audio_codec_new_i2s_data(&tx_i2s_cfg);
    IF_NULL_RETURN_FALSE(tx_data_if);

    esp_codec_dev_cfg_t tx_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = NULL, 
        .data_if  = tx_data_if,
    };
    _spk_dev = esp_codec_dev_new(&tx_dev_cfg);
    IF_NULL_RETURN_FALSE(_spk_dev);

    //----- Setup I2S channel from digital mics --------------------------------
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK_RETURN_FALSE(i2s_new_channel(&rx_chan_cfg, NULL, &_mic_handle));

    i2s_std_config_t rx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_mic_sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(RX_HW_BITS, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = GPIO_NUM_NC,
            .bclk = _RX_BCK,
            .ws = _RX_LRCK,
            .dout = GPIO_NUM_NC,
            .din = _RX_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };
    ESP_ERROR_CHECK_RETURN_FALSE(i2s_channel_init_std_mode(_mic_handle, &rx_cfg));

    audio_codec_i2s_cfg_t rx_i2s_cfg = {
        .port = rx_chan_cfg.id,
        .rx_handle = _mic_handle,
        .tx_handle = NULL,
    };
    auto rx_data_if = audio_codec_new_i2s_data(&rx_i2s_cfg);
    IF_NULL_RETURN_FALSE(rx_data_if);

    esp_codec_dev_cfg_t rx_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = NULL, 
        .data_if  = rx_data_if,
    };
    _mic_dev = esp_codec_dev_new(&rx_dev_cfg);
    IF_NULL_RETURN_FALSE(_mic_dev);

    ESP_LOGI(TAG, "AudioCodec_Direct initialized");
    return true;
}

//------------------------------------------------------------------------------

void AudioCodec_Direct::enableMic()
{
    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = RX_HW_BITS,
        .channel = 2,
        .channel_mask = 3,
        .sample_rate = _mic_sample_rate,
        .mclk_multiple = 0,
    };
    ESP_ERROR_CHECK(esp_codec_dev_open(_mic_dev, &fs));
}

//------------------------------------------------------------------------------

void AudioCodec_Direct::enableSpk()
{
    // Play 16bit 1 channel
    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel = (uint8_t) _spk_channels,
        .channel_mask = 0,
        .sample_rate = _spk_sample_rate,
        .mclk_multiple = 0,
    };
    ESP_ERROR_CHECK(esp_codec_dev_open(_spk_dev, &fs));
}


static void resample_32_to_16( int32_t* src, int16_t* dst, size_t nsamples, int nchannels )
{
    nsamples *= nchannels;
    while (nsamples--)
        *dst++ = *src++ >> 16;
}


int AudioCodec_Direct::audio_read(int16_t* dest, int samples)
{
    size_t bytes_read;
    static int32_t *rx_buf=NULL;
    static size_t rx_buf_size=0;

    if (RX_SW_BITS==I2S_DATA_BIT_WIDTH_16BIT && RX_HW_BITS==I2S_DATA_BIT_WIDTH_32BIT)
    {
        size_t required_rx_buf_size = samples * sizeof(int32_t) * _mic_channels;
        if (rx_buf_size != required_rx_buf_size) {
            free(rx_buf);
            rx_buf = (int32_t*) malloc( required_rx_buf_size );
            rx_buf_size = required_rx_buf_size;
            ESP_LOGD(TAG,"Allocated %u bytes for I2S RX buffer", required_rx_buf_size);
        }
        if (i2s_channel_read(
            _mic_handle, 
            rx_buf, 
            samples * sizeof(int32_t) * _mic_channels, 
            &bytes_read, 
            portMAX_DELAY
        ) != ESP_OK) {
            ESP_LOGE(TAG, "Read Failed!");
            return 0;
        }    
        size_t samples_read = bytes_read / (sizeof(int32_t) * _mic_channels);
        resample_32_to_16( rx_buf, dest, samples_read, _mic_channels );
        return samples_read;
    } else {
        if (i2s_channel_read(
            _mic_handle, 
            dest, 
            samples * sizeof(int16_t) * _mic_channels, 
            &bytes_read, 
            portMAX_DELAY
        ) != ESP_OK) {
            ESP_LOGE(TAG, "Read Failed!");
            return 0;
        }    
        return bytes_read / (sizeof(int16_t) * _mic_channels);
    }
}


int AudioCodec_Direct::audio_write(const int16_t* data, int samples)
{
    size_t bytes_written;
    ESP_ERROR_CHECK(
        i2s_channel_write(
            _spk_handle, 
            data, 
            samples * sizeof(int16_t) * _spk_channels, 
            &bytes_written, 
            portMAX_DELAY
        )
    );
    return bytes_written / sizeof(int16_t);
}
