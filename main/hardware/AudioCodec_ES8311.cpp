/**
 * @file        AudioCodec_ES8311.cpp
 * @project     voice control projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2026-08-03
 * tabsize      4
 * 
 * This Revision: $Id: AudioCodec_ES8311.cpp 2011 2026-08-15 10:12:25Z  $
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

/*
    Gain calculation:
    Microphone is AOS3729A analog MEMS
    - sensitivity is -42 dBV @ 94 dB SPL
    - max input is 125 dB SPL
    ADC is ES8311
    - full scale input is 1.0 V
    - gain range is 0-42 dB in 6 dB steps
    For desired max input of 90 dB SPL,
    - required total gain is 46 dB
    - split into 36 dB analog and 6 dB digital (bit shift of I2S input)
*/

#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include <Arduino.h>

#include "AudioCodec_ES8311.h"


#define TAG "AudioCodec_ES8311"

/*
    Couldn't get ESP-SR to work with a mono microphone signal, 
    so we convert the single mic signal to fake stereo, by duplicating L to R
*/
#define HW_MIC_CHANNELS 1   // mono/stereo on I2S bus
#define SW_MIC_CHANNELS 2   // mono/stereo in data sent to app


bool AudioCodec_ES8311::init( 
    unsigned mic_sample_rate, 
    unsigned spk_sample_rate 
)
{
    if (mic_sample_rate != spk_sample_rate) return false;

    if (NULL == _i2c_bus) init_I2C();
    
    _mic_sample_rate = mic_sample_rate;
    _spk_sample_rate = spk_sample_rate;
    _mic_channels = HW_MIC_CHANNELS;
    _spk_channels = 1;

    //----- create duplex channels
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK_RETURN_FALSE(i2s_new_channel(&chan_cfg, &_spk_handle, &_mic_handle));

    //----- configuration for input and output channel
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_mic_sample_rate),        
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, 
                        (HW_MIC_CHANNELS==1) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO
                    ),
        .gpio_cfg = {
            .mclk = (gpio_num_t) _config->mclk,
            .bclk = (gpio_num_t) _config->bclk,
            .ws = (gpio_num_t) _config->lrck,
            .dout = (gpio_num_t) _config->dout,
            .din = (gpio_num_t) _config->din,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    ESP_ERROR_CHECK_RETURN_FALSE(i2s_channel_init_std_mode(_spk_handle, &std_cfg));
    ESP_ERROR_CHECK_RETURN_FALSE(i2s_channel_init_std_mode(_mic_handle, &std_cfg));

    //----- I2S codec data stream
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = (uint8_t) _config->i2s_num,
        .rx_handle = _mic_handle,
        .tx_handle = _spk_handle,
    };
    auto data_if = audio_codec_new_i2s_data(&i2s_cfg);
    IF_NULL_RETURN_FALSE(data_if);

    //----- I2C control of codec 
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = (uint8_t) _config->i2c_num,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = _i2c_bus,
    };
    auto ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    IF_NULL_RETURN_FALSE(ctrl_if);

    //----- GPIO control, e.g. for power amplifier
    auto gpio_if = audio_codec_new_gpio();
    IF_NULL_RETURN_FALSE(gpio_if);

    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .pa_pin = (gpio_num_t) _config->pa_ctrl,
        .use_mclk = true,
        .hw_gain = {
            .pa_voltage = 1.0,
            .codec_dac_voltage = 1.0,
        },
    };
    auto codec_if = es8311_codec_new(&es8311_cfg);
    IF_NULL_RETURN_FALSE(codec_if);

    //----- create rx and tx device
    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = codec_if,
        .data_if = data_if,
    };
    _spk_dev = esp_codec_dev_new(&dev_cfg);
    IF_NULL_RETURN_FALSE(_spk_dev);
    dev_cfg.dev_type = ESP_CODEC_DEV_TYPE_IN;
    _mic_dev = esp_codec_dev_new(&dev_cfg);
    IF_NULL_RETURN_FALSE(_mic_dev);
    esp_codec_set_disable_when_closed(_spk_dev, false);
    esp_codec_set_disable_when_closed(_mic_dev, false);
    
    ESP_LOGI(TAG, "AudioCodec_Es8311 initialized");
    return true;
}


void AudioCodec_ES8311::enableMic()
{
    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel = HW_MIC_CHANNELS,
        .channel_mask = 0,
        .sample_rate = (uint32_t)_mic_sample_rate,
        .mclk_multiple = 0,
    };
    for (int i = 0;i < fs.channel; i++) {
        fs.channel_mask |= ESP_CODEC_DEV_MAKE_CHANNEL_MASK(i);
    }
    ESP_ERROR_CHECK(esp_codec_dev_open(_mic_dev, &fs));
    ESP_ERROR_CHECK(esp_codec_dev_set_in_gain(_mic_dev, _config->mic_gain));  
}


void AudioCodec_ES8311::enableSpk()
{
    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel = (uint8_t) _spk_channels,
        .channel_mask = 0,
        .sample_rate = (uint32_t)_spk_sample_rate,
        .mclk_multiple = 0,
    };
    ESP_ERROR_CHECK(esp_codec_dev_open(_spk_dev, &fs));
    ESP_ERROR_CHECK(esp_codec_dev_set_out_vol(_spk_dev, _config->volume));
}


static void mono_to_stereo( int16_t* dest, int samples )
{
    int16_t* rp = dest + samples;
    int16_t* wp = dest + samples*2;
    for (int i=0; i<samples; i++) {
        *--wp = *rp;
        *--wp = *--rp;
    }
}


int AudioCodec_ES8311::audio_read(int16_t* dest, int samples)
{
    size_t bytes_read;

    if (i2s_channel_read(
        _mic_handle, 
        dest, 
        samples * sizeof(int16_t) * HW_MIC_CHANNELS, 
        &bytes_read, 
        portMAX_DELAY
    ) != ESP_OK) {
        ESP_LOGE(TAG, "Read Failed!");
        return 0;
    }    
    // fake stereo: copy left to right
    size_t samples_read = bytes_read / (sizeof(int16_t) * HW_MIC_CHANNELS);
    if (HW_MIC_CHANNELS==1 && SW_MIC_CHANNELS==2) {
        mono_to_stereo( dest, samples_read );
    }
    return samples_read;
}


int AudioCodec_ES8311::audio_write(const int16_t* data, int samples)
{
    size_t bytes_written;
    size_t bytes_to_write = samples * sizeof(int16_t) * _spk_channels;
    ESP_ERROR_CHECK(
        i2s_channel_write( _spk_handle, data, bytes_to_write, &bytes_written, portMAX_DELAY )
    );
    size_t samples_written = bytes_written / (sizeof(int16_t) * _spk_channels);
    return samples_written;
}
