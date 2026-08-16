/**
 * @file        AudioCodec_Lite.cpp
 * @project     voice control projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2026-08-03
 * tabsize      4
 * 
 * This Revision: $Id: AudioCodec_Lite.cpp 2011 2026-08-15 10:12:25Z  $
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

/*
    Gain calculation:
    Microphones are MSM381A3729H9BPC analog MEMS
    - sensitivity is -38 dBV @ 94 dB SPL
    - max input is 123 dB SPL
    ADC is ES7243E
    - full scale input is 1.0 V
    - gain range is 0-37.5 dB in 3 dB steps
    For desired max input of 90 dB SPL,
    - required total gain is 42 dB
    - split into 36 dB analog and 6 dB digital (bit shift of I2S input)
*/

#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "dummy_codec.h"
#include "error_checks.h"

#include "AudioCodec_Lite.h"


#define TAG "AudioCodec_Lite"


//------------------------------------------------------------------------------

bool AudioCodec_BoxLite::init( 
    unsigned mic_sample_rate, 
    unsigned spk_sample_rate 
)
{
    if (mic_sample_rate != spk_sample_rate) return false;

    if (NULL == _i2c_bus) init_I2C();
    
    _mic_sample_rate = mic_sample_rate;
    _spk_sample_rate = spk_sample_rate;
    _mic_channels = 2;

    //----- Setup I2S peripheral
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK_RETURN_FALSE(i2s_new_channel(&chan_cfg, &_spk_handle, &_mic_handle));

    //----- Setup I2S channel to ES8156 codec
    i2s_std_config_t tx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_spk_sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = _MCLK,
            .bclk = _TX_BCK,
            .ws = _TX_LRCK,
            .dout = _TX_DOUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };
    tx_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    tx_cfg.slot_cfg.ws_width = I2S_SLOT_BIT_WIDTH_32BIT;
    ESP_ERROR_CHECK_RETURN_FALSE(i2s_channel_init_std_mode(_spk_handle, &tx_cfg));

    //----- Setup I2S channel from ES7243E codec
    i2s_std_config_t rx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_mic_sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = _MCLK,
            .bclk = _RX_BCK,
            .ws = _RX_LRCK,
            .dout = I2S_GPIO_UNUSED,
            .din = _RX_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };
    rx_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    rx_cfg.slot_cfg.ws_width = I2S_SLOT_BIT_WIDTH_32BIT;
    ESP_ERROR_CHECK_RETURN_FALSE(i2s_channel_init_std_mode(_mic_handle, &rx_cfg));

    // Do initialize of related interface: data_if, ctrl_if and gpio_if
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = _I2S_PORT,
        .rx_handle = _mic_handle,
        .tx_handle = _spk_handle,
    };
    auto data_if = audio_codec_new_i2s_data(&i2s_cfg);
    IF_NULL_RETURN_FALSE(data_if);

    //----- Output
    audio_codec_i2c_cfg_t tx_i2c_cfg = {
        .port = _I2C_PORT_CODEC,
        .addr = ES8156_CODEC_DEFAULT_ADDR,
        .bus_handle = _i2c_bus,
    };
    auto out_ctrl_if = audio_codec_new_i2c_ctrl(&tx_i2c_cfg);
    IF_NULL_RETURN_FALSE(out_ctrl_if);

    auto gpio_if = audio_codec_new_gpio();
    IF_NULL_RETURN_FALSE(gpio_if);

    es8156_codec_cfg_t cfg = {
        .ctrl_if = out_ctrl_if,
        .gpio_if = gpio_if,
        .pa_pin = _PA_CTRL,
        .hw_gain = { 
            .pa_voltage = 5.0,
            .codec_dac_voltage = 3.3,
        },
    };
    auto out_codec_if = es8156_codec_new(&cfg);
    IF_NULL_RETURN_FALSE(out_codec_if);

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = out_codec_if,
        .data_if  = data_if,
    };
    _spk_dev = esp_codec_dev_new(&dev_cfg);
    IF_NULL_RETURN_FALSE(_spk_dev);

    //----- Input
    audio_codec_i2c_cfg_t rx_i2c_cfg = {
        .port = _I2C_PORT_CODEC,
        .addr = ES7243E_CODEC_DEFAULT_ADDR,
        .bus_handle = _i2c_bus,
    };
    auto in_ctrl_if =  audio_codec_new_i2c_ctrl(&rx_i2c_cfg);
    IF_NULL_RETURN_FALSE(in_ctrl_if);

    es7243e_codec_cfg_t es7243_cfg = {
        .ctrl_if = in_ctrl_if,
    };
    auto in_codec_if = es7243e_codec_new(&es7243_cfg);
    IF_NULL_RETURN_FALSE(in_codec_if);

    esp_codec_dev_cfg_t codec_es7243_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = in_codec_if,
        .data_if = data_if,
    };

    _mic_dev = esp_codec_dev_new(&codec_es7243_dev_cfg);
    IF_NULL_RETURN_FALSE(_mic_dev);

    ESP_LOGD(TAG, "AudioCodec_BoxLite initialized");
    return true;
}

//------------------------------------------------------------------------------

void AudioCodec_BoxLite::enableMic()
{
    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel = (uint8_t) _mic_channels,
        .channel_mask = 0,
        .sample_rate = _mic_sample_rate,
        .mclk_multiple = 0,
    };
    for (int i = 0;i < fs.channel; i++) {
        fs.channel_mask |= ESP_CODEC_DEV_MAKE_CHANNEL_MASK(i);
    }
    ESP_ERROR_CHECK(esp_codec_dev_open(_mic_dev, &fs));
    ESP_ERROR_CHECK(esp_codec_dev_set_in_gain(_mic_dev, 36.0));  // was 37.5
}

//------------------------------------------------------------------------------

void AudioCodec_BoxLite::enableSpk()
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
    ESP_ERROR_CHECK(esp_codec_dev_set_out_vol(_spk_dev, 90.0));
}
