/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#pragma once
#include "sdkconfig.h"
//#if CONFIG_IDF_TARGET_ESP32S3 && (CONFIG_USE_WAKENET || CONFIG_USE_MULTINET)
#if (CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4) && (CONFIG_MODEL_IN_FLASH || CONFIG_MODEL_IN_SDCARD)

#include "driver/i2s_types.h"
#include "esp_err.h"
#include "esp32-hal-sr.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t srx_start(
  sr_fill_cb fill_cb, 
  void *fill_cb_arg, 
  sr_channels_t rx_chan, 
  sr_mode_t mode, 
  const char *input_format, 
  const sr_cmd_t *sr_commands, 
  size_t cmd_number, 
  sr_event_cb cb, 
  void *cb_arg
);
esp_err_t srx_stop(void);
esp_err_t srx_pause(void);
esp_err_t srx_resume(void);
esp_err_t srx_set_mode(sr_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif  // CONFIG_IDF_TARGET_ESP32S3
