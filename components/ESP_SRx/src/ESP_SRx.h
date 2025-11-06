/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#pragma once
#include "sdkconfig.h"
//#if CONFIG_IDF_TARGET_ESP32S3 && (CONFIG_USE_WAKENET || CONFIG_USE_MULTINET)
#if (CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4) && (CONFIG_MODEL_IN_FLASH || CONFIG_MODEL_IN_SDCARD)

#include "ESP_I2S.h"
#include "esp32-hal-srx.h"

typedef void (*srx_cb)(sr_event_t event, int command_id, int phrase_id);

class ESP_SRx_Class {
private:
  srx_cb cb;
  I2SClass *i2s;

public:
  ESP_SRx_Class();
  ~ESP_SRx_Class();

  void onEvent(srx_cb cb);
  bool begin(
    I2SClass &i2s, 
    const sr_cmd_t *sr_commands, 
    size_t sr_commands_len, 
    sr_channels_t rx_chan = SR_CHANNELS_STEREO, 
    sr_mode_t mode = SR_MODE_WAKEWORD, 
    const char *input_format = "MN"
  );
  bool end(void);
  bool setMode(sr_mode_t mode);
  bool pause(void);
  bool resume(void);

  void _srx_event(sr_event_t event, int command_id, int phrase_id);
  esp_err_t _fill(void *out, size_t len, size_t *bytes_read, uint32_t timeout_ms);
};

extern ESP_SRx_Class ESP_SRx;

#endif  // CONFIG_IDF_TARGET_ESP32S3
