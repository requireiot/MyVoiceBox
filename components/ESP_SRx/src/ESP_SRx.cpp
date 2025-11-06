/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "sdkconfig.h"
//#if CONFIG_IDF_TARGET_ESP32S3 && (CONFIG_USE_WAKENET || CONFIG_USE_MULTINET)
#if (CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4) && (CONFIG_MODEL_IN_FLASH || CONFIG_MODEL_IN_SDCARD)
#include "ESP_SRx.h"

static esp_err_t on_srx_fill(void *arg, void *out, size_t len, size_t *bytes_read, uint32_t timeout_ms) {
  return ((ESP_SRx_Class *)arg)->_fill(out, len, bytes_read, timeout_ms);
}

static void on_srx_event(void *arg, sr_event_t event, int command_id, int phrase_id) {
  ((ESP_SRx_Class *)arg)->_srx_event(event, command_id, phrase_id);
}

ESP_SRx_Class::ESP_SRx_Class() : cb(NULL), i2s(NULL) {}

ESP_SRx_Class::~ESP_SRx_Class() {
  end();
}

void ESP_SRx_Class::onEvent(srx_cb event_cb) {
  cb = event_cb;
}

bool ESP_SRx_Class::begin(
  I2SClass &_i2s, 
  const sr_cmd_t *sr_commands, 
  size_t sr_commands_len, 
  sr_channels_t rx_chan, 
  sr_mode_t mode,
  const char *input_format
) {
  i2s = &_i2s;
  esp_err_t err = srx_start(on_srx_fill, this, rx_chan, mode, input_format, sr_commands, sr_commands_len, on_srx_event, this);
  return (err == ESP_OK);
}

bool ESP_SRx_Class::end(void) {
  return srx_stop() == ESP_OK;
}

bool ESP_SRx_Class::setMode(sr_mode_t mode) {
  return srx_set_mode(mode) == ESP_OK;
}

bool ESP_SRx_Class::pause(void) {
  return srx_pause() == ESP_OK;
}

bool ESP_SRx_Class::resume(void) {
  return srx_resume() == ESP_OK;
}

void ESP_SRx_Class::_srx_event(sr_event_t event, int command_id, int phrase_id) {
  if (cb) {
    cb(event, command_id, phrase_id);
  }
}

esp_err_t ESP_SRx_Class::_fill(void *out, size_t len, size_t *bytes_read, uint32_t timeout_ms) {
  if (i2s == NULL) {
    return ESP_FAIL;
  }
  i2s->setTimeout(timeout_ms);
  *bytes_read = i2s->readBytes((char *)out, len);
  return (esp_err_t)i2s->lastError();
}

ESP_SRx_Class ESP_SRx;

#endif  // CONFIG_IDF_TARGET_ESP32S3
