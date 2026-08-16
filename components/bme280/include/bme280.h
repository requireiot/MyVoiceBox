/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

typedef enum {
    BME280_SAMPLING_NONE = 0b000,
    BME280_SAMPLING_X1 = 0b001,
    BME280_SAMPLING_X2 = 0b010,
    BME280_SAMPLING_X4 = 0b011,
    BME280_SAMPLING_X8 = 0b100,
    BME280_SAMPLING_X16 = 0b101
} bme280_sensor_sampling;

typedef enum {
    BME280_MODE_SLEEP = 0b00,
    BME280_MODE_FORCED = 0b01,
    BME280_MODE_NORMAL = 0b11
} bme280_sensor_mode;

typedef enum {
    BME280_FILTER_OFF = 0b000,
    BME280_FILTER_X2 = 0b001,
    BME280_FILTER_X4 = 0b010,
    BME280_FILTER_X8 = 0b011,
    BME280_FILTER_X16 = 0b100
} bme280_sensor_filter;

// standby durations in ms
typedef enum {
    BME280_STANDBY_MS_0_5 = 0b000,
    BME280_STANDBY_MS_10 = 0b110,
    BME280_STANDBY_MS_20 = 0b111,
    BME280_STANDBY_MS_62_5 = 0b001,
    BME280_STANDBY_MS_125 = 0b010,
    BME280_STANDBY_MS_250 = 0b011,
    BME280_STANDBY_MS_500 = 0b100,
    BME280_STANDBY_MS_1000 = 0b101
} bme280_standby_duration;

typedef void *bme280_handle_t; /*handle of bme280*/

#ifdef __cplusplus
extern "C"
{
#endif

bme280_handle_t bme280_create(uint8_t i2c_port, uint8_t dev_addr);

esp_err_t bme280_set_sampling(
    bme280_handle_t sensor, bme280_sensor_mode mode,
    bme280_sensor_sampling tempsampling,
    bme280_sensor_sampling presssampling,
    bme280_sensor_sampling humsampling, bme280_sensor_filter filter,
    bme280_standby_duration duration);

esp_err_t bme280_default_init(bme280_handle_t sensor);

esp_err_t bme280_take_forced_measurement(bme280_handle_t sensor);

esp_err_t bme280_read_temperature(bme280_handle_t sensor, float *temperature);

esp_err_t bme280_read_pressure(bme280_handle_t sensor, float *pressure);

esp_err_t bme280_read_humidity(bme280_handle_t sensor, float *humidity);

esp_err_t bme280_read_climate(
    bme280_handle_t sensor, 
    float* temperature, 
    float* humidity, 
    float* pressure 
);

esp_err_t bme280_read_altitude(bme280_handle_t sensor, float seaLevel, float *altitude);

esp_err_t bme280_calculates_pressure(bme280_handle_t sensor, float altitude,
                                     float atmospheric, float *pressure);

#ifdef __cplusplus
}
#endif