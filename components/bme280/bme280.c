/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Modifications for compatibility with old driver/i2c.h and various cleanup
 * Copyright(C) 2025 Bernd Waldmann
 */

#include "sdkconfig.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#if CONFIG_CODEC_I2C_BACKWARD_COMPATIBLE
 #include "driver/i2c.h"
#else
 #include "driver/i2c_master.h"
#endif
#include "esp32-hal-i2c.h"
#include "bme280.h"
#include "math.h"
#include "esp_log.h"
#include "esp_check.h"

#define TAG "bme280"

//===========================================================================
#pragma region BME280 registers


#define BME280_DEFAULT_CHIPID        (0x60)

#define WRITE_BIT      I2C_MASTER_WRITE         /*!< I2C master write */
#define READ_BIT       I2C_MASTER_READ          /*!< I2C master read */
#define ACK_CHECK_EN   0x1                      /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0                      /*!< I2C master will not check ack from slave */
#define ACK_VAL        0x0                      /*!< I2C ack value */
#define NACK_VAL       0x1                      /*!< I2C nack value */

#define BME_REG_DIG_T1              0x88
#define BME_REG_DIG_T2              0x8A
#define BME_REG_DIG_T3              0x8C

#define BME_REG_DIG_P1              0x8E
#define BME_REG_DIG_P2              0x90
#define BME_REG_DIG_P3              0x92
#define BME_REG_DIG_P4              0x94
#define BME_REG_DIG_P5              0x96
#define BME_REG_DIG_P6              0x98
#define BME_REG_DIG_P7              0x9A
#define BME_REG_DIG_P8              0x9C
#define BME_REG_DIG_P9              0x9E

#define BME_REG_DIG_H1              0xA1
#define BME_REG_DIG_H2              0xE1
#define BME_REG_DIG_H3              0xE3
#define BME_REG_DIG_H4              0xE4
#define BME_REG_DIG_H5              0xE5
#define BME_REG_DIG_H6              0xE7

#define BME_REG_CHIPID              0xD0
#define BME_REG_VERSION             0xD1
#define BME_REG_SOFTRESET           0xE0

#define BME_REG_CAL26               0xE1  // R calibration stored in 0xE1-0xF0

#define BME_REG_CONTROLHUMID        0xF2
#define BME_REG_STATUS              0XF3
#define BME_REG_CONTROL             0xF4
#define BME_REG_CONFIG              0xF5
#define BME_REG_PRESSUREDATA        0xF7
#define BME_REG_TEMPDATA            0xFA
#define BME_REG_HUMIDDATA           0xFD


typedef struct {
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;

    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;

    uint8_t dig_h1;
    int16_t dig_h2;
    uint8_t dig_h3;
    int16_t dig_h4;
    int16_t dig_h5;
    int8_t dig_h6;
} bme280_coeff_t;


// The config register
typedef struct config {
    // inactive duration (standby time) in normal mode
    // 000 = 0.5 ms
    // 001 = 62.5 ms
    // 010 = 125 ms
    // 011 = 250 ms
    // 100 = 500 ms
    // 101 = 1000 ms
    // 110 = 10 ms
    // 111 = 20 ms
    unsigned int t_sb : 3;

    // filter settings
    // 000 = filter off
    // 001 = 2x filter
    // 010 = 4x filter
    // 011 = 8x filter
    // 100 and above = 16x filter
    unsigned int filter : 3;

    // unused - don't set
    unsigned int none : 1;
    unsigned int spi3w_en : 1;
} bme280_config_t;

// The ctrl_meas register
typedef struct ctrl_meas {
    // temperature oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_t : 3;

    // pressure oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_p : 3;

    // device mode
    // 00       = sleep
    // 01 or 10 = forced
    // 11       = normal
    unsigned int mode : 2;
} bme280_ctrl_meas_t;

// The ctrl_hum register
typedef struct ctrl_hum {
    // unused - don't set
    unsigned int none : 5;

    // pressure oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_h : 3;
} bme280_ctrl_hum_t;

typedef struct {
#if CONFIG_CODEC_I2C_BACKWARD_COMPATIBLE    
    uint8_t i2c_dev;
    uint8_t dev_addr;
#else
    i2c_master_dev_handle_t dev_handle;
#endif
    bme280_coeff_t coeff;
    bme280_config_t config;
    bme280_ctrl_meas_t ctrl_meas;
    bme280_ctrl_hum_t ctrl_hum;
    int32_t t_fine;
    int32_t adc_T;
    int32_t adc_P;
    int32_t adc_H;
} bme280_dev_t;

#pragma endregion
//===========================================================================
#pragma region Basic I2C transactions


#if CONFIG_CODEC_I2C_BACKWARD_COMPATIBLE    
static int i2c_write(bme280_dev_t* const device, uint8_t reg_addr, const uint8_t *reg_data, uint8_t cnt)
{
	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (device->dev_addr << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, reg_addr, true);
	i2c_master_write(cmd, reg_data, cnt, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin((i2c_port_t)device->i2c_dev, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return espRc;
}
#endif


static int i2c_write_byte(bme280_dev_t* const device, uint8_t reg_addr, uint8_t reg_data)
{ 
#if CONFIG_CODEC_I2C_BACKWARD_COMPATIBLE    
    return i2c_write(device,reg_addr,&reg_data,1); 
#else
    uint8_t tx[] = { reg_addr, reg_data };
    return i2c_master_transmit( device->dev_handle, tx, sizeof(tx), 10 );
#endif
}


static int i2c_read(bme280_dev_t* const device, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
#if CONFIG_CODEC_I2C_BACKWARD_COMPATIBLE    
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (device->dev_addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (device->dev_addr << 1) | I2C_MASTER_READ, true);

	if (cnt > 1) {
		i2c_master_read(cmd, reg_data, cnt-1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data+cnt-1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin((i2c_port_t)device->i2c_dev, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return espRc;
#else
    return i2c_master_transmit_receive(
        device->dev_handle,
        &reg_addr, 1,
        reg_data, cnt,
        10
    );
#endif
}


static int i2c_read_byte(bme280_dev_t* const device, uint8_t reg_addr, uint8_t* reg_data)
{ 
    return i2c_read(device,reg_addr,reg_data,1); 
}


#pragma endregion
//===========================================================================
#pragma region Low-level functions


/**
 * @brief   Get the value of BME280_REGISTER_CONFIG register
 * @param  sensor object handle of bme280
 * @return
 *    - unsigned int: the value of BME280_REGISTER_CONFIG register
 */
static unsigned int bme280_getconfig(bme280_handle_t sensor)
{
    bme280_dev_t *sens = (bme280_dev_t *) sensor;
    return (sens->config.t_sb << 5) | (sens->config.filter << 3) | sens->config.spi3w_en;
}

/**
 * @brief   Get the value of BME280_REGISTER_CONTROL measure register
 * @param  sensor object handle of bme280
 * @return
 *    - unsigned int the value of BME280_REGISTER_CONTROL register
 */
static unsigned int bme280_getctrl_meas(bme280_handle_t sensor)
{
    bme280_dev_t *sens = (bme280_dev_t *) sensor;
    return (sens->ctrl_meas.osrs_t << 5) | (sens->ctrl_meas.osrs_p << 3) | sens->ctrl_meas.mode;
}

/**
 * @brief   Get the value of BME280_REGISTER_CONTROLHUMID measure register
 * @param  sensor object handle of bme280
 * @return
 *    - unsigned int the value of BME280_REGISTER_CONTROLHUMID register
 */
static unsigned int bme280_getctrl_hum(bme280_handle_t sensor)
{
    bme280_dev_t *sens = (bme280_dev_t *) sensor;
    return (sens->ctrl_hum.osrs_h);
}

/**
 * @brief return true if chip is busy reading cal data
 * @param  sensor object handle of bme280
 * @return
 *    - true chip is busy
 *    - false chip is idle or wrong
 */
static bool bme280_is_reading_calibration(bme280_handle_t sensor)
{
    uint8_t rstatus = 0;
    bme280_dev_t *sens = (bme280_dev_t *) sensor;
    if (ESP_OK != i2c_read_byte(sens, BME_REG_STATUS, &rstatus)) {
        ESP_LOGW(TAG,"bme280_is_reading_calibration failed");
    }
    return (rstatus & (1 << 0)) != 0;
}

/**
 * @brief Reads the factory-set coefficients
 * @param  sensor object handle of bme280
 * @return
 *    - ESP_OK Success
 *    - ESP_FAIL Fail
 */
static esp_err_t bme280_read_coefficients(bme280_handle_t sensor)
{
    bme280_dev_t *sens = (bme280_dev_t *) sensor;

    uint8_t ca[26];
    uint8_t cb[16];

    ESP_RETURN_ON_ERROR(i2c_read(sens,BME_REG_DIG_T1,ca,sizeof(ca)),TAG,"bme280_read_coefficients part 1 failed");
    ESP_RETURN_ON_ERROR(i2c_read(sens,BME_REG_DIG_H2,cb,sizeof(cb)),TAG,"bme280_read_coefficients part 2 failed");
    ESP_LOGD(TAG,"read %d + %d calibration bytes",sizeof(ca),sizeof(cb));

#define A_U16_LE(x) (uint16_t)((uint16_t)ca[x - 0x88 + 1] << 8 | ca[x - 0x88])
#define A_S16_LE(x) (int16_t)((uint16_t)ca[x - 0x88 + 1] << 8 | ca[x - 0x88])
#define A_U8(x) ca[x - 0x88]

#define B_U16_LE(x) (uint16_t)(((uint16_t)cb[x - 0xE1 + 1] << 8 | cb[x - 0xE1]))
#define B_S16_LE(x) (int16_t)(((uint16_t)cb[x - 0xE1 + 1] << 8 | cb[x - 0xE1]))
#define B_S12_HL(x) (int16_t)((uint16_t)cb[x - 0xE1] << 4 | (cb[x - 0xE1 + 1] & 0x0F))
#define B_S12_LH(x) (int16_t)(cb[x - 0xE1] >> 4 | (uint16_t)cb[x - 0xE1 + 1] << 8)
#define B_U8(x) cb[x - 0xE1]

    sens->coeff.dig_t1 = A_U16_LE(BME_REG_DIG_T1);
    sens->coeff.dig_t2 = A_S16_LE(BME_REG_DIG_T2);
    sens->coeff.dig_t3 = A_S16_LE(BME_REG_DIG_T3);
    ESP_LOGD(TAG,"T %" PRIu16 " %" PRId16 " %" PRId16, 
        sens->coeff.dig_t1, sens->coeff.dig_t2, sens->coeff.dig_t3);

    sens->coeff.dig_p1 = A_U16_LE(BME_REG_DIG_P1);
    sens->coeff.dig_p2 = A_S16_LE(BME_REG_DIG_P2);
    sens->coeff.dig_p3 = A_S16_LE(BME_REG_DIG_P3);
    sens->coeff.dig_p4 = A_S16_LE(BME_REG_DIG_P4);
    sens->coeff.dig_p5 = A_S16_LE(BME_REG_DIG_P5);
    sens->coeff.dig_p6 = A_S16_LE(BME_REG_DIG_P6);
    sens->coeff.dig_p7 = A_S16_LE(BME_REG_DIG_P7);
    sens->coeff.dig_p8 = A_S16_LE(BME_REG_DIG_P8);
    sens->coeff.dig_p9 = A_S16_LE(BME_REG_DIG_P9);
    ESP_LOGD(TAG,"P %" PRIu16 " %" PRId16 " %" PRId16 , 
        sens->coeff.dig_p1, sens->coeff.dig_p2, sens->coeff.dig_p3);

    sens->coeff.dig_h1 = A_U8(BME_REG_DIG_H1);
    sens->coeff.dig_h2 = B_S16_LE(BME_REG_DIG_H2);
    sens->coeff.dig_h3 = B_U8(BME_REG_DIG_H3);
    sens->coeff.dig_h4 = B_S12_HL(BME_REG_DIG_H4);
    sens->coeff.dig_h5 = B_S12_LH(BME_REG_DIG_H5);
    sens->coeff.dig_h6 = (int8_t)B_U8(BME_REG_DIG_H6);
    ESP_LOGD(TAG,"H %" PRIu8 " %" PRId16 " %" PRIu8 " %" PRId16 " %" PRId16 " %" PRId8, 
        sens->coeff.dig_h1, sens->coeff.dig_h2, sens->coeff.dig_h3, 
        sens->coeff.dig_h4, sens->coeff.dig_h5, sens->coeff.dig_h6);

    return ESP_OK;
}

/**
 * @brief Read all measurement data from device, and store as "raw" T,H,P values
 * 
 * @param sensor  sensor object handle of bme280
 * @return esp_err_t 
 */
static esp_err_t bme280_read_measurements(bme280_handle_t sensor)
{
    bme280_dev_t *sens = (bme280_dev_t *) sensor;
    
    uint8_t regs[8] = { 0 };
    ESP_RETURN_ON_ERROR(i2c_read(sens,0xF7,regs,sizeof(regs)),TAG,"i2c_read fail in bme280_read_measurements()");
    ESP_LOGD(TAG,"read meas %02X %02X %02X %02X %02X %02X %02X %02X",
        regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7]
    );
    sens->adc_P = (regs[0] << 12L) | (regs[1] << 4L) | (regs[2] >> 4);
    sens->adc_T = (regs[3] << 12L) | (regs[4] << 4L) | (regs[5] >> 4);
    sens->adc_H = (regs[6] << 8L) | (regs[7]);
    return ESP_OK;
}


static esp_err_t bme280_convert_temperature(bme280_handle_t sensor, float *fTemperature)
{
    bme280_dev_t *sens = (bme280_dev_t *) sensor;

    int32_t adc_T = sens->adc_T;
    if (adc_T == 0x80000) {      // value in case temp measurement was disabled
        ESP_LOGW(TAG,"invalid temperature");
        return ESP_FAIL;
    }

    /* 
        original compensation code from Bosch, 
        https://github.com/boschsensortec/BME280_SensorAPI/blob/master/bme280.c  
    */
    int32_t var1, var2;
    int32_t temperature;
    const int32_t temperature_min = -4000;
    const int32_t temperature_max = 8500;

    var1 = (int32_t)((adc_T / 8) - ((int32_t)sens->coeff.dig_t1 * 2));
    var1 = (var1 * ((int32_t)sens->coeff.dig_t2)) / 2048;
    var2 = (int32_t)((adc_T / 16) - ((int32_t)sens->coeff.dig_t1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)sens->coeff.dig_t3)) / 16384;
    sens->t_fine = var1 + var2;
    temperature = (sens->t_fine * 5 + 128) / 256;

    if (temperature < temperature_min) {
        temperature = temperature_min;
    } else if (temperature > temperature_max) {
        temperature = temperature_max;
    }
    *fTemperature = temperature / 100.0;
    return ESP_OK;
}


static esp_err_t bme280_convert_pressure(bme280_handle_t sensor, float *fPressure)
{
    bme280_dev_t *sens = (bme280_dev_t *) sensor;
    
    int32_t adc_P = sens->adc_P;
    if (adc_P == 0x80000) {  // value in case pressure measurement was disabled
        ESP_LOGW(TAG,"invalid pressure");
        return ESP_FAIL;
    }

    /* 
        original compensation code from Bosch, 
        https://github.com/boschsensortec/BME280_SensorAPI/blob/master/bme280.c  
    */
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    uint32_t var5;
    uint32_t pressure;
    const uint32_t pressure_min = 30000;
    const uint32_t pressure_max = 110000;

    var1 = (((int32_t)sens->t_fine) / 2) - (int32_t)64000;
    var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)sens->coeff.dig_p6);
    var2 = var2 + ((var1 * ((int32_t)sens->coeff.dig_p5)) * 2);
    var2 = (var2 / 4) + (((int32_t)sens->coeff.dig_p4) * 65536);
    var3 = (sens->coeff.dig_p3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
    var4 = (((int32_t)sens->coeff.dig_p2) * var1) / 2;
    var1 = (var3 + var4) / 262144;
    var1 = (((32768 + var1)) * ((int32_t)sens->coeff.dig_p1)) / 32768;

    /* Avoid exception caused by division by zero */
    if (var1) {
        var5 = (uint32_t)((uint32_t)1048576) - adc_P;
        pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;

        if (pressure < 0x80000000u) {
            pressure = (pressure * 2) / ((uint32_t)var1);
        } else {
            pressure = (pressure / (uint32_t)var1) * 2;
        }

        var1 = (((int32_t)sens->coeff.dig_p9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
        var2 = (((int32_t)(pressure / 4)) * ((int32_t)sens->coeff.dig_p8)) / 8192;
        pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + sens->coeff.dig_p7) / 16));

        if (pressure < pressure_min) {
            pressure = pressure_min;
        } else if (pressure > pressure_max) {
            pressure = pressure_max;
        }
    } else {
        pressure = pressure_min;
    }
    *fPressure = pressure / 100.0;
    return ESP_OK;
}


static esp_err_t bme280_convert_humidity(bme280_handle_t sensor, float *fHumidity)
{
    bme280_dev_t *sens = (bme280_dev_t *) sensor;

    int32_t adc_H = sens->adc_H;
    if (adc_H == 0x8000) { // value in case humidity measurement was disabled
        ESP_LOGW(TAG,"invalid humidity");
        return ESP_FAIL;
    }
    ESP_LOGD(TAG,"raw humidity is %" PRIi32 " and t_fine is %" PRIi32,adc_H,sens->t_fine);

    /* 
        original compensation code from Bosch, 
        https://github.com/boschsensortec/BME280_SensorAPI/blob/master/bme280.c  
    */
    int32_t var1, var2, var3, var4, var5;
    uint32_t humidity;
    const uint32_t humidity_max = 102400;

    var1 = sens->t_fine - ((int32_t)76800L);
    var2 = (int32_t)(adc_H * 16384L);
    var3 = (int32_t)(((int32_t)sens->coeff.dig_h4) * 1048576L);
    var4 = ((int32_t)sens->coeff.dig_h5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384L) / 32768L;
    var2 = (var1 * ((int32_t)sens->coeff.dig_h6)) / 1024L;
    var3 = (var1 * ((int32_t)sens->coeff.dig_h3)) / 2048L;
    var4 = ((var2 * (var3 + (int32_t)32768L)) / 1024L) + (int32_t)2097152L;
    var2 = ((var4 * ((int32_t)sens->coeff.dig_h2)) + 8192L) / 16384L;
    var3 = var5 * var2;
    var4 = ((var3 / 32768L) * (var3 / 32768L)) / 128L;
    var5 = var3 - ((var4 * ((int32_t)sens->coeff.dig_h1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400L ? 419430400L : var5);
    humidity = (uint32_t)(var5 / 4096);

    if (humidity > humidity_max) humidity = humidity_max;

    ESP_LOGD(TAG,"humidity is %" PRIu32, humidity);

    *fHumidity = humidity / 1024.0;
    return ESP_OK;
}


#pragma endregion
//===========================================================================
#pragma region High-level functions


/**
 * @brief   Create bme280 handle_t
 *
 * @param  i2c_port     I2C port to use, 0 or 1
 * @param  dev_addr     I2C 7-bit device address
 * @return
 *     - bme280_handle_t
 */
bme280_handle_t bme280_create(uint8_t i2c_port, uint8_t dev_addr)
{
    bme280_dev_t *sens = (bme280_dev_t *) calloc(1, sizeof(bme280_dev_t));
#if CONFIG_CODEC_I2C_BACKWARD_COMPATIBLE    
    sens->i2c_dev = i2c_port;
    sens->dev_addr = dev_addr;
    return (bme280_handle_t)sens;
#else
    i2c_master_bus_handle_t i2c_bus_h = (i2c_master_bus_handle_t)i2cBusHandle(i2c_port);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev_h;
    if (ESP_OK != i2c_master_bus_add_device(i2c_bus_h, &dev_cfg, &dev_h))
        return NULL;

    sens->dev_handle = dev_h;
    return (bme280_handle_t)sens;
#endif
}


/**
 * @brief  setup sensor with given parameters / settings
 *
 * @param sensor        object handle of bme280
 * @param mode          Sensor working mode (sleep/forced/normal)
 * @param tempSampling  temperature sampling factor, one of BME280_SAMPLING_XYZ constants
 * @param pressSampling pressure sampling factor, one of BME280_SAMPLING_XYZ constants
 * @param humSampling   humidity sampling factor, one of BME280_SAMPLING_XYZ constants
 * @param filter        Sensor filter factor, one of the BME280_FILTER constants
 * @param duration      standby duration of sensor
 * @return
 *    - ESP_OK Success
 *    - ESP_FAIL Fail
 */
esp_err_t bme280_set_sampling(bme280_handle_t sensor, bme280_sensor_mode mode, 
    bme280_sensor_sampling tempSampling, 
    bme280_sensor_sampling pressSampling, 
    bme280_sensor_sampling humSampling, 
    bme280_sensor_filter filter, 
    bme280_standby_duration duration
)
{
    bme280_dev_t *sens = (bme280_dev_t *) sensor;

    sens->ctrl_meas.mode = mode;
    sens->ctrl_meas.osrs_t = tempSampling;
    sens->ctrl_meas.osrs_p = pressSampling;
    sens->ctrl_hum.osrs_h = humSampling;
    sens->config.filter = filter;
    sens->config.t_sb = duration;

    // Set in sleep mode to provide write access to the “config” register
    ESP_RETURN_ON_ERROR(i2c_write_byte(sens, BME_REG_CONTROL, BME280_MODE_SLEEP),TAG,"bme280_set_sampling failed");
    // you must make sure to also set REGISTER_CONTROL after setting the
    // CONTROLHUMID register, otherwise the values won't be applied (see DS 5.4.3)
    ESP_RETURN_ON_ERROR(i2c_write_byte(sens, BME_REG_CONTROLHUMID, bme280_getctrl_hum(sensor)),TAG,"bme280_set_sampling failed");
    ESP_RETURN_ON_ERROR(i2c_write_byte(sens, BME_REG_CONFIG, bme280_getconfig(sensor)),TAG,"bme280_set_sampling failed");
    ESP_RETURN_ON_ERROR(i2c_write_byte(sens, BME_REG_CONTROL, bme280_getctrl_meas(sensor)),TAG,"bme280_set_sampling failed");
    return ESP_OK;
}


/**
 * @brief initialize bme280 device. Call this, then optionally 
 * call bme280_set_sampling() to change parameters
 *
 * @param sensor    object handle of bme280
 * @return
 *    - ESP_OK Success
 *    - ESP_FAIL Fail
 */
esp_err_t bme280_default_init(bme280_handle_t sensor)
{
    bme280_dev_t *sens = (bme280_dev_t *) sensor;
    // check if sensor, i.e. the chip ID is correct
    uint8_t chipid = 0;
    ESP_RETURN_ON_ERROR(i2c_read_byte(sens, BME_REG_CHIPID, &chipid),TAG,"read BME_REG_CHIPID failed");
    if (chipid != BME280_DEFAULT_CHIPID) {
        ESP_LOGE(TAG, "bme280_default_init->BME280_DEFAULT_CHIPID:%x", chipid);
        return ESP_FAIL;
    }
    // reset the sens using soft-reset, this makes sure the IIR is off, etc.
    ESP_RETURN_ON_ERROR(i2c_write_byte(sens, BME_REG_SOFTRESET, 0xB6),TAG,"BME_REG_SOFTRESET failed");

    // wait for chip to wake up.
    vTaskDelay(10 / portTICK_PERIOD_MS);    // was 300
    // if chip is still reading calibration, delay
    while (bme280_is_reading_calibration(sensor)) {
        vTaskDelay(10 / portTICK_PERIOD_MS);    // was 100
    }
    ESP_RETURN_ON_ERROR(bme280_read_coefficients(sensor),TAG,"bme280_read_coefficients failed"); // read trimming parameters, see DS 4.2.2
    ESP_RETURN_ON_ERROR(bme280_set_sampling(sensor, 
        BME280_MODE_NORMAL, 
        BME280_SAMPLING_X16, BME280_SAMPLING_X16, BME280_SAMPLING_X16, 
        BME280_FILTER_OFF, BME280_STANDBY_MS_0_5
    ),TAG,"bme280_set_sampling failed");
    return ESP_OK;
}


/**
 * @brief  Take a new single-shot measurement
 *
 * @param sensor    object handle of bme280
 * @return
 *    - ESP_OK Success
 *    - ESP_FAIL Fail
 */
esp_err_t bme280_take_forced_measurement(bme280_handle_t sensor)
{
    bme280_dev_t *sens = (bme280_dev_t *) sensor;
    uint8_t data = 0;

    sens->ctrl_meas.mode = BME280_MODE_FORCED;
    // set to forced mode, i.e. "take next measurement"
    ESP_RETURN_ON_ERROR(i2c_write_byte(sens, BME_REG_CONTROL, bme280_getctrl_meas(sensor)),TAG,"bme280_take_forced_measurement failed");

    // wait for measurement to begin
    ESP_RETURN_ON_ERROR(i2c_read_byte(sens, BME_REG_STATUS, &data),TAG,"bme280_tfm failed");                
    while ((data & 8)==0) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
        ESP_RETURN_ON_ERROR(i2c_read_byte(sens, BME_REG_STATUS, &data),TAG,"bme280_tfm failed");            
    }

    // wait for measurement to end
    while ((data & 0x08)) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        ESP_RETURN_ON_ERROR(i2c_read_byte(sens, BME_REG_STATUS, &data),TAG,"bme280_tfm failed");
    }

    return ESP_OK;
}


/**
 * @brief  Returns the temperature of most recent measurement from the sensor
 *
 * @param[in]  sensor        object handle of bme280
 * @param[out] temperature   pointer to temperature
 * @return esp_err_t
 */
esp_err_t bme280_read_temperature(bme280_handle_t sensor, float *temperature)
{
    ESP_RETURN_ON_ERROR(bme280_read_measurements(sensor),TAG,"bme280_read_measurements failed");
    ESP_RETURN_ON_ERROR(bme280_convert_temperature(sensor,temperature),TAG,"bme280_read_temperature failed");
    return ESP_OK;
}


/**
 * @brief  Returns the pressure of most recent measurement from the sensor,
 * or ESP_FAIL if pressure measurements are disabled
 *
 * @param[in]  sensor        object handle of bme280
 * @param[out] pressure      pointer to pressure value
 * @return esp_err_t
 */
esp_err_t bme280_read_pressure(bme280_handle_t sensor, float *pressure)
{
    ESP_RETURN_ON_ERROR(bme280_read_measurements(sensor),TAG,"bme280_read_measurements failed");
    float temperature;
    ESP_RETURN_ON_ERROR(bme280_convert_temperature(sensor,&temperature),TAG,"bme280_read_pressure failed");
    ESP_RETURN_ON_ERROR(bme280_convert_pressure(sensor,pressure),TAG,"bme280_read_pressure failed");
    return ESP_OK;
}


/**
 * @brief  Returns the humidity of most recent measurement from the sensor
 * or ESP_FAIL if humidity measurements are disabled
 *
 * @param[in]  sensor        object handle of bme280
 * @param[out] humidity      pointer to humidity value
 * @return esp_err_t
 */
esp_err_t bme280_read_humidity(bme280_handle_t sensor, float *humidity)
{
    ESP_RETURN_ON_ERROR(bme280_read_measurements(sensor),TAG,"bme280_read_measurements failed");
    float temperature;
    ESP_RETURN_ON_ERROR(bme280_convert_temperature(sensor,&temperature),TAG,"bme280_read_humidity failed");
    ESP_RETURN_ON_ERROR(bme280_convert_humidity(sensor,humidity),TAG,"bme280_read_humidity failed");
    return ESP_OK;
}


/**
 * @brief  Returns alll values of most recent measurement from the sensor
 *
 * @param[in]  sensor        object handle of bme280
 * @param[out] temperature   pointer to temperature
 * @param[out] humidity      pointer to humidity value
 * @param[out] pressure      pointer to pressure value
 * @return esp_err_t
 */
esp_err_t bme280_read_climate(bme280_handle_t sensor, float* temperature, float* humidity, float* pressure )
{
    ESP_RETURN_ON_ERROR(bme280_read_measurements(sensor),TAG,"bme280_read_measurements failed");
    ESP_RETURN_ON_ERROR(bme280_convert_temperature(sensor,temperature),TAG,"bme280_convert_temperature failed");
    if (humidity)
        ESP_RETURN_ON_ERROR(bme280_convert_humidity(sensor,humidity),TAG,"bme280_read_humidity failed");
    if (pressure)
        ESP_RETURN_ON_ERROR(bme280_convert_pressure(sensor,pressure),TAG,"bme280_convert_pressure failed");
    return ESP_OK;
}


/**
 * @brief Calculates the altitude (in meters) from the specified atmospheric
 *  pressure (in hPa), and sea-level pressure (in hPa).
 *
 * @param sensor object handle of bme280
 * @param seaLevel: Sea-level pressure in hPa
 * @param altitude pointer to altitude value
 * @return esp_err_t
 */
esp_err_t bme280_read_altitude(bme280_handle_t sensor, float seaLevel, float *altitude)
{
    float pressure = 0.0;
    float temp = 0.0;
    if (bme280_read_pressure(sensor, &temp) != ESP_OK) {
        return ESP_FAIL;
    }
    float atmospheric = pressure / 100.0F;
    *altitude = 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
    return ESP_OK;
}


/**
 * Calculates the pressure at sea level (in hPa) from the specified altitude
 * (in meters), and atmospheric pressure (in hPa).
 *
 * @param sensor object handle of bme280
 * @param altitude      Altitude in meters
 * @param atmospheric   Atmospheric pressure in hPa
 * @param pressure pointer to pressure value
 * @return esp_err_t
 */
esp_err_t bme280_calculates_pressure(bme280_handle_t sensor, float altitude,
                                     float atmospheric, float *pressure)
{
    *pressure = atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
    return ESP_OK;
}

#pragma endregion
