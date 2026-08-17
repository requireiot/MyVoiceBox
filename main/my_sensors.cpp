/**
 * @file        my_sensors.cpp
 * @project     MyVoiceBox
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2025-11-28
 * tabsize  4
 * 
 * This Revision: $Id: my_sensors.cpp 2008 2026-08-14 20:03:51Z  $
 */

/*
   Copyright (C) 2025 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, 
   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
   obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief   Access to climate and air quality sensors via I2C
 *
 */
#include "sdkconfig.h"
#include "prefs.h"
#include "pins.h"

#include <Arduino.h>
#include "esp_log.h"
#if CONFIG_CODEC_I2C_BACKWARD_COMPATIBLE
 #include "driver/i2c.h"
#else
 #include "driver/i2c_master.h"
 #include "esp32-hal-i2c.h"
#endif

#include "bme280.h"
#include "ahtxx.h"
#include "ens160.h"

#include "ansi.h"
#include "pins.h"
#include "my_sensors.h"


#define USE_BARO
#define USE_BME280


#define ON_ERROR_RETURN_FALSE(x)  do {          \
        esp_err_t err = (x);                    \
        if (err != ESP_OK) {                    \
            log_e("error %d in line %d" ,(int)err, __LINE__); \
            return false;                       \
        }                                       \
    } while(0)


//============================================================================
#pragma region I2C interface

 static i2c_master_bus_handle_t i2c_bus_h = NULL;

#define USE_I2C_DETECT


static bool i2c_bus_probe(i2c_master_bus_handle_t bus_h, uint8_t dev_addr)
{
	return ESP_OK == i2c_master_probe(bus_h, dev_addr, 2);
}


void probe_i2c( void* handle )
{
    if (handle==NULL) return;
    i2c_master_bus_handle_t bus_h = (i2c_master_bus_handle_t)handle;

    uint8_t iaddr;
    printf("\nI2C  0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    for (int y = 0; y < 128; y += 16) {
        printf("%02x: ", y);
        for (int x = 0; x < 16; x++) {
            iaddr = x + y;
            if (i2c_bus_probe(bus_h,iaddr))
            {
                printf("%02x ", iaddr);
            } else {
                printf("-- ");
            }
        }
        printf("\n");
    }
    printf("\n");
}


/**
 * @brief Initialize I2C interface for communication with climate sensors
 * 
 * @return true     successful initialization
 * @return false    something went wrong
 */
bool init_Sensors_I2C()
{
	ON_ERROR_RETURN_FALSE(i2cInit(I2C_PORT,PIN_I2C_SDA,PIN_I2C_SCL,100000));
    i2c_bus_h = (i2c_master_bus_handle_t)i2cBusHandle(I2C_PORT);

#ifdef USE_I2C_DETECT
    probe_i2c( i2c_bus_h );
#endif // USE_I2C_DETECT
    return true;
}

#pragma endregion
//============================================================================
#pragma region BME280 handling
#ifdef USE_BME280


static bme280_handle_t bme_handle=0;


/**
 * @brief Read data from BME280 climate sensor
 * 
 * @param[out]      temperature [°C]
 * @param[out]      humidity [%rH]
 * @param[out]      pressure [hPa] or not set if USE_BARO not defined
 * @return true     successful read from sensor
 * @return false    some error occured
 */
bool update_BME( float& temperature, float& humidity, float& pressure )
{
    //ON_ERROR_RETURN_FALSE(bme280_take_forced_measurement(bme_handle));
    ON_ERROR_RETURN_FALSE(bme280_read_climate(bme_handle,&temperature,&humidity
#ifdef USE_BARO
    ,&pressure
#endif
    ));

    return true;
}


/**
 * @brief Initialize BME280 climate sensor
 * 
 * @return true     success
 * @return false    some error during initialization
 */
bool init_Sensors_BME()
{
    bme_handle = bme280_create(I2C_PORT,BME280_ADDRESS);
    if (NULL==bme_handle) {
        log_e("bme280_create() failed");
        return false;
    }

    if (ESP_OK != bme280_default_init(bme_handle)) {
        log_e("bme280_default_init() failed");
        return false;
    }
    return true;

    delay(1000);
/*
    bme280_set_sampling(bme_handle,
        BME280_MODE_SLEEP,
        BME280_SAMPLING_X1,
#ifdef USE_BARO
        BME280_SAMPLING_X1,
#else
        BME280_SAMPLING_NONE,
#endif
        BME280_SAMPLING_X1,
        BME280_FILTER_OFF,
        BME280_STANDBY_MS_0_5
    );
*/  
    float t,h,p;
    bool ok = update_BME(t,h,p);
    if (ok)
        log_i( "BME: "
            ANSI_BOLD "%.1f" ANSI_RESET "°C  "
            ANSI_BOLD "%.0f" ANSI_RESET " %%rH  "
            ANSI_BOLD "%.0f" ANSI_RESET " hPa  ",
            t, h, p
        );
    return ok;
}


#else
 bool update_BME( float& temp, float& humidity, float& pressure ) { return false; }
 bool init_Sensors_BME() { return false; }
#endif // USE_BME280

#pragma endregion
//============================================================================
#pragma region AHT21 handling

static ahtxx_handle_t aht_handle=0;

// for use by ENS160 routines
static float ahtTemperature, ahtHumidity;


/**
 * @brief Read data from AHT21 sensor
 * 
 * @param[out]      t   temperature [°C]
 * @param[out]      h   humidity [%rH] 
 * @return true         successful read from sensor
 * @return false        some error occured
 */
bool update_AHT( float& t, float& h )
{
    if (aht_handle==0) return false;

    if (ESP_OK != ahtxx_get_measurement(aht_handle,&t, &h)) {
        log_e("error in ahtxx_get_measurement()");
        return false;
    } else {
        ahtTemperature = t;
        ahtHumidity = h;
        log_i( 
            "AHT: " ANSI_BOLD "%.1f" ANSI_RESET " °C  " ANSI_BOLD "%.0f" ANSI_RESET " %%rH", 
            t, h
        );
        return true;
    }
}


/**
 * @brief Initialize AHT21 climate sensor on the same board as ENS160
 * Assumes that I2C interface has been initialized before
 * 
 * @return true     success
 * @return false    some error during initialization
 */
bool init_Sensors_AHT()
{
    // initialize i2c device configuration
    ahtxx_config_t dev_cfg = AHT21_CONFIG_DEFAULT;
    //
    // init device
    if ((ESP_OK != ahtxx_init( &dev_cfg, &aht_handle)) || (aht_handle == NULL)) {
        log_e("ahtxx_init() failed");
        return false;
    }

    float t,h;
    return update_AHT(t,h);
}


#pragma endregion
//============================================================================
#pragma region ENS160 handling

static ens160_handle_t ens160_handle;


/**
 * @brief Read air quality data from ENS160 sensor
 * 
 * @param[out] aqi      air quality index [1..5]
 * @param[out] tvoc     total volatile organic compounds concentration [ppb]
 * @param[out] eco2     equivalent CP2 concentration [ppm]
 * @return true         successful read from sensor
 * @return false        some error occured
 */
bool update_AQI( unsigned& valid, unsigned& aqi, unsigned& tvoc, unsigned& eco2 )
{
    if (ens160_handle==NULL) return false;
    if (aht_handle!=NULL && ahtTemperature>0.0) {
        ens160_set_compensation_factors(ens160_handle,ahtTemperature,ahtHumidity);
    }
    ens160_validity_flags_t dev_flag;
    if (ESP_OK != ens160_get_validity_status(ens160_handle, &dev_flag)) return false; 

    if (dev_flag < ENS160_VALFLAG_INVALID_OUTPUT) {
        ens160_air_quality_data_t aq_data;
        if (ESP_OK != ens160_get_measurement(ens160_handle, &aq_data)) return false;
        valid = (unsigned)dev_flag;
        aqi = (unsigned)aq_data.uba_aqi;
        tvoc = aq_data.tvoc;
        eco2 = aq_data.eco2;
        return true;
    }
    return false;
}


/**
 * @brief Initialize ENS160 air quality sensor. Assumes that I2C interface 
 * has been initialized before
 * 
 * @return true     success
 * @return false    some error during initialization
 */
bool init_Sensors_ENS()
{
    // initialize i2c device configuration
    ens160_config_t dev_cfg = I2C_ENS160_CONFIG_DEFAULT;
    //
    // init device
    if ((ESP_OK != ens160_init( &dev_cfg, &ens160_handle )) || (ens160_handle == NULL)) {
        log_e( "ens160_init() failed");
        return false;
    }

    delay(1000);

    unsigned valid, aqi, tvoc, eco2;
    return update_AQI(valid,aqi,tvoc,eco2);
}

#pragma endregion
