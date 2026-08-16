/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file ens160.c
 *
 * ESP-IDF driver for ENS160 Air Quality sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 * 
 * Modifications for use with driver/i2c.h
 * Copyright (C) 2025 Bernd Waldmann
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "sdkconfig.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#if CONFIG_CODEC_I2C_BACKWARD_COMPATIBLE
 #include "driver/i2c.h"
#else
 #include "driver/i2c_master.h"
#endif
#include "esp32-hal-i2c.h"
#include "ens160.h"

static const char *TAG = "ens160";


#define ENS160_TEMPERATURE_MAX         (float)(125.0)  //!< ens160 maximum temperature range
#define ENS160_TEMPERATURE_MIN         (float)(-40.0)  //!< ens160 minimum temperature range
#define ENS160_HUMIDITY_MAX            (float)(100.0)  //!< ens160 maximum humidity range
#define ENS160_HUMIDITY_MIN            (float)(0.0)    //!< ens160 minimum humidity range

#define ENS160_POWERUP_DELAY_MS         15            //!< ens160 50ms delay before making i2c transactions
#define ENS160_APPSTART_DELAY_MS        25            //!< ens160 25ms delay before making a measurement
#define ENS160_CMD_DELAY_MS             5             //!< ens160 5ms delay before making the next i2c transaction
#define ENS160_MODE_DELAY_MS            10            //!< ens160 10ms delay when updating the operating mode
#define ENS160_RESET_DELAY_MS           50            //!< ens160 50ms delay when resetting the device
#define ENS160_CLEAR_GPR_DELAY_MS       10            //!< ens160 10ms delay when clearing general purpose registers
#define ENS160_DATA_READY_DELAY_MS      1             //!< ens160 1ms delay when checking data ready in a loop
#define ENS160_DATA_POLL_TIMEOUT_MS     1500          //!< ens160 1.5s timeout when making a measurement
#define ENS160_TX_RX_DELAY_MS           10

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ENS160_CONVERT_RS_RAW2OHMS_F(x) 	(pow (2, (float)(x) / 2048))
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

//===========================================================================
#pragma region ENS160 register definitions

#define ENS160_REG_PART_ID_R            0x00 //!< ens160 I2C part identifier (default id: 0x01, 0x60)
#define ENS160_REG_OPMODE_RW            0x10 //!< ens160 I2C operating mode
#define ENS160_REG_INT_CONFIG_RW        0x11 //!< ens160 I2C interrupt pin configuration
#define ENS160_REG_COMMAND_RW           0x12 //!< ens160 I2C additional system commands
#define ENS160_REG_TEMP_IN_RW           0x13 //!< ens160 I2C host ambient temperature information
#define ENS160_REG_RH_IN_RW             0x15 //!< ens160 I2C host relative humidity information
#define ENS160_REG_DEVICE_STATUS_R      0x20 //!< ens160 I2C operating status
#define ENS160_REG_DATA_AQI_R           0x21 //!< ens160 I2C air quality index
#define ENS160_REG_DATA_TVOC_R          0x22 //!< ens160 I2C TVOC concentration (ppb)
#define ENS160_REG_DATA_ETOH_R          0x22 //!< ens160 I2C ETOH concentration (ppb)
#define ENS160_REG_DATA_ECO2_R          0x24 //!< ens160 I2C equivalent CO2 concentration (ppm)
#define ENS160_REG_DATA_BL_R            0x28 //!< ens160 I2C baseline information
#define ENS160_REG_DATA_TEMP_R          0x30 //!< ens160 I2C temperature used in calculations
#define ENS160_REG_DATA_RH_R            0x32 //!< ens160 I2C relative humidity used in calculations
#define ENS160_REG_DATA_MISR_R          0x38 //!< ens160 I2C data integrity field
#define ENS160_REG_GPR_WRITE0_RW        0x40 //!< ens160 I2C general purpose write0 register
#define ENS160_REG_GPR_WRITE1_RW        0x41 //!< ens160 I2C general purpose write1 register
#define ENS160_REG_GPR_WRITE2_RW        0x42 //!< ens160 I2C general purpose write2 register
#define ENS160_REG_GPR_WRITE3_RW        0x43 //!< ens160 I2C general purpose write3 register
#define ENS160_REG_GPR_WRITE4_RW        0x44 //!< ens160 I2C general purpose write4 register
#define ENS160_REG_GPR_WRITE5_RW        0x45 //!< ens160 I2C general purpose write5 register
#define ENS160_REG_GPR_WRITE6_RW        0x46 //!< ens160 I2C general purpose write6 register
#define ENS160_REG_GPR_WRITE7_RW        0x47 //!< ens160 I2C general purpose write7 register
#define ENS160_REG_GPR_READ0_R          0x48 //!< ens160 I2C general purpose read0 register
#define ENS160_REG_GPR_READ1_R          0x49 //!< ens160 I2C general purpose read1 register
#define ENS160_REG_GPR_READ2_R          0x4a //!< ens160 I2C general purpose read2 register
#define ENS160_REG_GPR_READ3_R          0x4b //!< ens160 I2C general purpose read3 register
#define ENS160_REG_GPR_READ4_R          0x4c //!< ens160 I2C general purpose read4 register
#define ENS160_REG_GPR_READ5_R          0x4d //!< ens160 I2C general purpose read5 register
#define ENS160_REG_GPR_READ6_R          0x4e //!< ens160 I2C general purpose read6 register
#define ENS160_REG_GPR_READ7_R          0x4f //!< ens160 I2C general purpose read7 register

/**
 * @brief ENS160 status register structure.
 */
typedef union __attribute__((packed)) ens160_status_register_u {
    struct STS_REG_BIT_TAG {
        bool                        new_gpr_data:1;   /*!< true indicates new data is available in `GPR_READ` registers (bit:0)   */
        bool                        new_data:1;       /*!< true indicates new data is available in `DATA_x` registers   (bit:1)   */
        ens160_validity_flags_t     state:2;          /*!< device status                                                (bit:2-3) */
        uint8_t                     reserved:2;       /*!< reserved and set 0                                           (bit:4-5) */
        bool                        error:1;          /*!< true indicates an error is detected                          (bit:6)   */
        bool                        mode:1;           /*!< true indicates an operating mode is running                  (bit:7)   */
    } bits;            /*!< represents the 8-bit status register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit status register as `uint8_t`.   */
} ens160_status_register_t;

/**
 * @brief ENS160 interrupt configuration register structure.
 */
typedef union __attribute__((packed)) ens160_interrupt_config_register_u {
    struct CFG_REG_BIT_TAG {
        bool                        irq_enabled:1;       /*!< true indicates interrupt pin is enabled                       (bit:0)   */
        bool                        irq_data_enabled:1;  /*!< true indicates interrupt pin is asserted when new data is available in `DATA_XXX` registers  (bit:1)   */
        uint8_t                     reserved1:1;         /*!< reserved and set to 0                                         (bit:2)   */
        bool                        irq_gpr_enabled:1;   /*!< true indicates interrupt pin is asserted when new data is available in general purpose registers (bit:3) */
        uint8_t                     reserved2:1;         /*!< reserved and set to 0                                         (bit:4)   */
        ens160_interrupt_pin_drivers_t irq_pin_driver:1; /*!< interrupt pin driver configuration                        (bit:5)   */
        ens160_interrupt_pin_polarities_t irq_pin_polarity:1; /*!< interrupt pin polarity configuration                 (bit:6)   */
        uint8_t                     reserved3:1;         /*!< reserved and set to 0                                         (bit:7)   */
    } bits;            /*!< represents the 8-bit interrupt configuration register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit interrupt configuration register as `uint8_t`.   */
} ens160_interrupt_config_register_t;

/**
 * @brief ENS160 application version register structure.
 */
typedef union ens160_app_version_u {
    uint8_t major;      /*!< ens160 major firmware version */
    uint8_t minor;      /*!< ens160 minor firmware version */
    uint8_t release;    /*!< ens160 firmware version release */
    uint8_t bytes[3];   /*!< represents app version as a byte array. */
} ens160_app_version_t;

/**
 * @brief ENS160 calculated air quality index (aqi) data register structure.  See datasheet for AQI-UBA details.
 */
typedef union __attribute__((packed)) ens160_caqi_data_register_u {
    struct CAL_AQI_REG_BITS_TAG {
        uint8_t          aqi_uba:3;     /*!< air quality index per uba[1..5] (default: 0x01)  (bit:0-2)  */
        uint8_t          reserved:5;    /*!< reserved and set to 0                            (bit:3-7)  */
    } bits;        /*!< represents the 8-bit calculated air quality index data register parts in bits.  */
    uint8_t value; /*!< represents the 8-bit calculated air quality index data register as `uint8_t`.   */
} ens160_caqi_data_register_t;

/**
 * @brief ENS160 device descriptor structure definition.
 */
typedef struct ens160_device_s {
    ens160_config_t                     config;                 /*!< ens160 configuration */
#if !(CONFIG_CODEC_I2C_BACKWARD_COMPATIBLE)
    i2c_master_dev_handle_t             i2c_handle;             /*!< ens160 i2c device handle */
#endif
    uint16_t                            part_id;                /*!< ens160 part identifier */
    //i2c_ens160_operating_modes_t            mode;               /*!< ens160 operating mode */
    //float                                   temperature_comp;   /*!< ens160 temperature compensation in degrees Celsius */
    //float                                   humidity_comp;      /*!< ens160 humidity compensation in percentage */
} ens160_device_t;

#pragma endregion
//===========================================================================
#pragma region Basic I2C transactions


#if CONFIG_CODEC_I2C_BACKWARD_COMPATIBLE
static int i2c_bus_write(ens160_device_t* const device, uint8_t reg_addr, const uint8_t *reg_data, uint8_t cnt)
{
	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (device->config.i2c_address << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, reg_addr, true);
	i2c_master_write(cmd, reg_data, cnt, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin((i2c_port_t)device->config.i2c_port, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return espRc;
}
#endif


#if CONFIG_CODEC_I2C_BACKWARD_COMPATIBLE
static int i2c_bus_read(ens160_device_t* const device, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (device->config.i2c_address << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (device->config.i2c_address << 1) | I2C_MASTER_READ, true);

	if (cnt > 1) {
		i2c_master_read(cmd, reg_data, cnt-1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data+cnt-1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin((i2c_port_t)device->config.i2c_port, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return espRc;
}
#endif

#pragma endregion
//===========================================================================
#pragma region Internal function for I2C transactions

/**
 * @brief ENS160 I2C HAL write byte to register address transaction.
 * 
 * @param device ENS160 device descriptor.
 * @param reg_addr ENS160 register address to write to.
 * @param reg_data ENS160 write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_write_byte_to(ens160_device_t *const device, const uint8_t reg_addr, const uint8_t reg_data) 
{
    ESP_ARG_CHECK( device );
#if CONFIG_CODEC_I2C_BACKWARD_COMPATIBLE
    ESP_RETURN_ON_ERROR( i2c_bus_write(device, reg_addr, &reg_data, 1), TAG, "i2c_bus_write failed" );
#else
    uint8_t tx[] = { reg_addr, reg_data };
    ESP_RETURN_ON_ERROR( 
        i2c_master_transmit(device->i2c_handle, tx, sizeof(tx), 1 ),
        TAG, "i2c_bus_write failed" );
#endif
    return ESP_OK;
}

/**
 * @brief ENS160 I2C write word to register address transaction.
 * 
 * @param device ENS160 device descriptor.
 * @param reg_addr ENS160 register address to write to.
 * @param word ENS160 write transaction input word.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_write_word_to(ens160_device_t *const device, const uint8_t reg_addr, const uint16_t word) 
{
    const uint8_t tx[3] = { reg_addr, (uint8_t)(word & 0xff), (uint8_t)((word >> 8) & 0xff) };

    ESP_ARG_CHECK( device );
#if CONFIG_CODEC_I2C_BACKWARD_COMPATIBLE
    ESP_RETURN_ON_ERROR( i2c_bus_write(device, reg_addr, tx+1, 2 ), TAG, "ens160_i2c_write_word_to failed" );
#else
    ESP_RETURN_ON_ERROR( 
        i2c_master_transmit(device->i2c_handle, tx, sizeof(tx), 10 ),
        TAG, "ens160_i2c_write_word_to failed" );
#endif
    return ESP_OK;
}

/**
 * @brief ENS160 I2C read from register address transaction.  This is a write and then read process.
 * 
 * @param device ENS160 device descriptor.
 * @param reg_addr ENS160 register address to read from.
 * @param buffer ENS160 read transaction return buffer.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_read_from(ens160_device_t *const device, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) 
{
    ESP_ARG_CHECK( device );
#if CONFIG_CODEC_I2C_BACKWARD_COMPATIBLE
    ESP_RETURN_ON_ERROR( i2c_bus_read( device, reg_addr, buffer, size), TAG, "ens160_i2c_read_from failed" );
#else
    ESP_RETURN_ON_ERROR( 
        i2c_master_transmit_receive( 
            device->i2c_handle, 
            &reg_addr, 1,
            buffer, size,
            10
        ),
        TAG, "ens160_i2c_read_from failed" 
    );
#endif
    return ESP_OK;
}

/**
 * @brief ENS160 I2C read word from register address transaction.
 * 
 * @param device ENS160 device descriptor.
 * @param reg_addr ENS160 register address to read from.
 * @param word ENS160 read transaction return word.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_read_word_from(ens160_device_t *const device, const uint8_t reg_addr, uint16_t *const word) 
{
    uint8_t rx[2] = { 0 };

    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_read_from( device, reg_addr, rx, 2 ), 
        TAG, "ens160_i2c_read_word_from failed" );
    *word = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);
    return ESP_OK;
}

/**
 * @brief ENS160 I2C read byte from register address transaction.
 * 
 * @param device ENS160 device descriptor.
 * @param reg_addr ENS160 register address to read from.
 * @param reg_data ENS160 read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_read_byte_from(
    ens160_device_t *const device, const uint8_t reg_addr, uint8_t *const reg_data) 
{
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_read_from( device, reg_addr, reg_data, 1 ), 
    TAG, "ens160_i2c_read_byte_from failed" );
    return ESP_OK;
}

#pragma endregion
//===========================================================================
#pragma region Internal function for ESN160 register access and conversions

/**
 * @brief Get air quality (uba) index.
 */
static inline unsigned ens160_get_aqi_uba(const ens160_caqi_data_register_t reg) 
{
    return reg.bits.aqi_uba;
}

/**
 * @brief Decodes `uint16_t` temperature format to degrees Celsius.
 * 
 * @param[in] encoded_temperature compensation temperature from register.
 * @return float temperature compensation in degrees Celsius.
 */
static inline float ens160_decode_temperature(const uint16_t encoded_temperature) 
{
    return (float)((encoded_temperature / 64.0f) - 273.15f);
}

/**
 * @brief Encodes temperature in degrees Celsius to `uint16_t` format.
 * 
 * @param[in] decoded_temperature compensation temperature in degrees Celsius.
 * @return uint16_t temperature compensation.
 */
static inline uint16_t ens160_encode_temperature(const float decoded_temperature) 
{
    return (uint16_t)((decoded_temperature + 273.15) * 64);
}

/**
 * @brief Decodes `uint16_t` humidity format.
 * 
 * @param[in] encoded_humidity compensation humidity from register.
 * @return float humidity compensation.
 */
static inline float ens160_decode_humidity(const uint16_t encoded_humidity) 
{
    return (float)(encoded_humidity / 512.0f);
}

/**
 * @brief Encodes humidity to `uint16_t` format.
 * 
 * @param[in] decoded_humidity compensation humidity.
 * @return uint16_t humidity compensation.
 */
static inline uint16_t ens160_encode_humidity(const float decoded_humidity) 
{
    return (uint16_t)(decoded_humidity * 512);
}

/**
 * @brief Reads command from ENS160 command register.
 * 
 * @param device ENS160 device descriptor.
 * @param command Command returned from ENS160 command register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_get_command(ens160_device_t *const device, ens160_commands_t *const command) 
{
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_read_byte_from(device, ENS160_REG_COMMAND_RW, (uint8_t*)command), TAG, "read command register failed" );
    return ESP_OK;
}

/**
 * @brief Writes command to ENS160 command register.
 * 
 * @param device ENS160 device descriptor.
 * @param command ENS160 command for command register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_set_command(ens160_device_t *const device, const ens160_commands_t command) 
{
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_write_byte_to(device, ENS160_REG_COMMAND_RW, command), TAG, "write command register failed" );
    return ESP_OK;
}

/**
 * @brief Reads operating mode register from ENS160.
 * 
 * @param[in] device ENS160 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_get_mode_register(ens160_device_t *const device, ens160_operating_modes_t *const mode) 
{
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_read_byte_from(device, ENS160_REG_OPMODE_RW, (uint8_t*)mode), TAG, "read operating mode register for get mode failed" );
    vTaskDelay(pdMS_TO_TICKS(ENS160_MODE_DELAY_MS));
    return ESP_OK;
}

/**
 * @brief Writes operating mode register to ENS160.
 * 
 * @param[in] device ENS160 device descriptor.
 * @param[in] mode Operating mode register setting.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_set_mode_register(ens160_device_t *const device, const ens160_operating_modes_t mode) 
{
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_write_byte_to(device, ENS160_REG_OPMODE_RW, mode), TAG, "write operating mode register for set mode failed" );
    vTaskDelay(pdMS_TO_TICKS(ENS160_MODE_DELAY_MS));
    return ESP_OK;
}

/**
 * @brief Reads interrupt configuration register from ENS160.
 *
 * @param[in] device ENS160 device descriptor.
 * @param[out] reg ENS160 interrupt configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_get_interrupt_config_register(ens160_device_t *const device, ens160_interrupt_config_register_t *const reg) 
{
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_read_byte_from(device, ENS160_REG_INT_CONFIG_RW, &reg->reg), TAG, "read interrupt configuration register failed" );
    return ESP_OK;
}

/**
 * @brief Writes interrupt configuration register to ENS160.
 *
 * @param[in] device ENS160 device descriptor.
 * @param[in] reg ENS160 interrupt configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_set_interrupt_config_register(ens160_device_t *const device, const ens160_interrupt_config_register_t reg) 
{
    ESP_ARG_CHECK( device );
    ens160_interrupt_config_register_t irq_config = { .reg = reg.reg };
    irq_config.bits.reserved1 = 0;
    irq_config.bits.reserved2 = 0;
    irq_config.bits.reserved3 = 0;
    ESP_RETURN_ON_ERROR( ens160_i2c_write_byte_to(device, ENS160_REG_INT_CONFIG_RW, irq_config.reg), TAG, "write interrupt configuration register failed" );
    return ESP_OK; 
}

/**
 * @brief Reads status register from ENS160.
 *
 * @param[in] device ENS160 device descriptor.
 * @param[out] reg ENS160 status configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_get_status_register(ens160_device_t *const device, ens160_status_register_t *const reg) 
{
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_read_byte_from(device, ENS160_REG_DEVICE_STATUS_R, &reg->reg), TAG, "read device status register failed" );
    return ESP_OK;
}

/**
 * @brief Resets command to operate normal and clears general purpose registers on ENS160.
 *
 * @param[in] device ENS160 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_clear_general_purpose_registers(ens160_device_t *const device) 
{
    ESP_ARG_CHECK( device );

    /* attempt to set normal operation command */
    ESP_RETURN_ON_ERROR( ens160_i2c_set_command(device, ENS160_CMD_NORMAL), TAG, "write normal operation command failed" );
    
    /* attempt to set clear general purpose registers command */
    ESP_RETURN_ON_ERROR( ens160_i2c_set_command(device, ENS160_CMD_CLEAR_GPR), TAG, "write clear general purpose registers command failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(ENS160_CLEAR_GPR_DELAY_MS));

    /* attempt to set normal operation command */
    ESP_RETURN_ON_ERROR( ens160_i2c_set_command(device, ENS160_CMD_NORMAL), TAG, "write normal operation command failed" );

    return ESP_OK;
}

/**
 * @brief Reads temperature and humidity compensation registers from ENS160.
 *
 * @param[in] device ENS160 device descriptor.
 * @param[out] temperature temperature compensation in degree Celsius.
 * @param[out] humidity humidity compensation in percentage.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_get_compensation_registers(ens160_device_t *const device, float *const temperature, float *const humidity) 
{
    uint16_t t; uint16_t h;

    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_read_word_from(device, ENS160_REG_TEMP_IN_RW, &t), TAG, "read temperature compensation register failed" );
    ESP_RETURN_ON_ERROR( ens160_i2c_read_word_from(device, ENS160_REG_RH_IN_RW, &h), TAG, "read humidity compensation register failed" );
    *temperature = ens160_decode_temperature(t);
    *humidity    = ens160_decode_humidity(h);
    return ESP_OK;
}

/**
 * @brief Writes temperature and humidity compensation registers to ENS160.
 *
 * @param[in] device ENS160 device descriptor.
 * @param[in] temperature temperature compensation in degree Celsius.
 * @param[in] humidity humidity compensation in percentage.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_set_compensation_registers(ens160_device_t *const device, const float temperature, const float humidity) 
{
    ESP_ARG_CHECK( device );
    if(temperature > ENS160_TEMPERATURE_MAX || temperature < ENS160_TEMPERATURE_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "temperature is out of range, write compensation registers failed");
    }
    if(humidity > ENS160_HUMIDITY_MAX || humidity < ENS160_HUMIDITY_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "humidity is out of range, write compensation registers failed");
    }
    uint16_t t = ens160_encode_temperature(temperature); 
    uint16_t h = ens160_encode_humidity(humidity);
    ESP_RETURN_ON_ERROR( ens160_i2c_write_word_to(device, ENS160_REG_TEMP_IN_RW, t), TAG, "write temperature compensation register failed" );
    ESP_RETURN_ON_ERROR( ens160_i2c_write_word_to(device, ENS160_REG_RH_IN_RW, h), TAG, "write humidity compensation register failed" );
    return ESP_OK;
}

/**
 * @brief Reads part identifier register from ENS160.
 *
 * @param[in] device ENS160 device descriptor.
 * @param[out] reg Part id register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_get_part_id_register(ens160_device_t *const device, uint16_t *const reg) 
{
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_read_word_from(device, ENS160_REG_PART_ID_R, reg), TAG, "read part identifier register failed" );
    return ESP_OK;
}

/**
 * @brief Write reset command to ENS160 operating mode register.
 * 
 * @param device device ENS160 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_set_reset(ens160_device_t *const device) 
{
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_set_mode_register(device, ENS160_OPMODE_RESET), TAG, "write mode for soft-reset failed" );
    vTaskDelay(pdMS_TO_TICKS(ENS160_RESET_DELAY_MS));
    return ESP_OK;

}

/**
 * @brief Setup ENS160 registers.
 * 
 * @param device device ENS160 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ens160_i2c_setup_registers(ens160_device_t *const device) 
{
    ens160_interrupt_config_register_t irq_config;

    ESP_ARG_CHECK( device );

    /* attempt to read part identifier */
    ESP_RETURN_ON_ERROR( ens160_i2c_get_part_id_register(device, &device->part_id), TAG, "read part identifier register for setup registers failed" );

    /* attempt to read interrupt configuration register */
    ESP_RETURN_ON_ERROR( ens160_i2c_get_interrupt_config_register(device, &irq_config), TAG, "read interrupt configuration register for setup registers failed" );

    /* attempt to enable idle operating mode before writing to configuration registers */
    ESP_RETURN_ON_ERROR( ens160_i2c_set_mode_register(device, ENS160_OPMODE_IDLE), TAG, "enable idle operating mode for setup registers failed" );

    /* attempt to clear general purpose registers */
    ESP_RETURN_ON_ERROR( ens160_i2c_clear_general_purpose_registers(device), TAG, "clear general purpose registers for setup registers failed" );

    /* copy irq configuration from device handle */
    irq_config.bits.irq_enabled         = device->config.irq_enabled;
    irq_config.bits.irq_data_enabled    = device->config.irq_data_enabled;
    irq_config.bits.irq_gpr_enabled     = device->config.irq_gpr_enabled;
    irq_config.bits.irq_pin_driver      = device->config.irq_pin_driver;
    irq_config.bits.irq_pin_polarity    = device->config.irq_pin_polarity;

    /* attempt to write interrupt configuration register */
    ESP_RETURN_ON_ERROR( ens160_i2c_set_interrupt_config_register(device, irq_config), TAG, "write interrupt configuration register for setup registers failed" );

    /* attempt to enable standard operating mode to start making measurements (idle by default)  */
    ESP_RETURN_ON_ERROR( ens160_i2c_set_mode_register(device, ENS160_OPMODE_STANDARD), TAG, "enable standard operating mode for setup registers failed" );

    return ESP_OK;
}

#pragma endregion
//===========================================================================
#pragma region Public functions


esp_err_t ens160_init(const ens160_config_t *ens160_config, ens160_handle_t *ens160_handle) 
{
    /* validate arguments */
    ESP_ARG_CHECK( ens160_config );

    /* power-up task delay */
    vTaskDelay(pdMS_TO_TICKS(ENS160_POWERUP_DELAY_MS));

    /* validate memory availability for handle */
    ens160_device_t* device = (ens160_device_t*)calloc(1, sizeof(ens160_device_t));
    if (NULL==device) return ESP_ERR_NO_MEM;

    /* copy configuration */
    device->config = *ens160_config;

#if !(CONFIG_CODEC_I2C_BACKWARD_COMPATIBLE)
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = device->config.i2c_address,
        .scl_speed_hz       = device->config.i2c_clock_speed,
    };

    i2c_master_bus_handle_t i2c_bus_h = (i2c_master_bus_handle_t)i2cBusHandle(ens160_config->i2c_port);
    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(i2c_bus_h, &i2c_dev_conf, &device->i2c_handle), 
        TAG, "i2c new bus for init failed"
    );
#endif

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(ENS160_CMD_DELAY_MS));

    /* attempt to reset device and initialize device configuration and handle */

    /* attempt to reset  */
    ESP_RETURN_ON_ERROR( ens160_i2c_set_reset(device), TAG, "set soft-reset for init failed" );

    /* attempt to setup registers to reset  */
    ESP_RETURN_ON_ERROR( ens160_i2c_setup_registers(device), TAG, "setup registers for init failed" );

    /* set device handle */
    *ens160_handle = (ens160_handle_t)device;

    /* app-start task delay  */
    vTaskDelay(pdMS_TO_TICKS(ENS160_APPSTART_DELAY_MS));

    return ESP_OK;
}


esp_err_t ens160_get_measurement(ens160_handle_t handle, ens160_air_quality_data_t *const data) 
{
    esp_err_t                       ret             = ESP_OK;
    uint64_t                        start_time      = 0;
    bool                            data_is_ready   = false;
    ens160_caqi_data_register_t     caqi_reg;
    uint16_t                        tvoc_data;
    uint16_t                        etoh_data;
    uint16_t                        eco2_data;
    ens160_device_t*                device          = (ens160_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set start time (us) for timeout monitoring */
    start_time = esp_timer_get_time(); 

    /* attempt to poll until data is available or timeout */
    do {
        ens160_status_register_t status;

        /* attempt to poll if data is ready or timeout */
        //ESP_GOTO_ON_ERROR( ens160_get_data_status(handle, &data_is_ready), err, TAG, "data ready read for measurement failed." );
        /* attempt to read status register  */
        ESP_RETURN_ON_ERROR( ens160_i2c_get_status_register(device, &status), TAG, "read status register for get measurement failed" );

        /* set ready state */
        data_is_ready = status.bits.new_data;

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(ENS160_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (ENS160_DATA_POLL_TIMEOUT_MS * 1000)))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* attempt i2c data read transactions */
    ESP_GOTO_ON_ERROR( ens160_i2c_read_byte_from(device, ENS160_REG_DATA_AQI_R, &caqi_reg.value), err, TAG, "read calculated air quality index data register for measurement failed" );
    ESP_GOTO_ON_ERROR( ens160_i2c_read_word_from(device, ENS160_REG_DATA_TVOC_R, &tvoc_data), err, TAG, "read tvoc data register for measurement failed" );
    ESP_GOTO_ON_ERROR( ens160_i2c_read_word_from(device, ENS160_REG_DATA_ETOH_R, &etoh_data), err, TAG, "read etoh data register for measurement failed" );
    ESP_GOTO_ON_ERROR( ens160_i2c_read_word_from(device, ENS160_REG_DATA_ECO2_R, &eco2_data), err, TAG, "read eco2 data register for measurement failed" );

    /* set air quality fields */
    data->uba_aqi = ens160_get_aqi_uba(caqi_reg);
    data->tvoc    = tvoc_data;
    data->etoh    = etoh_data;
    data->eco2    = eco2_data;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(ENS160_CMD_DELAY_MS));

    return ESP_OK;

    err:
        return ret;
}


esp_err_t ens160_get_raw_measurement(ens160_handle_t handle, ens160_air_quality_raw_data_t *const data) 
{
    esp_err_t ret                 = ESP_OK;
    uint64_t  start_time          = 0;
    bool      gpr_data_is_ready   = false;
    uint8_t   rx[8]             = { 0 };
    ens160_device_t* device       = (ens160_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set start time (us) for timeout monitoring */
    start_time = esp_timer_get_time(); 

    /* attempt to poll until gpr data is available or timeout */
    do {
        /* attempt to check if gpr data is ready */
        ESP_GOTO_ON_ERROR( ens160_get_gpr_data_status(handle, &gpr_data_is_ready), err, TAG, "gpr data ready read for raw measurement failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(ENS160_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (ENS160_DATA_POLL_TIMEOUT_MS * 1000)))
            return ESP_ERR_TIMEOUT;
    } while (gpr_data_is_ready == false);

    /* attempt i2c gpr data read transactions */
    ESP_GOTO_ON_ERROR( ens160_i2c_read_from(device, ENS160_REG_GPR_READ0_R, rx, sizeof(rx)), err, TAG, "read resistance signal gpr data registers for raw measurement failed" );

    /* convert gpr raw resistance and set resistance signals */
    data->hp0_ri = ENS160_CONVERT_RS_RAW2OHMS_F((uint32_t)(rx[0] | ((uint16_t)rx[1] << 8)));
    data->hp1_ri = ENS160_CONVERT_RS_RAW2OHMS_F((uint32_t)(rx[2] | ((uint16_t)rx[3] << 8)));
    data->hp2_ri = ENS160_CONVERT_RS_RAW2OHMS_F((uint32_t)(rx[4] | ((uint16_t)rx[5] << 8)));
    data->hp3_ri = ENS160_CONVERT_RS_RAW2OHMS_F((uint32_t)(rx[6] | ((uint16_t)rx[7] << 8)));

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(ENS160_CMD_DELAY_MS));

    /* attempt i2c baseline data read transactions */
    ESP_GOTO_ON_ERROR( ens160_i2c_read_from(device, ENS160_REG_DATA_BL_R, rx, sizeof(rx)), err, TAG, "read baseline resistance data registers for raw measurement failed" );

    /* convert baseline raw resistance and set resistance signals */
    data->hp0_bl = ENS160_CONVERT_RS_RAW2OHMS_F((uint32_t)(rx[0] | ((uint16_t)rx[1] << 8)));
    data->hp1_bl = ENS160_CONVERT_RS_RAW2OHMS_F((uint32_t)(rx[2] | ((uint16_t)rx[3] << 8)));
    data->hp2_bl = ENS160_CONVERT_RS_RAW2OHMS_F((uint32_t)(rx[4] | ((uint16_t)rx[5] << 8)));
    data->hp3_bl = ENS160_CONVERT_RS_RAW2OHMS_F((uint32_t)(rx[6] | ((uint16_t)rx[7] << 8)));

    /* attempt to clear general purpose registers */
    //ESP_GOTO_ON_ERROR( ens160_clear_general_purpose_registers(ens160_handle), err, TAG, "clear general purpose registers failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(ENS160_CMD_DELAY_MS));

    return ESP_OK;

    err:
        return ret;
}


esp_err_t ens160_get_data_status(ens160_handle_t handle, bool *const ready) 
{
    ens160_status_register_t status;
    ens160_device_t* device = (ens160_device_t*)handle;
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_get_status_register(device, &status), TAG, "read status register for get data status failed" );
    *ready = status.bits.new_data;
    return ESP_OK;
}


esp_err_t ens160_get_gpr_data_status(ens160_handle_t handle, bool *const ready) 
{
    ens160_status_register_t status;
    ens160_device_t* device = (ens160_device_t*)handle;
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_get_status_register(device, &status), TAG, "read status register for get general purpose registers data status failed" );
    *ready = status.bits.new_gpr_data;
    return ESP_OK;
}


esp_err_t ens160_get_validity_status(ens160_handle_t handle, ens160_validity_flags_t *const state) 
{
    ens160_status_register_t status;
    ens160_device_t* device = (ens160_device_t*)handle;
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_get_status_register(device, &status), TAG, "read status register for get validity flag status failed" );
    *state = status.bits.state;

    return ESP_OK;
}


esp_err_t ens160_get_error_status(ens160_handle_t handle, bool *const error) 
{
    ens160_status_register_t status;
    ens160_device_t* device = (ens160_device_t*)handle;
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_get_status_register(device, &status), TAG, "read status register for get error status failed" );
    *error = status.bits.error;
    return ESP_OK;
}


esp_err_t ens160_get_mode_status(ens160_handle_t handle, bool *const mode) 
{
    ens160_status_register_t status;
    ens160_device_t* device = (ens160_device_t*)handle;
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_get_status_register(device, &status), TAG, "read status register for get operating mode status failed" );
    *mode = status.bits.mode;
    return ESP_OK;
}


esp_err_t ens160_get_status(ens160_handle_t handle, bool *const data_ready, bool *const gpr_data_ready, ens160_validity_flags_t *const state, bool *const error, bool *const mode) 
{
    ens160_status_register_t status;
    ens160_device_t* device = (ens160_device_t*)handle;
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_get_status_register(device, &status), TAG, "read status register for get status failed" );
    *data_ready     = status.bits.new_data;
    *gpr_data_ready = status.bits.new_gpr_data;
    *state          = status.bits.state;
    *error          = status.bits.error;
    *mode           = status.bits.mode;
    return ESP_OK;
}


esp_err_t ens160_get_compensation_factors(ens160_handle_t handle, float *const temperature, float *const humidity) 
{
    ens160_device_t* device = (ens160_device_t*)handle;
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_get_compensation_registers(device, temperature, humidity), TAG, "read compensation registers failed" );
    return ESP_OK;
}


esp_err_t ens160_set_compensation_factors(ens160_handle_t handle, const float temperature, const float humidity) 
{
    ens160_device_t* device = (ens160_device_t*)handle;
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_set_compensation_registers(device, temperature, humidity), TAG, "write compensation registers failed" );
    return ESP_OK;
}


esp_err_t ens160_enable_standard_mode(ens160_handle_t handle) 
{
    ens160_device_t* device = (ens160_device_t*)handle;
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_set_mode_register(device, ENS160_OPMODE_STANDARD), TAG, "write mode for standard operating mode failed" );
    return ESP_OK;
}


esp_err_t ens160_enable_idle_mode(ens160_handle_t handle) 
{
    ens160_device_t* device = (ens160_device_t*)handle;
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_set_mode_register(device, ENS160_OPMODE_IDLE), TAG, "write mode for idle operating mode failed" );
    return ESP_OK;
}


esp_err_t ens160_enable_deep_sleep_mode(ens160_handle_t handle) 
{
    ens160_device_t* device = (ens160_device_t*)handle;
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_set_mode_register(device, ENS160_OPMODE_DEEP_SLEEP), TAG, "write mode for deep sleep operating mode failed" );
    return ESP_OK;
}


esp_err_t ens160_reset(ens160_handle_t handle) 
{
    ens160_device_t* device = (ens160_device_t*)handle;
    ESP_ARG_CHECK( device );
    ESP_RETURN_ON_ERROR( ens160_i2c_set_reset(device), TAG, "write mode for soft-reset failed" );
    ESP_RETURN_ON_ERROR( ens160_i2c_setup_registers(device), TAG, "setup registers for soft-reset failed" );
    return ESP_OK;
}

#pragma endregion