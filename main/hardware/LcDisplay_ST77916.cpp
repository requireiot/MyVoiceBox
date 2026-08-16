/**
 * @file        LcDisplay_ST7789.cpp
 * @project     voice control projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2026-08-03
 * tabsize      4
 * 
 * This Revision: $Id: $
 */

/*
   Copyright (C) 2026 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. 
   If a copy of the MPL was not distributed with this file, You can obtain one at 
   http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief Implementation of hardware control specific to 
 * custom device with 320x240 LCD sith ST7789 controller
 */

 #include "esp_err.h"
#include "esp_log.h"
#include "esp_lvgl_port.h" 
#include "esp_lcd_st77916.h"
#include "disp_init_data_st77916.h"

#include "error_checks.h"
#include "EspBoard.h"


#define TAG "my_lcd_77916"


#define _ERROR_CHECK_RETURN_NULL(x)  do {       \
        esp_err_t err;                          \
        if (unlikely((err=(x)) != ESP_OK)) {    \
            ESP_LOGW(TAG,"error %d in line %d" ,(int)err, __LINE__); \
            return NULL;                        \
        }                                       \
    } while(0)


#define LCD_DRAW_BUF_HEIGHT 24
#define MAX_TRANSFER_SIZE   4096
#define LCD_BIGENDIAN       (1)

#define LCD_SPI_HOST SPI2_HOST
#define USE_QSPI 1

/**
 * @brief Initialize LCD with ST77916 controller
 * 
 * @return lv_disp_t*   LVGL display handle or NULL if error
 */
lv_disp_t* LcDisplay_ST77916::get_disp_handle()
{
   ESP_LOGI(TAG, "display is %d H x %d V",
        _config->width, _config->height
    );

    ESP_LOGI(TAG, "Initialize QSPI bus");
    const spi_bus_config_t buscfg = 
    {
        .data0_io_num = _config->data0,
        .data1_io_num = _config->data1,
        .sclk_io_num = _config->sclk,
        .data2_io_num = _config->data2,
        .data3_io_num = _config->data3,
        .max_transfer_sz = (int)(_config->width * 80 * sizeof(uint16_t)),
    };
    _ERROR_CHECK_RETURN_NULL(
        spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO)
    );

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = 
        ST77916_PANEL_IO_QSPI_CONFIG(
            _config->cs, NULL, NULL);
    _ERROR_CHECK_RETURN_NULL(
        esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_config, &io_handle)
    );

    ESP_LOGI(TAG, "Install ST77916 panel driver");
    st77916_vendor_config_t vendor_config = {
        .init_cmds = disp_init_data,         // Uncomment these line if use custom initialization commands
        .init_cmds_size = sizeof(disp_init_data) / sizeof(st77916_lcd_init_cmd_t),
        .flags = {
            .use_qspi_interface = 1,
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = _config->rst,
        //.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,     // Implemented by LCD command `36h`
        .rgb_endian = LCD_RGB_ENDIAN_RGB,   // was BGR
        .bits_per_pixel = 16,    // Implemented by LCD command `3Ah` (16/18)
        .vendor_config = &vendor_config,
    };
    esp_lcd_panel_handle_t lcd_panel_handle;
    _ERROR_CHECK_RETURN_NULL(
        esp_lcd_new_panel_st77916(io_handle, &panel_config, &lcd_panel_handle)
    );



    esp_lcd_panel_reset(lcd_panel_handle);
    esp_lcd_panel_init(lcd_panel_handle);
    //esp_lcd_panel_mirror(lcd_panel_handle, _config->mirror_x, _config->mirror_y); 
    //esp_lcd_panel_swap_xy(lcd_panel_handle, _config->swap_xy);  
    //esp_lcd_panel_invert_color(lcd_panel_handle, _config->color_invert);

    ERROR_CHECK_RETURN_NULL(esp_lcd_panel_disp_on_off(lcd_panel_handle,true));

    //----- add LCD screen to LVGL
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = lcd_panel_handle,
        .buffer_size = _config->width * LCD_DRAW_BUF_HEIGHT,
        .double_buffer = false,
        .trans_size = MAX_TRANSFER_SIZE,
        .hres = _config->width,
        .vres = _config->height,
        .monochrome = false,
        .rotation = {
            .swap_xy = _config->swap_xy, 
            .mirror_x = _config->mirror_x,
            .mirror_y = _config->mirror_y,    // was true
        },
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags = {
            .buff_dma = true,
            .buff_spiram = true,
            .sw_rotate = false,
            .swap_bytes = (LCD_BIGENDIAN ? true : false),
        }
    };
    lv_disp_t* disp_handle;
    disp_handle = lvgl_port_add_disp(&disp_cfg);
    return disp_handle;
}
