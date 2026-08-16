/**
 * @file        LcDisplay_ST7789.cpp
 * @project     voice control projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2026-08-03
 * tabsize      4
 * 
 * This Revision: $Id: LcDisplay_ST7789.cpp 2006 2026-08-12 18:59:44Z  $
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
#include "esp_lcd_panel_st7789.h"

#include "error_checks.h"
#include "EspBoard.h"


#define TAG "LcDisplay_ST7789"

#define LCD_DRAW_BUF_HEIGHT 24
#define MAX_TRANSFER_SIZE   4096
#define LCD_BIGENDIAN       (1)



/**
 * @brief Initialize LCD with ST7789 controller, like on the ESP-BOX-Lite gadget
 * 
 * @return lv_disp_t*   LVGL display handle or NULL if error
 */
lv_disp_t* LcDisplay_ST7789::get_disp_handle()
{
   ESP_LOGI(TAG, "display is %d H x %d V",
        _config->width, _config->height
    );

    //----- init SPI bus
    spi_bus_config_t buscfg = { 
        .mosi_io_num = (gpio_num_t) _config->mosi,
        .miso_io_num = GPIO_NUM_NC,
        .sclk_io_num = (gpio_num_t) _config->sclk,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = MAX_TRANSFER_SIZE,
    };
    ERROR_CHECK_RETURN_NULL(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));

    //----- init LCD IO
    const esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = (gpio_num_t) _config->cs,
        .dc_gpio_num = (gpio_num_t) _config->dc,
        .spi_mode = 0,
        .pclk_hz = _config->pclk_hz,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
	esp_lcd_panel_io_handle_t io_handle = NULL;
    ERROR_CHECK_RETURN_NULL(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_config, &io_handle));

    //----- init LCD driver
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = (gpio_num_t) _config->rst,
        .color_space = _config->color_space,
        .bits_per_pixel = 16,
    };    
    esp_lcd_panel_handle_t lcd_panel_handle;
    ERROR_CHECK_RETURN_NULL(esp_lcd_new_panel_st7789(io_handle, &panel_config, &lcd_panel_handle));

    esp_lcd_panel_reset(lcd_panel_handle);
    esp_lcd_panel_init(lcd_panel_handle);
    esp_lcd_panel_mirror(lcd_panel_handle, _config->mirror_x, _config->mirror_y); 
    esp_lcd_panel_swap_xy(lcd_panel_handle, _config->swap_xy);  
    esp_lcd_panel_invert_color(lcd_panel_handle, _config->color_invert);

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
