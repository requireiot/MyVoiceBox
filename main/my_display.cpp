/**
 * @file        display.cpp
 * @project     MyVoiceBoxLite
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2025-11-28
 * tabsize  4
 * 
 * This Revision: $Id: my_display.cpp 2014 2026-08-16 16:07:27Z  $
 */

/*
   Copyright (C) 2025 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, 
   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
   obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief   Handle LVGL-controlled display on ESP-BOX-Lite, display climate data
 *
 */


#include "sdkconfig.h"
#include "prefs.h"
#include "pins.h"
//#include <Arduino.h>
#if HAS_DISPLAY
#include "esp_lvgl_port.h" 
#include "my_display.h"

/*
  built-in display is 320x240 with 2.4" diagonal, or 167ppi both H and V
*/

//==============================================================================
#pragma region Preferences

/*
 converted using Font Converter from https://lvgl.io/tools/fontconverter
 and MontSerrat 72 TTS font from https://fonts.google.com/selection 
*/
LV_FONT_DECLARE(lv_font_freesans_20)
LV_FONT_DECLARE(lv_font_freesans_bold_20)
LV_FONT_DECLARE(lv_font_freesans_24)
LV_FONT_DECLARE(lv_font_freesans_bold_24)
LV_FONT_DECLARE(lv_font_freesans_28)
LV_FONT_DECLARE(lv_font_freesans_bold_28)
LV_FONT_DECLARE(lv_font_freesans_40)
LV_FONT_DECLARE(lv_font_freesans_bold_40)
LV_FONT_DECLARE(lv_font_freesans_48)
LV_FONT_DECLARE(lv_font_freesans_bold_48)
LV_FONT_DECLARE(lv_font_freesans_56)
LV_FONT_DECLARE(lv_font_freesans_bold_56)
LV_FONT_DECLARE(lv_font_freesans_72)
LV_FONT_DECLARE(lv_font_freesans_bold_72)


static const lv_color_t color_int = lv_color_hex(0xFF40FF); 
static const lv_color_t color_ext = lv_color_hex(0x40FF40);
static const lv_color_t color_unit = lv_color_hex(0xC0C0C0);

#if USE_CUSTOM_BOARD
static const lv_font_t* font_Ti = &lv_font_freesans_bold_72;    // row 1 72pt
static const lv_font_t* font_Hi = &lv_font_freesans_bold_48;    // row 2 56pt
static const lv_font_t* font_Pi = &lv_font_freesans_bold_40;    // row 3 40pt
static const lv_font_t* font_Tx = &lv_font_freesans_72;         // row 1 72pt
static const lv_font_t* font_Hx = &lv_font_freesans_56;         // row 2 56pt
static const lv_font_t* font_Px = &lv_font_freesans_40;         // row 3 40pt
static const lv_font_t* font_U  = &lv_font_freesans_20;         // units 28pt
static const lv_font_t* font_AQI = &lv_font_freesans_bold_40;       
#elif USE_MUMA
//static const lv_font_t* font_Ti = &lv_font_freesans_bold_20;    // row 1 20pt
//static const lv_font_t* font_Hi = &lv_font_freesans_bold_20;    // row 2 20pt
//static const lv_font_t* font_Pi = &lv_font_freesans_bold_20;    // row 3 20pt
static const lv_font_t* font_Tx = &lv_font_freesans_72;         // row 1 72pt
static const lv_font_t* font_Hx = &lv_font_freesans_48;         // row 2 48pt
static const lv_font_t* font_Px = &lv_font_freesans_28;         // row 3 28pt
static const lv_font_t* font_U  = &lv_font_freesans_20;         // units 20pt
//static const lv_font_t* font_AQI = &lv_font_freesans_bold_20;       
#else
static const lv_font_t* font_Ti = &lv_font_freesans_bold_72;    // row 1 72pt
static const lv_font_t* font_Hi = &lv_font_freesans_bold_56;    // row 2 56pt
static const lv_font_t* font_Pi = &lv_font_freesans_bold_40;    // row 3 40pt
static const lv_font_t* font_Tx = &lv_font_freesans_72;         // row 1 72pt
static const lv_font_t* font_Hx = &lv_font_freesans_56;         // row 2 56pt
static const lv_font_t* font_Px = &lv_font_freesans_40;         // row 3 40pt
static const lv_font_t* font_U  = &lv_font_freesans_24;         // units 28pt
static const lv_font_t* font_AQI = &lv_font_freesans_bold_40;          
#endif // USE_CUSTOM_BOARD

struct v2c { 
    unsigned value;     // upper limit for this color
    uint32_t color;     // RGB color, argument to lv_color_hex()
};

const v2c AQI_LIMITS[] = {
    {   1,0x00FF00 },{   2,0x87CEEB },{   3,0xFFFF00 },{    4,0xffa500 },{ 5,0xFF0000 }
};
const v2c CO2_LIMITS[] = {
    { 600,0x00FF00 },{ 800,0x87CEEB },{1000,0xFFFF00 },{ 1500,0xffa500 },{ 65000,0xFF0000 }
};
const v2c TVOC_LIMITS[] = {
    { 131,0x00FF00 },{ 436,0x87CEEB },{1307,0xFFFF00 },{ 4358,0xffa500 },{ 65000,0xFF0000 }
//    0.3 mg/m³          1 mg/m³          3 mg/m³          10 mg/m³
// conversion: mg/m³ * 24.45 / 56.1 g/mol * 1000
};

#define N_AQI_COLORS (sizeof(AQI_LIMITS)/sizeof(AQI_LIMITS[0]))


#pragma endregion
//==============================================================================
#pragma region Local variables

#define xINT 0
#define xEXT 1

static lv_obj_t *ui_Screen1, *ui_Cont1;
static lv_obj_t *ui_RowT, *ui_RowH, *ui_RowP, *ui_Row_AQI;
static lv_obj_t* uiTemp[2];
static lv_obj_t* uiHum[2];
static lv_obj_t* uiBaro[2];
static lv_obj_t *unitT, *unitH, *unitP;
static lv_obj_t *uiAQI, *uiTVOC, *uiCO2;

#pragma endregion
//==============================================================================
#pragma region Little helpers

/**
 * @brief Create an LVGL label object
 * 
 * @param parent    pointer to parent object
 * @param color     text color
 * @param align     box alignment, e.g. LV_ALIGN_LEFT_MID
 * @param textalign text alignment, e.g. LV_TEXT_ALIGN_RIGHT
 * @param width_pct box width in percent of parent
 * @param font      which font to use
 * @param text      initial text to display
 * @return lv_obj_t* 
 */
lv_obj_t* createLabel( 
    lv_obj_t* parent, 
    lv_color_t color, 
    lv_align_t align, 
    lv_text_align_t textalign,
    int width_pct,
    const lv_font_t* font, 
    const char* text=NULL
)
{
    lv_obj_t* obj = lv_label_create(parent);
    lv_obj_set_width(obj, LV_PCT(width_pct));   
    lv_obj_set_height(obj, LV_SIZE_CONTENT);  
    lv_obj_set_align(obj, align);
    lv_obj_set_style_text_align(obj, textalign, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (text) lv_label_set_text(obj, text);
    lv_obj_set_style_text_color(obj, color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(obj, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(obj, font, LV_PART_MAIN | LV_STATE_DEFAULT);
    return obj;
}


/**
 * @brief Create a row, i.e. a black rectangle as wide as the parent
 * 
 * @param parent 
 * @return lv_obj_t* 
 */
static lv_obj_t* makeRow( lv_obj_t* parent )
{
    lv_obj_t* row = lv_obj_create(parent);
    lv_obj_remove_style_all(row);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(row, LV_OPA_100, LV_PART_MAIN);
    lv_obj_set_style_bg_color(row, lv_color_hex(0), LV_PART_MAIN | LV_STATE_DEFAULT );   
    return row;
}


/**
 * @brief Hide or unhide LVGL object 
 * 
 * @param obj   the object
 * @param en    true=hidden, false=visible
 */
static void set_hidden(lv_obj_t* obj, bool en)
{
#if LVGL_VERSION_MAJOR == 9
  (en)? lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN) : lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN);
#elif LVGL_VERSION_MAJOR == 8
  if (en)
    lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
  else
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
#endif
}

static inline void lv_hide(lv_obj_t* obj) { set_hidden(obj,true); }
static inline void lv_show(lv_obj_t* obj) { set_hidden(obj,false); }


static uint32_t map_value_to_color( unsigned value, const v2c* v2c_map, size_t nitems )
{
    for (int i=0; i<nitems; i++)
        if (value <= v2c_map[i].value)
            return v2c_map[i].color;
    return v2c_map[nitems-1].color;
}

#define MAP_VALUE_TO_COLOR( v,m ) map_value_to_color( v, m, sizeof(m)/sizeof(m[0]) )

#if HAS_SENSORS
 #define THP_WIDTH 35   // width of temp/hum/baro field in percent
#else
 #define THP_WIDTH 70
#endif

#pragma endregion

/**
 * @brief Initialize LVGL subsystem, define labels
 * 
 * @param disp 
 * @return true 
 * @return false 
 */
bool init_LVGL( lv_disp_t* disp )
{
    unsigned hres = lv_display_get_horizontal_resolution(disp);
    unsigned vres = lv_display_get_vertical_resolution(disp);

    ui_Screen1 = lv_obj_create(NULL);
    ui_Cont1 = lv_obj_create(ui_Screen1);
    lv_obj_remove_style_all(ui_Cont1);
    lv_obj_set_width(ui_Cont1, hres - 20);
    lv_obj_set_height(ui_Cont1, vres - 20);
    lv_obj_set_align(ui_Cont1, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(ui_Cont1, LV_OPA_100, LV_PART_MAIN);
    lv_obj_set_style_bg_color(ui_Cont1, lv_color_hex(0), LV_PART_MAIN | LV_STATE_DEFAULT );   
   
    ui_RowT = makeRow(ui_Cont1);      // temperature
    uiTemp[xEXT] = createLabel( ui_RowT, color_ext, LV_ALIGN_LEFT_MID, LV_TEXT_ALIGN_RIGHT, THP_WIDTH, font_Tx, "-9" );
    unitT = createLabel( ui_RowT, color_unit, LV_ALIGN_CENTER, LV_TEXT_ALIGN_CENTER, 30, font_U, "°C" );
#if HAS_SENSORS
    uiTemp[xINT] = createLabel( ui_RowT, color_int, LV_ALIGN_RIGHT_MID, LV_TEXT_ALIGN_RIGHT, 35, font_Ti, "99" );
#else
    lv_obj_align_to( unitT, uiTemp[xEXT], LV_ALIGN_OUT_RIGHT_MID,0,0 );
#endif

    ui_RowH = makeRow(ui_Cont1);      // humidity
    lv_obj_align_to( ui_RowH, ui_RowT,LV_ALIGN_OUT_BOTTOM_MID,0,0);
    uiHum[xEXT] = createLabel( ui_RowH, color_ext, LV_ALIGN_LEFT_MID, LV_TEXT_ALIGN_RIGHT, THP_WIDTH, font_Hx, "99" );
    unitH = createLabel( ui_RowH, color_unit, LV_ALIGN_CENTER, LV_TEXT_ALIGN_CENTER, 30, font_U, "%rH" );
    lv_obj_set_style_bg_color(unitH, lv_color_hex(0x404040), LV_PART_MAIN | LV_STATE_DEFAULT );   
#if HAS_SENSORS
    uiHum[xINT] = createLabel( ui_RowH, color_int, LV_ALIGN_RIGHT_MID, LV_TEXT_ALIGN_RIGHT, 35, font_Hi, "99" );
#else
    lv_obj_align_to( unitH, uiHum[xEXT], LV_ALIGN_OUT_RIGHT_MID,0,0 );
#endif

    ui_RowP = makeRow(ui_Cont1);      // pressure
    lv_obj_align_to( ui_RowP, ui_RowH,LV_ALIGN_OUT_BOTTOM_MID,0,0);
    uiBaro[xEXT] = createLabel( ui_RowP, color_ext, LV_ALIGN_LEFT_MID, LV_TEXT_ALIGN_RIGHT, THP_WIDTH, font_Px, "999" );
    unitP = createLabel( ui_RowP, color_unit, LV_ALIGN_CENTER, LV_TEXT_ALIGN_CENTER, 30, font_U, "hPa" );
#if HAS_SENSORS
    uiBaro[xINT] = createLabel( ui_RowP, color_int, LV_ALIGN_RIGHT_MID, LV_TEXT_ALIGN_RIGHT, 35, font_Pi, "999" );
#else
    lv_obj_align_to( unitP, uiBaro[xEXT], LV_ALIGN_OUT_RIGHT_MID,0,0 );
#endif 

    ui_Row_AQI = makeRow(ui_Cont1);
    lv_obj_align_to( ui_Row_AQI, ui_RowH,LV_ALIGN_OUT_BOTTOM_MID,0,0);

    lv_obj_t* ui_hline = makeRow(ui_Row_AQI);
    lv_obj_set_size(ui_hline,LV_PCT(100),3);
    lv_obj_set_style_bg_color(ui_hline, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT );   

#if HAS_SENSORS
    lv_obj_t* ui_Row_AQI_label = makeRow(ui_Row_AQI);
    lv_obj_align_to( ui_Row_AQI_label, ui_hline,LV_ALIGN_OUT_BOTTOM_MID,0,5);
    createLabel( ui_Row_AQI_label, color_unit, LV_ALIGN_LEFT_MID, LV_TEXT_ALIGN_CENTER, 33, font_U, "AQI" );
    createLabel( ui_Row_AQI_label, color_unit, LV_ALIGN_CENTER, LV_TEXT_ALIGN_CENTER, 33, font_U, "TVOC" );
    createLabel( ui_Row_AQI_label, color_unit, LV_ALIGN_RIGHT_MID, LV_TEXT_ALIGN_CENTER, 33, font_U, "CO2" );
    lv_obj_t* ui_Row_AQI_value = makeRow(ui_Row_AQI);
    lv_obj_align_to( ui_Row_AQI_value, ui_Row_AQI_label,LV_ALIGN_OUT_BOTTOM_MID,0,0);
    uiAQI  = createLabel( ui_Row_AQI_value, lv_color_hex(0xFFFFFF), LV_ALIGN_LEFT_MID, LV_TEXT_ALIGN_CENTER, 33, font_AQI, "9" );
    uiTVOC = createLabel( ui_Row_AQI_value, lv_color_hex(0xFFFFFF), LV_ALIGN_CENTER, LV_TEXT_ALIGN_CENTER, 33, font_AQI, "99" );
    uiCO2  = createLabel( ui_Row_AQI_value, lv_color_hex(0xFFFFFF), LV_ALIGN_RIGHT_MID, LV_TEXT_ALIGN_CENTER, 33, font_AQI, "999" );
#endif

    lv_hide(ui_RowP);
    lv_hide(ui_Row_AQI);

    lv_disp_load_scr(ui_Screen1);

    return true;
}


void display_update_climate( bool ext, float temp, float hum, float baro )
{
    char buf[10];
    lv_obj_t* obj;
    int index = ext ? xEXT : xINT;
#if !HAS_SENSORS
    if (!ext) return;
#endif

    if (ui_Screen1 && lvgl_port_lock(0)) {
        snprintf(buf,sizeof buf,"%.0f",temp);
        obj = uiTemp[index];
        lv_label_set_text( obj, buf );
        lv_obj_invalidate(obj);

        snprintf(buf,sizeof buf,"%.0f",hum);
        obj = uiHum[index];
        lv_label_set_text( obj, buf );
        lv_obj_invalidate(obj);

        if (baro > 0) {
            snprintf(buf,sizeof buf,"%.0f",baro);
            obj = uiBaro[index];
            lv_label_set_text( obj, buf );
            lv_obj_invalidate(obj);

            lv_hide(ui_Row_AQI);
            lv_show(ui_RowP);
        } else {
            lv_hide(ui_RowP);
        }
        lvgl_port_unlock();
    }
}


void display_update_aqi( int aqi, int tvoc, int eco2 )
{
    uint32_t color;

#if !HAS_SENSORS
    return;
#endif

    if (ui_Screen1 && lvgl_port_lock(0)) {
        lv_hide(ui_RowP);
        lv_show(ui_Row_AQI);
        char buf[10];

        snprintf(buf,sizeof buf,"%d",aqi);
        color = MAP_VALUE_TO_COLOR(aqi,AQI_LIMITS);
        lv_obj_set_style_text_color(uiAQI, lv_color_hex(color), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_label_set_text( uiAQI, buf );
        lv_obj_invalidate(uiAQI);

        snprintf(buf,sizeof buf,"%d",tvoc);
        color = MAP_VALUE_TO_COLOR(tvoc,TVOC_LIMITS);
        lv_obj_set_style_text_color(uiTVOC, lv_color_hex(color), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_label_set_text( uiTVOC, buf );
        lv_obj_invalidate(uiTVOC);

        snprintf(buf,sizeof buf,"%d",eco2);
        color = MAP_VALUE_TO_COLOR(eco2,CO2_LIMITS);
        lv_obj_set_style_text_color(uiCO2, lv_color_hex(color), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_label_set_text( uiCO2, buf );
        lv_obj_invalidate(uiCO2);

        lvgl_port_unlock();
    }
}


void display_update_frame_color( lv_color_t color )
{
    if (ui_Screen1 && lvgl_port_lock(0)) {
        lv_obj_set_style_bg_color(ui_Screen1, color, LV_PART_MAIN | LV_STATE_DEFAULT );
        lv_obj_invalidate(ui_Screen1);
        lvgl_port_unlock();
    }
}


#endif // #if HAS_DISPLAY
