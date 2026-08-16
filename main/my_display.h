#pragma once
#include "sdkconfig.h"
#include "prefs.h"
#if HAS_DISPLAY

bool init_LVGL( lv_disp_t* disp );
void display_update_frame_color( lv_color_t color );
void display_update_climate( bool ext, float temp, float hum, float pressure=0.0 );
void display_update_aqi( int aqi, int tvoc, int eco2 );

#endif // #if HAS_DISPLAY
