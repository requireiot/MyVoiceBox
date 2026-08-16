#pragma once

bool init_Sensors_I2C();
void probe_i2c( void* handle );

bool init_Sensors_BME();
bool init_Sensors_ENS();
bool init_Sensors_AHT();

bool update_BME( float& t, float& h, float& p );
bool update_AHT( float& t, float& h );
bool update_AQI( unsigned& valid, unsigned& aqi, unsigned& tvoc, unsigned& eco2 );
