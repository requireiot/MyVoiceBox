/**
 * @file 		  rsyslog.cpp
 *
 * Author		: Bernd Waldmann
 * Created		: 29-Mar-2026
 * Tabsize		: 4
 * 
 * This Revision: $Id: $
 */

/*
   Copyright (C) 2026 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, 
   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
   obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief Wrapper for library that sends messages to remote syslog server
 */

#include <map>
#include <Arduino.h>
#include <WiFi.h>
#include <PicoSyslog.h>         // LGPL-3.0+ license, https://github.com/mlesniew/PicoSyslog

#include "rsyslog.h"


static PicoSyslog::Logger syslog(
    "esp",
    WiFi.getHostname(),
    PicoSyslog::LogLevel::information,
    &Serial0
);


static const std::map<esp_log_level_t, PicoSyslog::LogLevel> map_loglevel = {
    { ESP_LOG_DEBUG, PicoSyslog::LogLevel::debug },
    { ESP_LOG_INFO, PicoSyslog::LogLevel::information },
    { ESP_LOG_WARN, PicoSyslog::LogLevel::warning },
    { ESP_LOG_ERROR, PicoSyslog::LogLevel::error },
};


/**
 * @brief initialize remote syslog library
 * 
 * @param servername  name of remote syslog server
 */
void init_syslog( const char* servername )
{
    IPAddress log_ip;
    WiFi.hostByName(servername,log_ip);
    syslog.server = log_ip.toString();
    log_i("Connect to SYSLOG server at '%s' (%s)",
        servername,
        log_ip.toString().c_str()
    );
}


/**
 * @brief Helper function to send message to syslog server,
 * with log level specified as an ESP_LOG_xyz constant
 * 
 * @param esp_log_level  one of ESP_LOG_DEBUG, ESP_LOG_INFO etc
 * @param format  printf-style formatting specification
 * @param ...   variable arguments to printf-style function
 */
void _syslog( esp_log_level_t esp_log_level, const char* format, ... )
{
    auto item = map_loglevel.find( esp_log_level );
    PicoSyslog::LogLevel level = 
        (item != map_loglevel.end()) ? item->second : PicoSyslog::LogLevel::information;

    va_list va;
    va_start(va,format);
    char msgbuf[240];
    vsnprintf( msgbuf, sizeof msgbuf, format, va );
    va_end(va);
    syslog.get_stream(level).println(msgbuf);

}
