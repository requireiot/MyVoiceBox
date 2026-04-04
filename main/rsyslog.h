/**
 * @file 		  rsyslog.h
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
#pragma once

extern void init_syslog( const char* servername );

extern void _syslog( esp_log_level_t esp_log_level, const char* fmt, ... );

#define syslog_d(fmt,...) _syslog(ESP_LOG_DEBUG, fmt __VA_OPT__(,) __VA_ARGS__)
#define syslog_i(fmt,...) _syslog(ESP_LOG_INFO, fmt __VA_OPT__(,) __VA_ARGS__)
#define syslog_w(fmt,...) _syslog(ESP_LOG_WARN, fmt __VA_OPT__(,) __VA_ARGS__)
#define syslog_e(fmt,...) _syslog(ESP_LOG_ERROR, fmt __VA_OPT__(,) __VA_ARGS__)
