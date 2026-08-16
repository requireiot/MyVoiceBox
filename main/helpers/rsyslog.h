#pragma once

extern void init_syslog( const char* servername );

extern void _syslog( esp_log_level_t esp_log_level, const char* fmt, ... );

#define syslog_d(fmt,...) _syslog(ESP_LOG_DEBUG, fmt __VA_OPT__(,) __VA_ARGS__)
#define syslog_i(fmt,...) _syslog(ESP_LOG_INFO, fmt __VA_OPT__(,) __VA_ARGS__)
#define syslog_w(fmt,...) _syslog(ESP_LOG_WARN, fmt __VA_OPT__(,) __VA_ARGS__)
#define syslog_e(fmt,...) _syslog(ESP_LOG_ERROR, fmt __VA_OPT__(,) __VA_ARGS__)
