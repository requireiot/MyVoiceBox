/**
 * @file 		  SimpleReports.cpp
 *
 * Author		: Bernd Waldmann
 * Created		: 21-Sep-2025
 * Tabsize		: 4
 * 
 * This Revision: $Id: SimpleReports.cpp 2013 2026-08-16 12:18:31Z  $
 */

/*
   Copyright (C) 2025 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, 
   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
   obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief gather information about CPU, memory, SDK version etc, and either print
 * to serial port or provide as JSON-formatted string, for publishing via MQTT or syslog
 */

#include <map>
#include <Arduino.h>
#include <WiFi.h>           // LGPLv2.1+ license
#include <ArduinoJson.h>    // MIT license, https://github.com/bblanchon/ArduinoJson
#include <esp_system.h>
#include <rom/rtc.h>            // Apache-2.0 license

#include "ansi.h"
#include "SimpleReports.h"


static char msgbuf[256];  // buffer for JSON string

/*
static const char* reset_reasons_32[] = {
"0: none",
"1: Vbat power on reset",
"2: unknown",
"3: Software reset digital core",
"4: Legacy watch dog reset digital core",
"5: Deep Sleep reset digital core",
"6: Reset by SLC module, reset digital core",
"7: Timer Group0 Watch dog reset digital core",
"8: Timer Group1 Watch dog reset digital core",
"9: RTC Watch dog Reset digital core",
"10: Instrusion tested to reset CPU",
"11: Time Group0 reset CPU",
"12: Software reset CPU",
"13: RTC Watch dog Reset CPU",
"14: for APP CPU, reseted by PRO CPU",
"15: VDD voltage is not stable",
"16: RTC Watch dog reset digital core and rtc module",
"17: Time Group1 reset CPU",
"18: super watchdog reset digital core and rtc module",
"19: glitch reset digital core and rtc module",
"20: efuse reset digital core",
"21: USB Uart reset digital core",
"22: USB JTAG reset digital core",
"23: power glitch reset digital core and rtc module"
};

#define NREASONS sizeof(reset_reasons_32)/sizeof(reset_reasons_32[0])
*/

const std::map<esp_reset_reason_t,const char*> ESP32_RESET_REASON_STRINGS = {
    { ESP_RST_UNKNOWN, "unknown" },
    { ESP_RST_POWERON, "power-on" },
    { ESP_RST_EXT, "external pin" },
    { ESP_RST_SW, "Software reset via esp_restart" },
    { ESP_RST_PANIC, "Software reset due to exception/panic" },
    { ESP_RST_INT_WDT, "interrupt watchdog" },
    { ESP_RST_TASK_WDT, "task watchdog" },
    { ESP_RST_WDT, "other watchdogs" },
    { ESP_RST_DEEPSLEEP, "exiting deep sleep" },
    { ESP_RST_BROWNOUT, "Brownout" },
    { ESP_RST_SDIO, "SDIO" },
    { ESP_RST_USB, "USB peripheral" },
    { ESP_RST_JTAG, "JTAG" },
    { ESP_RST_EFUSE, "efuse error" },
    { ESP_RST_PWR_GLITCH, "power glitch detected" },
    { ESP_RST_CPU_LOCKUP, "CPU lock up (double exception)" },
};


const char* getResetReasonString()
{
    esp_reset_reason_t rr = esp_reset_reason();
    auto item = ESP32_RESET_REASON_STRINGS.find(rr);
    if (item != ESP32_RESET_REASON_STRINGS.end())
        return item->second;
    else
        return NULL;
}


void printResetReason( Print& serial )
{
    serial.printf("%s\n", getResetReasonString());
    /*
  	int reset_reason = rtc_get_reset_reason(0); 
    if (reset_reason < NREASONS)
        serial.printf(" Reset reason %s\n",reset_reasons_32[reset_reason]);
    else
        serial.printf(" Reset reason %d (unknown)\n", reset_reason );
    */
}


inline static uint32_t toKB( unsigned nbytes )
{
    return (nbytes + 1023) / 1024;
}


/**
 * @brief print information about chip, SDK version, etc. to a serial port
 * 
 * @param serial    Serial port to print to
 */
void printEnvironment( Print& serial )
{
    // static information about hardware
    serial.printf( " Chip:" ANSI_BOLD "%s" ANSI_RESET, 
        ESP.getChipModel() );
    serial.printf( " at " ANSI_BOLD "%" PRIu32 ANSI_RESET "MHz",
        ESP.getCpuFreqMHz());
    serial.printf("  Flash:" ANSI_BOLD "%" PRIu32 ANSI_RESET "K", 
        toKB(ESP.getFlashChipSize()));
    serial.printf("  PSRAM:" ANSI_BOLD "%" PRIu32 ANSI_RESET "M",
        toKB(ESP.getPsramSize() / 1024u));
    serial.println();

    // static information about build environment
    serial.printf(" SDK:" ANSI_BOLD "%s" ANSI_RESET, 
        ESP.getSdkVersion() );
    serial.printf("  Core:" ANSI_BOLD "%s" ANSI_RESET, 
        ESP.getCoreVersion() );
    serial.printf("  C++" ANSI_BOLD "%d" ANSI_RESET, (int)(__cplusplus/100)-2000 );
    serial.println();
}


/**
 * @brief add information about chip, memory size, SDK version, etc. to a JSON document.
 * Caller may add additional information to the document and then publish via MQTT
 * 
 * @param doc  The JSON structure to add items to
 */
void reportEnvironmentJSON( JsonDocument& doc )
{
    doc["Reset"] = rtc_get_reset_reason(0);
    doc["Chip"] = ESP.getChipModel();
    doc["MHz"] = ESP.getCpuFreqMHz();
    doc["FlashK"] = toKB(ESP.getFlashChipSize());
    doc["Heap"] = ESP.getHeapSize();
    doc["PsramK"] = toKB(ESP.getPsramSize());
    doc["SDK"] = ESP.getSdkVersion();
    doc["Arduino"] = ESP.getCoreVersion();
    doc["C++"] = __cplusplus;
}


/**
 * @brief return a JSON-formatted string with information about chip, memory size, 
 * etc.
 * 
 * @return String 
 */
const char* reportEnvironmentString()
{
    JsonDocument doc;
    reportEnvironmentJSON(doc);
    serializeJson(doc,msgbuf,sizeof(msgbuf));
    return msgbuf;
}


/**
 * @brief print information about memory and stack usage to a serial port
 * 
 * @param serial    Serial port to print to 
 */
void printMemoryInfo( Print& serial, const char* description )
{
    static int32_t old_heap;
    int32_t new_heap = ESP.getFreeHeap();

    if (description) {
        serial.printf(ANSI_REVERSED " %s " ANSI_RESET, description);
        serial.printf(" down " ANSI_RED "%" PRIi32 ANSI_RESET, old_heap-new_heap);
        old_heap = new_heap;
    }
    serial.printf(
        " Heap: free " ANSI_BOLD "%" PRIi32 ANSI_RESET " / total %" PRIu32,
        new_heap, ESP.getHeapSize() );
    serial.printf( 
        "  min free:" ANSI_BOLD "%" PRIu32 ANSI_RESET 
        "  max alloc:" ANSI_BOLD "%" PRIu32 ANSI_RESET,
        ESP.getMinFreeHeap(), ESP.getMaxAllocHeap() );
    serial.printf( 
        "\n  PSRAM: free " ANSI_BOLD "%" PRIu32 ANSI_RESET "K / total %" PRIu32 "K",
        toKB(ESP.getFreePsram()), toKB(ESP.getPsramSize()) );
    serial.printf("\n  Stack watermark:" ANSI_BOLD "%u" ANSI_RESET, 
        (unsigned)uxTaskGetStackHighWaterMark(NULL) );
    serial.println();
}


/**
 * @brief add information about current memory use to a JSON document.
 * Caller may add additional information to the document and then publish via MQTT
 * 
 * @param doc  The JSON structure to add items to
 */
void reportMemoryInfoJSON( JsonDocument& doc )
{
    doc["FreeHeap"] = ESP.getFreeHeap();
    doc["MinFreeHeap"] = ESP.getMinFreeHeap();
    doc["MaxAllocHeap"] = ESP.getMaxAllocHeap();
    doc["FreePsram"] = ESP.getFreePsram();
    doc["StackWM"] = (unsigned)uxTaskGetStackHighWaterMark(NULL);
}


/**
 * @brief return a JSON-formatted string with information about current memory use
 * 
 * @return String 
 */
const char* reportMemoryInfoString()
{
    JsonDocument doc;
    
    reportMemoryInfoJSON(doc);
    serializeJson(doc,msgbuf,sizeof(msgbuf));
    return msgbuf;
}


/**
 * @brief print static information about network connection to a serial port
 * 
 * @param serial  the serial port to print to
 */
void printNetworkInfo( Print& serial )
{
    serial.printf(" MAC:" ANSI_BOLD "%s" ANSI_RESET "  ",
        WiFi.macAddress().c_str());
    serial.printf("IP:" ANSI_BOLD "%s" ANSI_RESET "  ",
        WiFi.localIP().toString().c_str());
    serial.printf("Hostname:'" ANSI_BLUE "%s" ANSI_RESET "'",
        WiFi.getHostname());
    serial.println();
}


/**
 * @brief add information about network connection to a JSON document
 * 
 * @param doc  The JSON structure to add items to
 */
void reportNetworkInfoJSON( JsonDocument& doc )
{
    doc["SSID"] = WiFi.SSID();
    doc["BSSID"] = WiFi.BSSIDstr();
    doc["MAC"] = WiFi.macAddress();
    doc["IP"] = WiFi.localIP().toString();
    doc["Hostname"] = WiFi.getHostname();
}


/**
 * @brief return a JSON-formatted string with information about network connection
 * 
 * @return String 
 */
const char* reportNetworkInfoString()
{
    JsonDocument doc;
    reportNetworkInfoJSON(doc);
    serializeJson(doc,msgbuf,sizeof(msgbuf));
    return msgbuf;
}
