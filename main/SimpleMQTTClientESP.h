/**
 * @file 		  SimpleMqttClientESP.h
 *
 * Author		: Bernd Waldmann
 * Created		: 23-Sep-2025
 * Tabsize		: 4
 * 
 * This Revision: $Id: SimpleMQTTClientESP.h 1916 2025-11-10 11:02:02Z  $
 */

/*
   Copyright (C) 2025 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, 
   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
   obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

#include "mqtt_client.h"
#include "esp_event.h"


class SimpleMqttClientESP {
    typedef void (*receive_cb_t)(const char *, const char*, size_t, size_t);
protected:
    esp_mqtt_client_handle_t _client;
    const char* _broker;
    String _publishTopic;
    String _subscribeTopic;
    receive_cb_t _receive_cb;

    static void _on_event(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
    void _on_receive( 
        char* topic, unsigned topic_len, 
        byte* payload, unsigned payload_len, unsigned total_length );
public:
    bool _connected;

    SimpleMqttClientESP( const char* broker ) 
        : _broker(broker), _receive_cb(nullptr), _connected(false) {}

    bool begin( const char* subscribeTopic=nullptr, const char* publishTopic=nullptr );
    void publish(const char* topic, const char* subtopic,const char* payload, bool retain=false);
    void publish(const char* subtopic,const char* payload, bool retain=false);
    void do_subscribe();
    bool isConnected() const { return _connected; }

    void onReceive( receive_cb_t cb ) { _receive_cb = cb; };
};