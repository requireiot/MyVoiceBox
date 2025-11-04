/**
 * @file 		  SimpleMqttClientESP.cpp
 *
 * Author		: Bernd Waldmann
 * Created		: 23-Sep-2025
 * Tabsize		: 4
 * 
 * This Revision: $Id: SimpleMQTTClientESP.cpp 1906 2025-11-01 12:01:24Z  $
 */

/*
   Copyright (C) 2025 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, 
   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
   obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief Simple MQTT client for IoT stuff.
 * Base topic for publishing and base topic for subscribing defined in call to begin(),
 *  with optional ${HOSTNAME} placeholder replaced by WiFi hostname.
 * Behavior when subscribed topic is received can be defined by overriding onSubscribe() method.
 * Publishing to a subtopic of defined base topic is done by calling publish() method.
 * 
 * Usage example:
 * ```
 *  SimpleMqttClientESP myMqttClient("ha-server.local");
 * void setup() {
 *  myMqttClient.begin("home/${HOSTNAME}/set/","home/${HOSTNAME}/get/");
 *  // publish to "home/esp32c3-123456/get/status"
 *  myMqttClient.publish("status","ON");
 * }
 * void loop() {
 *  myMqttClient.loop();
 * }
 * ```
 */

#define USE_ESP_MQTT

#include <WiFi.h>
#include "mqtt_client.h"
#include "esp_event.h"

#include "SimpleMqttClientESP.h"
#include "ansi.h"


#define TAG ANSI_CYAN ANSI_BOLD "MQTT " ANSI_RESET


/**
 * @brief Static method called when an event occurs, such as connect, disconnect, 
 * msg received etc
 * 
 * @param handler_args  Context pointer from call to esp_mqtt_client_register_event()
 * @param base 
 * @param event_id 
 * @param event_data    pointer to mqtt-specific event information
 */
void SimpleMqttClientESP::_on_event(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    bool multipart;

    log_d(TAG "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    SimpleMqttClientESP* simpleClient = (SimpleMqttClientESP*) handler_args;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        log_d(TAG "MQTT_EVENT_CONNECTED");
        simpleClient->_connected=true;
        log_i(TAG "connected to broker");
        simpleClient->do_subscribe();
        break;
    case MQTT_EVENT_DISCONNECTED:
        log_w(TAG ANSI_RED "MQTT_EVENT_DISCONNECTED" ANSI_RESET);
        simpleClient->_connected=false;
        break;
    case MQTT_EVENT_SUBSCRIBED:
        log_d(TAG "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        log_d(TAG "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        log_d(TAG "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        multipart = (event->total_data_len > event->data_len);
        if (multipart) {
            log_d(TAG " MQTT_EVENT_DATA SIZE=%u bytes, total size=%u bytes)", 
                event->data_len, event->total_data_len );
        } else {
            log_d(TAG "MQTT_EVENT_DATA SIZE=%u bytes,\n"
                " TOPIC='" ANSI_BLUE "%.*s" ANSI_RESET "'",
                event->data_len,
                event->topic_len, event->topic
            );
        }
        simpleClient->_on_receive( 
            event->topic, event->topic_len, 
            (byte*) event->data, event->data_len, event->total_data_len
        );
        break;
    case MQTT_EVENT_ERROR:
        log_e(TAG ANSI_RED "MQTT_EVENT_ERROR" ANSI_RESET);
        break;
    default:
        log_d(TAG "Other event id:%d", event_id);
        break;
    }
}


/**
 * @brief start with MQTT client, connect to broker and define topics
 * 
 * @param subTopic  topic to subscribe to when connected, or NULL if none. 
 *                  This can be a full topic like "home/status", or a base topic 
 *                  ending with "/", in which case we subscribe to all subtopics
 * @param pubTopic  top-level topic to publish to, or NULL if none
 * @return true     if connection succeeded
 * @return false    if connection failed
 */
bool SimpleMqttClientESP::begin( const char* subTopic, const char* pubTopic )
{
    if (subTopic) {
        _subscribeTopic = subTopic;
        _subscribeTopic.replace("${HOSTNAME}",WiFi.getHostname());
    }
    if (pubTopic) {
        _publishTopic = pubTopic;
        _publishTopic.replace("${HOSTNAME}",WiFi.getHostname());
    }

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.hostname = _broker;
    mqtt_cfg.broker.address.port = 1883;
    mqtt_cfg.broker.address.transport = MQTT_TRANSPORT_OVER_TCP;
    mqtt_cfg.buffer.size = 4096u;
    bool ok=true;
    _client = esp_mqtt_client_init(&mqtt_cfg);
    ok = ok && (_client!=nullptr);
    ok = ok && (ESP_OK==esp_mqtt_client_register_event(
        _client, 
        (esp_mqtt_event_id_t) ESP_EVENT_ANY_ID, 
        SimpleMqttClientESP::_on_event, 
        this
    ));
    ok = ok && (ESP_OK==esp_mqtt_client_start(_client));
    return ok;
}


/**
 * @brief MQTT subscribe callback, called when mesage arrives. It calls the 
 * _receive_cb() method of the instance. If a subscribe topic was defined in 
 * begin(), it is stripped off the topic parameter before calling _receive_cb().
 * 
 * @param topic     full topic of received message, 0-terminated
 * @param payload   message as a byte string, not 0-terminated
 * @param length    number of bytes in payload
 */
void SimpleMqttClientESP::_on_receive( 
    char* topic, unsigned topic_len,
    byte* payload, unsigned length, 
    unsigned total_length )
{
    static unsigned total_bytes_received = 0;
    if (topic) total_bytes_received = 0;
    bool multipart = (length < total_length);
    total_bytes_received += length;

    if (topic==NULL) {  // part of a multipart message
        log_d(TAG "partial, len=%u, sum %u, total %u", 
            length, total_bytes_received, total_length);
        if (_receive_cb) _receive_cb ( NULL, (char*)payload, length, total_length );
        return;
    }

    // make sure that `topic` is a proper 0-terminated string
    char topic_buf[topic_len+1];
    memcpy(topic_buf,topic,topic_len);
    topic_buf[topic_len]=0;

    char* subtopic = topic_buf;
    if (_subscribeTopic.length()>0) {
        // we have a pre-defined subscribe topic, strip it off
        if (!strncmp(
            topic,
            _subscribeTopic.c_str(),
            _subscribeTopic.length())
            ) {
            subtopic += _subscribeTopic.length();
            if (*subtopic=='/') subtopic++;
            log_i(TAG "subtopic='" ANSI_BLUE "%s" ANSI_RESET "'", subtopic );
        } else {
            log_w(TAG "received message for unknown topic '%s'",topic_buf);
            return;
        }
    }
    /* 
        large messages are assumed to be non-string, so don't print the message,
        and don't 0-terminate it 
    */
    if (!multipart) {        
        char msg[length+1];
        memcpy(msg,payload,length);
        msg[length] = 0;
        log_i( TAG
            "receive\n  topic='" ANSI_BLUE "%s" ANSI_RESET 
            "'  message='" ANSI_BLUE "%.*s" ANSI_RESET "'",
            topic_buf, 
            length, msg ? msg : "(null)"
        );
        if (_receive_cb) _receive_cb(subtopic,msg,length,total_length);
    } else {
        log_i( TAG
            "receive large message\n  topic='" ANSI_BLUE "%s" ANSI_RESET,
            topic_buf
            );
        if (_receive_cb) _receive_cb(subtopic,(char*)payload,length,total_length);
    }
}


/**
 * @brief Subscribe to the topic(s) specified in all to begin(). This is called 
 * from the CONNECTED event
 * 
 */
void SimpleMqttClientESP::do_subscribe()
{
    if (_subscribeTopic.length()>0) {
        String subscribeTopic = _subscribeTopic;
        if (subscribeTopic.endsWith("/")) subscribeTopic += "#";
        log_i(TAG "subscribe to\n '" ANSI_BLUE "%s" ANSI_RESET "'",
            subscribeTopic.c_str());
        esp_mqtt_client_subscribe_single(_client, subscribeTopic.c_str(), 0);
    }
}


/**
 * @brief Publish a message to MQTT broker
 * 
 * @param subtopic    topic or subtopic. If _publishTopic was defined in begin(), it is prepended
 * @param payload     message payload, 0-terminated
 * @param retain      if true, message is retained by broker 
 */
void SimpleMqttClientESP::publish(const char* subtopic,const char* payload, bool retain)
{
	if (!isConnected()) return;
    size_t buflen = strlen(subtopic)+_publishTopic.length()+2;
    char topic[buflen];

    topic[0] = 0;
    if (_publishTopic.length()>0) strncat(topic,_publishTopic.c_str(),buflen-1);
	strncat(topic,subtopic,buflen-1);
    bool ok = (0 <= esp_mqtt_client_publish(_client, topic, payload, 0, 1, retain));
    vTaskDelay(1);
    if (ok) {
        log_d( TAG
            "publish\n  topic='" ANSI_CYAN "%s" ANSI_RESET 
            "'\n  payload='" ANSI_BLUE "%s" ANSI_RESET "'", 
            topic, 
            payload ? payload : "(null)"
        );
	} else {
		log_e(TAG "publish " ANSI_RED "fail !" ANSI_RESET);
	}
}
