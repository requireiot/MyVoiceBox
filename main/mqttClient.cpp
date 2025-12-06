/**
 * @file 		  mqttClient.cpp
 *
 * Author		: Bernd Waldmann
 * Created		: 23-Sep-2025
 * Tabsize		: 4
 * 
 * This Revision: $Id: mqttClient.cpp 1940 2025-12-03 23:14:38Z  $
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
 * Base topic for publishing defined in call to begin(),
 *  with optional ${HOSTNAME} placeholder replaced by WiFi hostname.
 */

#include <WiFi.h>
#include "mqtt_client.h"    // Apache license 2.0, part of ESP-IDF
#include "esp_event.h"

#include "mqttClient.h"
#include "ansi.h"


#define TAG ANSI_CYAN ANSI_BOLD "MQTT " ANSI_RESET


unsigned SimpleMqttClientESP::_clientNo=0;

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
    log_d(TAG "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    SimpleMqttClientESP* theClient = (SimpleMqttClientESP*) handler_args;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        log_d(TAG "MQTT_EVENT_CONNECTED");
        theClient->_connected=true;
        log_d(TAG "connected to broker");
        break;
    case MQTT_EVENT_DISCONNECTED:
        log_w(TAG ANSI_RED "MQTT_EVENT_DISCONNECTED" ANSI_RESET);
        theClient->_connected=false;
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
        theClient->_on_receive( 
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
 *                  ending with "/", in which case we subscribe to all subtopics.
 * @param pubTopic  top-level topic to publish to, or NULL if none
 * @return true     if connection succeeded
 * @return false    if connection failed
 * 
 * @note  The pattern mazching is primitive, so DO NOT use wildcards in the middle 
 * of the path -- subscribing to "one/two/three/+" is ok, but subscribing to
 * "one/+/three/" is not.
 */
bool SimpleMqttClientESP::begin( const char* pubTopic )
{
    if (pubTopic) {
        _publishTopic = pubTopic;
        _publishTopic.replace("${HOSTNAME}",WiFi.getHostname());
    }

    String clientId = String(WiFi.getHostname())+"_"+String(++_clientNo);
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.credentials.client_id = clientId.c_str();
    mqtt_cfg.broker.address.hostname = _broker;
    mqtt_cfg.broker.address.port = 1883;
    mqtt_cfg.broker.address.transport = MQTT_TRANSPORT_OVER_TCP;
    mqtt_cfg.buffer.size = 2048u;
    mqtt_cfg.buffer.out_size = 512u;
    mqtt_cfg.task = { .stack_size = 4 * 1024, };
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
    unsigned long t1 = millis();
    while (!isConnected()) {
        vTaskDelay(10);
        if ((millis() - t1) > 1000) return false;
    }
    log_i(TAG "connected to '%s' after %u ms", _broker, (unsigned)millis()-t1);
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
    // reset byte counter if this is the first part of a multi-part message, or a complete message
    if (topic) {
        total_bytes_received = 0;
        _multipart_cb = NULL;
    }
    bool multipart = (length < total_length);
    total_bytes_received += length;

    if (topic==NULL) {  // subsequent part of a multipart message
        log_d(TAG "partial, len=%u, sum %u, total %u", 
            length, total_bytes_received, total_length);
        if (_multipart_cb) _multipart_cb ( NULL, (char*)payload, length, total_length );
        return;
    }

    String sTopic(topic,topic_len);
    String sSubtopic = sTopic;

    // lookup the right subtopic
    receive_cb_t cb=nullptr;
    for (auto sub : _subscriptions) {
        if (sTopic.startsWith(sub.prefix)) {
            cb = sub.callback;
            sSubtopic = sTopic.substring(sub.prefix.length());
            while (sSubtopic.startsWith("/")) sSubtopic = sSubtopic.substring(1);
            break;
        }
    }
    if ((_subscriptions.size()>0) && (cb==nullptr)) {
        log_w(TAG "received message for unknown topic '%s'",sTopic.c_str());
        return;
    }

    if (multipart) {        
        /* multipart messages are assumed to be non-string, so don't print the message,
        and don't 0-terminate it  */
        log_d( TAG "receiving large message\n  topic='" ANSI_BLUE "%s" ANSI_RESET,
            topic_buf );
        if (cb) cb(sSubtopic.c_str(),(char*)payload,length,total_length);
        _multipart_cb = cb;
    } else {
        char msg[length+1];
        memcpy(msg,payload,length);
        msg[length] = 0;
        log_i( TAG
            "receive\n  topic='" ANSI_BLUE "%s" ANSI_RESET 
            "'  message='" ANSI_BLUE "%.*s" ANSI_RESET "'",
            sTopic.c_str(), 
            length, msg ? msg : "(null)"
        );
        if (cb) cb(sSubtopic.c_str(),msg,length,total_length);
    }
}


/**
 * @brief Subscribe to messages. Can be called multiple times. 
 * Call this /after/ calling begin().
 * 
 * @param subscribeTopic  topic to subscribe to, can end in + or #
 * @param cb              callback function to receive
 */
void SimpleMqttClientESP::on( const char* subscribeTopic, receive_cb_t cb )
{
    if (!isConnected()) return;

    String topic(subscribeTopic);
    topic.replace("${HOSTNAME}",WiFi.getHostname());
    String prefix = topic;
    while (prefix.endsWith("+")) prefix.remove(prefix.length()-1);
    while (prefix.endsWith("#")) prefix.remove(prefix.length()-1);
    if (topic.endsWith("/")) topic += "#";

    subscription_t sub = { .prefix=prefix, .callback=cb };
    _subscriptions.push_back(sub);

    log_i(TAG "subscribe %d at '%s' to\n '" ANSI_BLUE "%s" ANSI_RESET "'",
        _subscriptions.size(),
        _broker,
        topic.c_str());
    esp_mqtt_client_subscribe_single(_client, topic.c_str(), 0);
}


/**
 * @brief Publish a message to MQTT broker
 * 
 * @param topic       first part of topic for publishing. May contain $(HOSTNAME), 
 *                      which will be replace by WiFi hostname
 * @param subtopic    second part of topic for publishing, may be NULL
 * @param payload     message payload, 0-terminated
 * @param retain      if true, message is retained by broker 
 */
void SimpleMqttClientESP::publish(const char* topic, const char* subtopic, const char* payload, bool retain)
{
	if (!isConnected()) return;
    String sTopic = topic;
    String sSubtopic = subtopic;
    if (
        sTopic.length() > 0
        && sSubtopic.length() > 0 
        && !sTopic.endsWith("/")
        )
        sTopic += "/";
    sTopic += sSubtopic;
    sTopic.replace("${HOSTNAME}",WiFi.getHostname());

    log_d( TAG
        "publish to\n"
        "  topic='" ANSI_BLUE "%s" ANSI_RESET "'\n"
        "  msg='" ANSI_BLUE "%s" ANSI_RESET "'", 
        sTopic.c_str(), 
        payload ? payload : "(null)"
    );

    bool ok = (0 <= esp_mqtt_client_enqueue(_client, sTopic.c_str(), payload, 0, 1, retain, true));
    vTaskDelay(1);
    if (ok) {
	} else {
		log_e(TAG "publish " ANSI_RED "fail !" ANSI_RESET);
	}
}


/**
 * @brief Publish a message to MQTT broker
 * 
 * @param subtopic    topic or subtopic. If _publishTopic was defined in begin(), it is prepended
 * @param payload     message payload, 0-terminated
 * @param retain      if true, message is retained by broker 
 */
void SimpleMqttClientESP::publish(const char* subtopic, const char* payload, bool retain)
{
    publish( _publishTopic.c_str(), subtopic, payload, retain );
}
