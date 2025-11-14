/**
 * @file        main.cpp
 * @project     MyVoiceBox
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2025-10-06
 * tabsize  4
 * 
 * This Revision: $Id: main.cpp 1916 2025-11-10 11:02:02Z  $
 */

/*
   Copyright (C) 2025 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, 
   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
   obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief   Voice interaction satellite for OpenHAB, using ESP-SR and Rhasspy
 * built with ESP-IDF and Arduino-as-a-component
 */

#define USE_SRX             // use modified ESP_SR library
//#define USE_SR_COMMANDS   // define SR commands in call to ESP_SR.begin(), not later

//----- Arduino and libraries
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>         // LGPLv2.1+ license
#include <NTPClient.h>          // MIT license, https://github.com/arduino-libraries/NTPClient
#include <Syslog.h>             // MIT license, https://github.com/arcao/Syslog
#include <ArduinoJSON.h>        // MIT license, https://github.com/bblanchon/ArduinoJson
#include <ArduinoOTA.h>         // Apache license
#include <ESPAsyncWebServer.h>  // LGPL-3.0+ license, https://github.com/ESP32Async/ESPAsyncWebServer
#include <LittleFS.h>           // Apache license
#include "wav_header.h"
#include "esp_littlefs.h"
#include <map>
#include <ESP_I2S.h>
#ifdef USE_SRX
 #include <ESP_SRx.h>
 #define ESP_SR ESP_SRx
#else
 #include <ESP_SR.h>
#endif
#include "sdkconfig.h"

//----- project-specific headers
#include "ansi.h"
#include "myauth.h"
#include "sr_commands.h"
#include "uploadWAV.h"
#include "SimpleMqttClientESP.h"
#include "SimpleOTA.h"
#include "SimpleReports.h"
#include "StatusLED.h"


#define DebugSerial Serial0

const char VERSION[] = 
    "MyVoiceBox $Id: main.cpp 1916 2025-11-10 11:02:02Z  $ built "  __DATE__ " " __TIME__;

//==============================================================================
#pragma region Hardware configuration

#define PIN_BUTTON  0   ///< use the 'BOOT' button on the devkit for simple user input

#define PIN_LED_A   5   // common anode
#define PIN_LED_R   4
#define PIN_LED_G   6
#define PIN_LED_B   7

// Note: constants define here start with lower case 'i2s_' to distinguish them from
// upper case 'I2S_' constants defined in ESP-IDF headers

//----- GPIO pins

#define i2s_TX_PORT     I2S_NUM_1
#define i2s_TX_DOUT     40
#define i2s_TX_BCK      39
#define i2s_TX_LRCK     38

#define i2s_RX_PORT     I2S_NUM_0
#define i2s_RX_DIN      15
#define i2s_RX_BCK      16
#define i2s_RX_LRCK     17

//----- TX data format

#define i2s_TX_STEREO 0
#define i2s_TX_BITS         I2S_DATA_BIT_WIDTH_16BIT

#if i2s_TX_STEREO
 #define i2s_TX_SLOTMODE     I2S_SLOT_MODE_STEREO
 #define TX_NCHANNELS   2
#else
 #define i2s_TX_SLOTMODE     I2S_SLOT_MODE_MONO
 #define TX_NCHANNELS   1
#endif

#if (i2s_TX_BITS == 32)
 typedef int32_t tx_raw_t;
#else
 typedef int16_t tx_raw_t;
#endif

//----- RX data format

#define i2s_RX_STEREO 1
#define i2s_RX_DATA_BIT_WIDTH I2S_DATA_BIT_WIDTH_32BIT     // what the hardware interface produces

#define i2s_RX_BITS     16                                 // what the library delivers
#if (i2s_RX_BITS == 32)
 #define RX_TRANSFORM    I2S_RX_TRANSFORM_NONE
 typedef int32_t rx_raw_t;
#else
 #define RX_TRANSFORM    I2S_RX_TRANSFORM_32_TO_16
 typedef int16_t rx_raw_t;
#endif

#if i2s_RX_STEREO
 #define i2s_RX_SLOTMODE I2S_SLOT_MODE_STEREO                 
 #define i2s_RX_SLOTMASK I2S_STD_SLOT_BOTH
 #define RX_NCHANNELS    2
#else
 #define i2s_RX_SLOTMODE I2S_SLOT_MODE_MONO                 
 #define i2s_RX_SLOTMASK I2S_STD_SLOT_LEFT
 #define RX_NCHANNELS    1
#endif


#pragma endregion
//==============================================================================
#pragma region Preferences


#define SAMPLE_RATE 16000  // sample rate in Hz

// dimensions of TX buffer to feed to ESP-IDF, in samples
#define TX_BUFLEN       1024

#define VOICE_DURATION  5  // seconds

// dimensions of RX buffer to feed to ESP-IDF, in samples
#define RX_BUFLEN       1024

#define MQTT_PUB_BASE "hermes"

//#define CSV_NAME "/oh_sr_commands.csv"

#pragma endregion
//==============================================================================
#pragma region Types


/**
 * @brief Buffer for waveform, allocated in PSRAM
 * 
 */
struct wav_buffer_t {
    int16_t* buf;           ///< points to start of buffer
    int16_t* end;           ///< points past end of buffer
    int16_t* ptr;           ///< points to current position in buffer
    unsigned samplerate;    ///< sample  rate [Hz] to set before playing
    bool alloc( size_t nsamples ) {
        buf = (int16_t*) ps_malloc(nsamples * sizeof(int16_t));
        if (buf==NULL) return false;
        end = buf + nsamples;
        ptr = buf;
        return true;
    };
    void free() {
        if (buf) ::free(buf);
        buf = end = ptr = nullptr;
    };
};


/**
 * @brief Wrapper class for I2SClass so we can mess with incoming audio stream
 * 
 */
class MyRxI2SClass: public I2SClass {
public:
    MyRxI2SClass() : I2SClass() {}
    size_t readBytes(char *buffer, size_t size);
};


#pragma endregion
//==============================================================================
#pragma region Global variables

#define GREEN_OK ANSI_BRIGHT_GREEN "OK " ANSI_RESET

char msgbuf[256];

StatusLED rgbLED( PIN_LED_A, PIN_LED_R, PIN_LED_G, PIN_LED_B );

String sessionId;   // remember the uuid from the MQTT message /hermes/audioServer/$(HOSTNAME)/playBytes/

SR_Commands srCommands;

I2SClass tx_i2s;
MyRxI2SClass rx_i2s;

//----- I2S raw data buffers, these depend on the selected word size
tx_raw_t tx_buf[TX_BUFLEN * TX_NCHANNELS];

// all buffers below are always 16 bit mono, independent of I2S word size

//----- RX buffer for entire recording, in mono 16 bits for a WAV file
wav_buffer_t voice;
size_t   voice_samples; // number of valid samples in voice buffer

wav_buffer_t wav;           // waveform received via MQTT
char* wav_byte_ptr;     // byte-wise pointer into wav.buf during collection
unsigned wav_sample_rate;   // sample rate of the WAV file being collected

wav_buffer_t beep_hello;
wav_buffer_t beep_wake;
wav_buffer_t beep_error;

//----- state indicators, may be set by one task and read by another
volatile bool hasRecord = false;    ///< voice recording complete, ready to be written to FTP
volatile bool isRecording = false;  ///< is currently recording voice from mic or SR input
volatile bool isPlaying = false;    ///< is currently playing a WAV file from memory
volatile bool isCollecting = false; ///< is currently receiving a WAV file via MQTT

//---- network stuff

WiFiUDP udpClient;
NTPClient ntpClient(udpClient);
SimpleMqttClientESP mqttClient( MQTT_BROKER_RHASSPY );
#ifdef SYSLOG_SERVER
 Syslog syslog(udpClient, SYSLOG_SERVER, SYSLOG_PORT, SYSLOG_NILVALUE, SYSLOG_NILVALUE, LOG_USER);
#endif
AsyncWebServer server(80);

#pragma endregion
//==============================================================================
#pragma region Configuration and Statistics

String CSV_lastModified("FFS");
String HTML_lastModified("FFS");

struct stats_t {
    unsigned n_cmds_ok;
    unsigned n_cmds_err;
    void clear() 
        { memset(this,0,sizeof(*this)); }
} stats;


struct config_t {
    unsigned checksum;
    bool opt_playback;
    bool opt_savewave;
    int gain_mqtt;      // gain/attenuation for incoming WAV files, in steps of 6 dB
    int gain_beep;      // gain/attenuation for beeps from flash, in steps of 6 dB
    int gain_mic;       // gain/attenuation for raw mic signal, in steps of 6 dB
    int gain_mic_play;  // gain/atten for raw mic signal, while playing beep

    unsigned calc_checksum() {
        unsigned sum=0xDEADBEEF; 
        uint8_t* p= (uint8_t*)this + sizeof(checksum);
        size_t nbytes=sizeof(config_t)-sizeof(checksum);
        while (nbytes--) sum += *p++;
        return sum;
    }
    void set_checksum() { checksum=calc_checksum(); }
    bool is_valid() { return checksum==calc_checksum(); }
    config_t& operator=(const config_t& rhs)
        { memcpy(this,&rhs,sizeof(*this)); checksum=calc_checksum(); return *this; }
};


config_t default_config = {
    .opt_playback = false,
    .opt_savewave = false,
    .gain_mqtt = -1,
    .gain_beep = -1,
    .gain_mic  =  3,
    .gain_mic_play = -3,
};


RTC_DATA_ATTR config_t config;


struct conv_t {
    String (*toWebString)();
    void (*toValue)(const AsyncWebParameter* wp);
};

/**
 * @brief For each config item, define how to convert to/from web page
 * Template names are uppercase like %PLAYBACK%
 * parameter names in HTTP request are lowercase, like http://host/set?playback=OFF
 * 
 */
const std::map<const std::string, conv_t> config_mapper = {
    { "playback", {
        []() { return String( config.opt_playback ? "checked" : "" ); },
        [](const AsyncWebParameter* wp) { config.opt_playback = wp->value().equals("ON"); }
    }},
    { "savewave", {
        []() { return String( config.opt_savewave ? "checked" : "" ); },
        [](const AsyncWebParameter* wp) { config.opt_savewave = wp->value().equals("ON"); }
    }},
    { "wavgain", {
        []() { return String( config.gain_mqtt); },
        [](const AsyncWebParameter* wp) { config.gain_mqtt = wp->value().toInt(); }
    }},
    { "beepgain", {
        []() { return String( config.gain_beep); },
        [](const AsyncWebParameter* wp) { config.gain_beep = wp->value().toInt(); }
    }},
    { "micgain", {
        []() { return String( config.gain_mic); },
        [](const AsyncWebParameter* wp) { config.gain_mic = wp->value().toInt(); }
    }},
    { "micatten", {
        []() { return String( config.gain_mic_play); },
        [](const AsyncWebParameter* wp) { config.gain_mic_play = wp->value().toInt(); }
    }},
    { "ncmdsok", {
        []() { return String( stats.n_cmds_ok ); },
        NULL
    }},
    { "ncmdserr", {
        []() { return String( stats.n_cmds_err ); },
        NULL
    }},
    { "ncommands", {
        []() { return String(srCommands.count()); },
        NULL
    }},
    { "version", {
        []() { return String(VERSION); },
        NULL
    }},
    { "csv_version", {
        []() { return CSV_lastModified; },
        NULL
    }},
    { "html_version", {
        []() { return HTML_lastModified; },
        NULL
    }},
    { "hostname", {
        []() { return String(WiFi.getHostname()); },
        NULL
    }},
};


#pragma endregion
//==============================================================================
#pragma region Little helpers


/**
 * @brief initialize LittleFS flash file system
 * 
 * @return true     all ok, found expected partition
 * @return false    some error
 */
bool init_FS() 
{
    return LittleFS.begin(false,"/littlefs",10,"littlefs");
/*
    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/littlefs",
        .partition_label = "littlefs",
        .format_if_mount_failed = false,
        .dont_mount = false,
    };
    return ESP_OK==esp_vfs_littlefs_register(&conf);
*/
}


/**
 * @brief Find smallest and largest value in array of samples
 * 
 * @param data      points to array of 16-bit mono samples
 * @param nsamples  number of samples
 * @return  const char* formatted like '[min..max]'
 */
template<typename T>
const char* calc_min_max( const T* data, size_t nsamples )
{
    T dmin=INT16_MAX, dmax=INT16_MIN;
    while (nsamples--) {
        if (dmin > *data) dmin = *data;
        if (dmax < *data) dmax = *data;
        data++;
    }
    static char s[40];
    snprintf( s, sizeof s, "[%6ld..%6ld]", (long)dmin, (long)dmax);
    return s;
}


/**
 * @brief Amplify or attenuate audio samples
 * 
 * @param buffer    pointer to int16_t samples
 * @param nsamples  number of samples
 * @param shifts    gain, 1=+6 dB, 2=+12 dB, ... -1=-6 dB, -2=-12 dB, ...
 */
void amplify( int16_t* buffer, size_t nsamples, int shifts )
{
    int16_t* wavp = buffer;
    if (shifts > 0) {
        while (nsamples--)
            *wavp++ <<= shifts;        
    } else if (shifts < 0) {
        shifts = -shifts;
        while (nsamples--)
            *wavp++ >>= shifts;        
    }
}


#pragma endregion
//==============================================================================
#pragma region I2S transmit (using Arduino library)


bool i2s_tx_initialize() 
{
    tx_i2s.setPins( i2s_TX_BCK, i2s_TX_LRCK, i2s_TX_DOUT);
    bool ok = tx_i2s.begin( I2S_MODE_STD, SAMPLE_RATE, i2s_TX_BITS, i2s_TX_SLOTMODE );
    log_i(GREEN_OK "Init I2S Tx" );
    tx_i2s.setTimeout( 1000 * TX_BUFLEN / SAMPLE_RATE + 10 );
    return ok;
}


bool i2s_tx_set_sample_rate( unsigned sample_rate )
{
    log_i("Set I2S TX sample rate to %u Hz", sample_rate);
    return tx_i2s.configureTX( sample_rate, i2s_TX_BITS, i2s_TX_SLOTMODE );
}


bool i2s_tx_write( tx_raw_t* data, size_t bytes_to_write )
{
    size_t bytes_written = tx_i2s.write((uint8_t*)data, bytes_to_write);
    if (bytes_written != bytes_to_write) {
        log_e("Write Task: i2s write failed, expected %u, got %u",
            bytes_to_write, bytes_written);
        return false;
    }
    return true;
}


#pragma endregion
//==============================================================================
#pragma region I2S TX task, play buffers


int16_t* play_buf;
int16_t* play_end;
int16_t* play_ptr;


void start_playing( int16_t* buf, int16_t* end, unsigned samplerate = SAMPLE_RATE )
{
    i2s_tx_set_sample_rate( samplerate );
    play_buf = buf;
    play_ptr = buf;
    play_end = end;
    isPlaying = true;
    unsigned nsamples = play_end - play_buf;
    log_i("start playing, %u samples (%u s) %s", 
        nsamples, (unsigned) nsamples / samplerate, calc_min_max(buf,nsamples) );
}


void start_playing( wav_buffer_t& wv ) 
{
    start_playing( wv.buf, wv.end, wv.samplerate );
}


void publishFinished();

void stop_playing()
{
    isPlaying = false;
    play_ptr = play_buf;
    log_i("stop playing");
    publishFinished();
}


/**
 * @brief Fill 'nsamples' worth of tx buffer from play buffer
 * 
 * @param data      pointer to tx buffer (16-bit stereo)
 * @param nsamples  number of samples to copy to Tx buffer
 * @return true     if more samples available
 * @return false    if end of buffer reached
 */
bool fill_tx_buffer_from_play( tx_raw_t* data, size_t nsamples )
{
    size_t samples_available = play_end - play_ptr;
    size_t samples_to_copy = min( samples_available, nsamples );
    for (size_t i=0; i < samples_to_copy; i++) {
#if i2s_TX_STEREO
        *data++ = *play_ptr;   // channel 0
#endif
        *data++ = *play_ptr++;   // channel 1
    }
    nsamples -= samples_to_copy;
    if (nsamples) { 
        memset( data, 0, nsamples * sizeof(int16_t) * TX_NCHANNELS );
        //return false;
    }
    return (samples_available > 0);//true;
}


/**
 * @brief Task to write to I2S speaker, from buffer in PSRAM 
 * 
 * @param userData 
 */
void i2s_tx_task( void* userData )
{
    size_t bytes_to_write = TX_BUFLEN * TX_NCHANNELS * sizeof(tx_raw_t);

    while (1) {
        if (isPlaying) {
            bool ok = fill_tx_buffer_from_play( tx_buf, TX_BUFLEN );
            if (!ok) stop_playing();
        } else {
            memset( tx_buf, 0, sizeof tx_buf );
        }
        // Write i2s data
        i2s_tx_write( tx_buf, bytes_to_write );
        vTaskDelay(1);
    }
}


#pragma endregion
//==============================================================================
#pragma region State management


enum state_t { 
    stUnknown=-1,
    stIdle=0, 
    stDetected, 
    stCommandOk, 
    stCommandError,
    stSystemError
};
volatile state_t state = stIdle;
volatile unsigned long t_statechanged=0;
uint32_t t_recording_start=0;


void setState( state_t new_state )
{
    static state_t old_state = stUnknown;
    if ((new_state != old_state) && (new_state != stIdle))
        t_statechanged = millis();
    state = old_state = new_state;
}


void loopState()
{
    if (((unsigned)(millis() - t_statechanged) > 3000) && (state >= stCommandOk))
        state = stIdle;
}


void start_recording()
{
    voice.ptr = voice.buf;
    voice_samples = 0;
    isRecording = true;
    t_recording_start = millis();
    log_i("Start recording");
    
}


void stop_recording()
{
    if (!isRecording) return;
    voice_samples = voice.ptr - voice.buf;
    voice.ptr = voice.buf;
    isRecording = false;
    hasRecord = true;
    log_i("Stop recording after %u ms, received %u samples", 
        millis()-t_recording_start,
        (unsigned)voice_samples
    );
}


void start_playback()
{
    log_i("Start playback, %u samples %s", 
        unsigned(voice_samples), calc_min_max(voice.buf, voice_samples));
    start_playing( voice.buf, voice.buf + voice_samples, SAMPLE_RATE );
}


#pragma endregion
//==============================================================================
#pragma region RGB LED stuff


static void update_LED_mode()
{
    switch (state) {
        case stIdle:        // Blue
            rgbLED.set_color(0, 0, RGB_BRIGHTNESS); 
            rgbLED.set_mode( StatusLED::MODE_BREATHE );
            break; 
        case stDetected:    // Yellow
            rgbLED.set_color(RGB_BRIGHTNESS, RGB_BRIGHTNESS, 0 ); 
            rgbLED.set_mode( StatusLED::MODE_BLINK );
            break; 
        case stCommandOk:   // Green
            rgbLED.set_color(0, RGB_BRIGHTNESS, 0); 
            rgbLED.set_mode( StatusLED::MODE_ON );
            break; 
        case stCommandError:  // Red
            rgbLED.set_color(RGB_BRIGHTNESS, 0, 0); 
            rgbLED.set_mode( StatusLED::MODE_ON );
            break; 
        case stSystemError:
            rgbLED.set_color(255,0,0);
            rgbLED.set_mode(StatusLED::MODE_BLINK);
            break;
        default: 
            rgbLED.set_color(32, 32, 32); 
            rgbLED.set_mode( StatusLED::MODE_BLINK );
            break;
    }    
}


static void led_hello()
{ 
    rgbLED.set_color(RGB_BRIGHTNESS, 0, 0); 
    delay(1000);
    rgbLED.set_color(0, RGB_BRIGHTNESS, 0); 
    delay(1000);
    rgbLED.set_color(0, 0, RGB_BRIGHTNESS); 
    delay(1000);
    rgbLED.set_color(0, 0, 0);    
}


void led_task( void* userData )
{
    static state_t old_state = stUnknown;

    led_hello();

    while (1) {
        rgbLED.tick();
        if (state != old_state) {
            old_state = state;
            update_LED_mode();
        }
        vTaskDelay(StatusLED::MS_PER_TICK);
    }
}


void init_LED()
{
    xTaskCreate(led_task, "led_task", 4000, NULL, 1, NULL); 
}


#pragma endregion
//==============================================================================
#pragma region I2S receive (using Arduino library)


int16_t sr_rx_min_L=0, sr_rx_max_L=0;
int16_t sr_rx_min_R=0, sr_rx_max_R=0;
unsigned long esp_sr_fill_counter;

bool copy_buf_to_voice( rx_raw_t* data, size_t nbytes );


static void _sr_rx_process( char* out, size_t nbytes )
{
    size_t nsamples = nbytes / (sizeof(int16_t) * RX_NCHANNELS);
/*
    int16_t y;
    int16_t* p16 = (int16_t*) out;
    while (nsamples--) {
        y = *p16++;
        if (y < sr_rx_min_L) sr_rx_min_L = y;
        if (y > sr_rx_max_L) sr_rx_max_L = y;
#if (RX_NCHANNELS == 2)
        y = *p16++;
        if (y < sr_rx_min_R) sr_rx_min_R = y;
        if (y > sr_rx_max_R) sr_rx_max_R = y;
#endif
    }
*/
    int ampshift = isPlaying ? config.gain_mic_play : config.gain_mic;   // amplify mic unless beep is playing
    nsamples = nbytes / sizeof(int16_t);
    amplify( (int16_t*)out, nsamples, ampshift );
}


size_t MyRxI2SClass::readBytes(char *buffer, size_t size)
{
    size_t bytes_read = I2SClass::readBytes(buffer,size);
    _sr_rx_process( buffer, size );
    if (isRecording) {
        bool ok = copy_buf_to_voice( (rx_raw_t*)buffer, size);
        if (!ok) stop_recording();
    }
    esp_sr_fill_counter++;
    return bytes_read;
}


/**
 * @brief initialize I2S interface for receiving from microphone(s)
 * 
 * @return true     init worked
 * @return false    some error
 */
bool i2s_rx_initialize() 
{
    rx_i2s.setPins( i2s_RX_BCK, i2s_RX_LRCK, -1, i2s_RX_DIN );
    rx_i2s.setTimeout(1000);

    bool ok = rx_i2s.begin( 
                    I2S_MODE_STD, 
                    SAMPLE_RATE, 
                    i2s_RX_DATA_BIT_WIDTH, 
                    i2s_RX_SLOTMODE, 
                    i2s_RX_SLOTMASK 
                );
    // this always sets .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(...)
    if (ok) log_i(GREEN_OK "Init I2S Rx");

    ok = rx_i2s.configureRX(SAMPLE_RATE, i2s_RX_DATA_BIT_WIDTH, i2s_RX_SLOTMODE, RX_TRANSFORM);
    if (ok) log_i(GREEN_OK "config I2S Rx");

    rx_i2s.setTimeout( 1000 * RX_BUFLEN / SAMPLE_RATE + 10 );

    return ok;
}


#pragma endregion
//==============================================================================
#pragma region Voice recording and playback


/**
 * @brief Prepare for voice recording, allocate a buffer in PSRAM large enough
 * to hold `VOICE_DURATION` seconds of audio
 * 
 * @return true  if successful
 * @return false 
 */
bool initVoiceRecorder()
{
    if (!voice.alloc(VOICE_DURATION * SAMPLE_RATE)) {
        log_e("Not enough free PSRAM, need %d, have %d", 
            VOICE_DURATION * SAMPLE_RATE * sizeof(int16_t), 
            ESP.getFreePsram() );
        return false;
    } 
    log_d("allocated voice buffer, %d bytes", 
        VOICE_DURATION * SAMPLE_RATE * sizeof(int16_t));
    voice.samplerate = SAMPLE_RATE;
    return true;
}


#if (i2s_RX_BITS == 32)
 // use top 16 bit
 #define rx_to_16(p) (p >> 16)
#else
 #define rx_to_16(p) (p)
#endif

/**
 * @brief Copy a chunk of raw I2S microphone data to voice buffer (appending)
 * 
 * @param data      points to raw I2S microphone data
 * @param nbytes    number of bytes in buffer
 * @return true     if voice buffer has room for more data
 * @return false    if voice buffer is full
 */
bool copy_buf_to_voice( rx_raw_t* data, size_t nbytes )
{
    size_t nsamples = nbytes / (RX_NCHANNELS * sizeof(rx_raw_t));
    int16_t y;

    size_t n_free_samples = voice.end - voice.ptr;
    if (n_free_samples < nsamples) return false;

    while (nsamples--) {
        y = rx_to_16(data[0]);
        //y <<= 3;   // amplify 18dB
        *voice.ptr++ = y;
        data += RX_NCHANNELS;
        if (voice.ptr==voice.end) {
            isRecording = false;
            return false;
        }
    }
    return true;
}


bool saveWave( const char* namebase, const int16_t* data, size_t nsamples )
{
#ifdef FTP_SERVER
    time_t epoch = ntpClient.getEpochTime();
    struct tm *tmnow = localtime(&epoch);
    char s_now[20]; // "20241231T235901Z"
    strftime(s_now, sizeof s_now, "%Y%m%dT%H%M%SZ", tmnow);
    char filename[100];
    snprintf(filename,sizeof filename,"%s_%s.WAV",namebase,s_now);
    log_i("write wave to '" ANSI_BLUE "%s" ANSI_RESET "' (%u samples)",
        filename, nsamples);
    log_i("%s", calc_min_max( data, nsamples ));    
    return uploadWave( 
        FTP_SERVER, FTP_USER, FTP_PASS,
        filename, 
        data, nsamples, 
        SAMPLE_RATE 
    );
#else
    log_w("WAV file not saved, FTP_SERVER is not defined")
#endif // FTP_SERVER
}


#pragma endregion
//==============================================================================
#pragma region Button stuff


void init_Button()
{
    pinMode( PIN_BUTTON, INPUT_PULLUP );
}


void poll_Button()
{
    static int old_press = HIGH;
    int new_press = digitalRead(PIN_BUTTON);

    if (new_press != old_press) {
        old_press = new_press;
        if (new_press==HIGH) {
            sr_rx_min_L = sr_rx_max_L = sr_rx_min_R = sr_rx_max_R = 0;
        }
    }
}


#pragma endregion
//==============================================================================
#pragma region Reporting stuff to UART


void printSRinfo( Print& serial )
{
    const char* wwname =
#if CONFIG_SR_WN_WN9_ALEXA
        "WN9 Alexa";
#elif CONFIG_SR_WN_WN9_HIESP
        "WN9 HiESP";
#else
        "unknown";
#endif

    serial.printf(" I2S Audio: %d Hz, TX %d bits %s, RX %d/%d bits %s", 
        SAMPLE_RATE, 
        (int)i2s_TX_BITS,
        i2s_TX_STEREO ? "stereo" : "mono",
        (int)i2s_RX_DATA_BIT_WIDTH, (int)i2s_RX_BITS,
        i2s_RX_STEREO ? "stereo" : "mono"
        );
    serial.println();
    serial.printf(" Wakeword: " ANSI_BOLD "%s" ANSI_RESET , wwname );
    serial.println();
}


/**
 * @brief Report current FreeRTOS tasks
 * 
 */
void reportTasks()
{
#if CONFIG_ARDUHAL_LOG_DEFAULT_LEVEL > 3
    /*
    unsigned ntasks = uxTaskGetNumberOfTasks();
    TaskStatus_t status[ntasks];
    unsigned n = uxTaskGetSystemState( status, ntasks, NULL );
    */
    DebugSerial.printf( "\nTask Name\tStatus\tPrio\tHWM\tTask\tAffinity\n");
    DebugSerial.printf( "---------\t-------\t-------\t-------\t-------\t--------\n");
    char stats_buffer[1024];
    vTaskList(stats_buffer);
    DebugSerial.printf("%s\n", stats_buffer);
    DebugSerial.println();
#endif
}


#pragma endregion
//==============================================================================
#pragma region WiFi stuff


#define IF_NAME ANSI_MAGENTA "WiFi " ANSI_RESET

void onWiFiEvent(WiFiEvent_t event) 
{
	switch (event) {
		
	case ARDUINO_EVENT_WIFI_STA_START:
		log_i( IF_NAME "started, hostname is '" ANSI_BOLD "%s" ANSI_RESET "'",
            WiFi.getHostname());
		break;
		
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
		log_i( IF_NAME ANSI_BRIGHT_GREEN "connected" ANSI_RESET );
		break;
		
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
		log_i( IF_NAME "got IP: " ANSI_BOLD "%s" ANSI_RESET , 
            WiFi.localIP().toString().c_str() );
		break;
		
    case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
        log_i( IF_NAME "got IPv6" );
        break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        // try to re-connect
        //WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
        // fall-thru
		log_e( IF_NAME ANSI_BRIGHT_RED "Disconnected" ANSI_RESET);
		break;
		
    case ARDUINO_EVENT_WIFI_STA_STOP:
		log_i( IF_NAME "stopped");
		break;
		
    case ARDUINO_EVENT_WIFI_READY:
        log_i( IF_NAME "ready");
        break;

	default:
        log_i("event = %d", (int)event);
		break;
	}
}


/**
 * @brief Connect to WiFi AP
 * 
 * @return true  if connection successful
 */
bool setupWifi()
{
    WiFi.onEvent(onWiFiEvent);
    WiFi.mode(WIFI_STA);
    WiFi.begin( WIFI_SSID, WIFI_PASSWORD );
    uint8_t wf = WiFi.waitForConnectResult(); 
    if (wf==WL_CONNECTED) {
        return true;
    } else {
        log_e( ANSI_RED "WiFi connect failed, status=%d" ANSI_RESET, (int)wf);
        return false;
    }    
}


/**
 * @brief Refresh WiFi connection if necessary
 * 
 * @return true  WiFi is now connected
 * @return false  WiFi is not connected
 */
bool loopWifi()
{
    if (WiFi.status() != WL_CONNECTED) {
        delay(1);
        setupWifi();
    }
    return WiFi.isConnected();
}


#pragma endregion
//==============================================================================
#pragma region Receive WAV file via MQTT and play


/**
 * @brief allocate buffer and start collecting a WAV file received via MQTT, 
 * 
 * @param nbytes      total length in bytes specified in WAV header
 * @param sample_rate sample rate in Hz specified in WAV header
 */
void start_collecting( unsigned nbytes, unsigned sample_rate )
{
    log_i("Start collecting WAV file, %u bytes at %d Hz", nbytes, sample_rate );
    wav.free();
    if (!wav.alloc(nbytes/sizeof(int16_t))) {
        log_e(ANSI_RED "Failed to allocate %u bytes for WAV buffer" ANSI_RESET, 
            nbytes );
        return;
    }
    log_i("Allocated %u bytes for WAV buffer", nbytes );
    wav_byte_ptr = (char*) wav.buf;
    wav.samplerate = sample_rate;
    isCollecting = true;
}


/**
 * @brief end collecting WAV file, and start playback
 * 
 */
void stop_collecting()
{
    // complete WAV file received, now playback
    isCollecting = false;
    wav.end = (int16_t*) wav_byte_ptr;
    unsigned nsamples = (unsigned)(wav.end - wav.buf);
    log_i("received complete WAV file, %u samples, %s",
        nsamples, calc_min_max( wav.buf, nsamples ));
    if (config.gain_mqtt)
        amplify( wav.buf, nsamples, config.gain_mqtt );
    start_playing( wav );
}


/**
 * @brief Add a chunk of data to the WAV buffer being collected
 * 
 * @param message   pointer to data chunk
 * @param length    length of data chunk in bytes, may be odd
 */
void collect_wav_bytes( const char* message, size_t length )
{
    memcpy( wav_byte_ptr, message, length );
    wav_byte_ptr += length;
}


#pragma endregion
//==============================================================================
#pragma region MQTT stuff


/*
    Rhasspy will also send the following messages:

    When wakeword has been detected
        topic = hermes/asr/startListening
        payload = {"siteId": "raspi17", ... more stuff ... }        

    When command has been recognized
        topic = hermes/nlu/intentParsed
        payload = 
            {
            "input": "turn OFF N115_Proxy", 
            "intent": {
                "intentName": "switch", 
                ... more stuff ...
            }, 
            "siteId": "raspi17", 
            ... more stuff ... 
            "slots": [
                {
                    "entity": "state", 
                    "value": {"kind": "Unknown", "value": "OFF"}, 
                    ... more stuff ...
                    "rawValue": "off", 
                    ... more stuff ...
                }, 
                {
                    "entity": "oh_items", 
                    "value": {"kind": "Unknown", "value": "N115_Proxy"}, 
                    ... more stuff ...
                    "rawValue": "right hallway floorlight", 
                    ... more stuff ...                
                }
            ], 
            ... more stuff ...
            }

    When timeout has occured
        topic = 
        payload = {"input": "", "siteId": "raspi17", ... more stuff ... }

    When WAV playback has finished, the *satellite* publishes
        topic = hermes/audioServer/_siteId_/playFinished
        payload = 
            {
                "id": "the-subtopic-name-from-the-playBytes-command", 
                "sessionId": "the-subtopic-name-from-the-playBytes-command"}
            }
*/

/// Format of MQTT message, subset of what Rhasspy would produce
// 1st arg is hostname, 2nd is value, 3rd is itemname, 4th is label, 5th is raw text
const char* hermes_intent = R"rawliteral({
 "siteId": "%s", 
 "slots": [
  {"entity": "state","value":{"value":"%s"}}, 
  {"entity": "oh_items","value":{"value":"%s"},"rawValue":"%s", }
 ], 
 "rawInput": "%s", 
})rawliteral";

/// MQTT message "wakeword detected", subset of what Rhasspy would produce
// 1st arg is hostname
const char* hermes_wakeword = R"rawliteral({"siteId": "%s"})rawliteral";
#define TOPIC_WAKEWORD_DETECTED "hermes/asr/startListening"

/// MQTT message "timeout occurred", subset of what Rhasspy would produce
// 1st arg is hostname
const char* hermes_timeout = R"rawliteral({"siteId": "%s"})rawliteral";
#define TOPIC_TIMEOUT_OCCURRED "hermes/nlu/intentNotRecognized"

/// MQTT message "playback finished"
const char* hermes_playFinished = R"rawliteral({"id": "%s", "sessionId": "%s"})rawliteral";
#define TOPIC_PLAYFINISHED "hermes/audioServer/${HOSTNAME}/playFinished"


void publishFinished()
{
    if (sessionId.length() > 0) {
        const char* id = sessionId.c_str();
        snprintf(msgbuf,sizeof(msgbuf),hermes_playFinished,id,id);
        sessionId = "";
        mqttClient.publish(TOPIC_PLAYFINISHED,NULL,msgbuf);
    }
}


/**
 * @brief Publish a Rhasspy-esque JSON string with info about command `id`
 * 
 * @param id   command id, starts with 1
 */
void publish_command( int id )
{
    const command_info_t* pinfo = srCommands[id];
    if (pinfo==NULL) {
        log_e("unknown command %d",id);
    } else {
        constexpr size_t MSG_BUFLEN = 500;
        char* msg = (char*) malloc(MSG_BUFLEN);
        snprintf( msg, MSG_BUFLEN, hermes_intent,
            WiFi.getHostname(), 
            pinfo->value,
            pinfo->itemname,
            pinfo->label,
            pinfo->grapheme
        );
        log_i("publish id %d to '%s':\n" ANSI_BLUE "%s" ANSI_RESET, 
            id, pinfo->action, msg);
        mqttClient.publish( pinfo->action, msg );
        free(msg);
    }
}


/**
 * @brief Process message received via MQTT
 * 
 * @param topic     subtopic (0-terminated) or NULL for multi-part message
 * @param message   payload, 0-terminated unless it is part of a multi-part message
 * @param length    number of bytes in this chunk
 * @param total_length  total number of bytes in multi-part message
 */
void onReceive( 
    const char* topic, 
    const char* message, 
    size_t length, 
    size_t total_length )
{
    static unsigned total_bytes_received;
    if (topic) total_bytes_received = 0;    // first part in a multi-part message
    total_bytes_received += length;

    if (topic==NULL) {  // part of a multi-part message
        if (isCollecting) {
            log_d("Collecting %u bytes, now %u", length, total_bytes_received );
            collect_wav_bytes( message, length );
            if (total_bytes_received >= total_length)
                stop_collecting();
        }
        return;
    }
    if (message && *message) {
        log_i("MQTT received topic '" ANSI_BLUE "%s" ANSI_RESET "'", topic );
        sessionId = topic;
        pcm_wav_header_t* pWAV = (pcm_wav_header_t*) message;
        unsigned ms = 1000u * pWAV->data_chunk.subchunk_size / pWAV->fmt_chunk.byte_rate;
        log_i("receiving WAV: %u bytes, %d Hz, %d bits, %d channels (%u ms)", 
            pWAV->data_chunk.subchunk_size,
            pWAV->fmt_chunk.sample_rate,
            pWAV->fmt_chunk.bits_per_sample,
            pWAV->fmt_chunk.num_of_channels,
            ms
        );
        if (pWAV->fmt_chunk.bits_per_sample==16 
            && pWAV->fmt_chunk.num_of_channels==1 
            && !isCollecting
            && !isPlaying
            ) {
            // ok, let's start collecting data, only if we are not currently playing something else
            start_collecting( pWAV->data_chunk.subchunk_size, pWAV->fmt_chunk.sample_rate );
            if (isCollecting) {
                collect_wav_bytes( message + sizeof(pcm_wav_header_t), length-sizeof(pcm_wav_header_t) );
            }
        }
    }
}


/**
 * @brief Connect to MQTT broker and subscribe to topics
 * 
 * @return true     everything ok
 * @return false    some error
 */
bool initMQTT()
{
    bool ok = mqttClient.begin( "hermes/audioServer/${HOSTNAME}/playBytes/", "hermes/intent/" );
    if (ok) {
        mqttClient.onReceive( onReceive );
    }
    return ok;
}


#pragma endregion
//==============================================================================
#pragma region Download from HTTP server


bool download_from_HTTP( const String& url, String& content, String& modified)
{
    String s;
    HTTPClient httpClient;
    httpClient.begin( url );
    const char* headerNames[] = { "Last-Modified" };
    httpClient.collectHeaders(headerNames,1);
    int res = httpClient.GET();
    if (res==200) {
        content = httpClient.getString();
        modified = httpClient.header("Last-Modified");
        log_i("downloaded \n'" ANSI_BLUE "%s" ANSI_RESET "' (%s)", url.c_str(), modified.c_str() );
        return true;
    } else {
        log_e(ANSI_RED "failed to download \n'" ANSI_BLUE "%s" ANSI_RESET "' (%s)", url.c_str(), modified.c_str() );        
    }
    return false;
}


bool download_and_store( const String& baseURL, const String& filename, String& content, String& modified )
{
    if (download_from_HTTP( baseURL+filename, content, modified)) {
        File f = LittleFS.open( ("/"+filename).c_str(), "w");
        if (f) {
            f.print(content);
            f.close();
            log_i("Wrote '" ANSI_BLUE "%s" ANSI_RESET "' to FFS", filename.c_str() );
            return true;
        } else {
            log_e(ANSI_RED "failed to write '%s' to flash" ANSI_RESET, filename.c_str() );
            return true;
        }
    } else {
        return false;
    }
}


#pragma endregion
//==============================================================================
#pragma region ESP-SR


static void print_commands()
{
#if CONFIG_ARDUHAL_LOG_DEFAULT_LEVEL > 3
    DebugSerial.printf("%-20s|%-20s|%-10s|%-15s|%-10s\n",
        "grapheme","phoneme","action","topic","value"
        );

    for (int i=0; i<n_sr_commands; i++) {
        DebugSerial.printf("%-20s|%-20s|%-10s|%-15s|%-10s\n",
            sr_command_infos[i].grapheme,
            sr_command_infos[i].phoneme,
            sr_command_infos[i].action,
            sr_command_infos[i].itemname,
            sr_command_infos[i].value
        );
    }
#endif
}


/**
 * @brief Load CSV file with speech command info, and parse it into the 
 * structure required by the rest of the application
 * 
 * @return true     all went well
 * @return false    some error occured
 */
bool load_sr_commands()
{
    //const char* CSV_COMMANDS_LOCAL = "/littlefs/oh_sr_commands.csv";
    String csv;

// try to read CSV from HTTP server
    #define CSV_NAME "oh_sr_commands.csv"
    /*
    HTTPClient httpClient;
    httpClient.begin( CSV_COMMANDS_URL );
    const char* headerNames[] = { "Last-Modified" };
    httpClient.collectHeaders(headerNames,1);
    int res = httpClient.GET();
    if (res==200) {
        csv = httpClient.getString();
        CSV_lastModified = httpClient.header("Last-Modified");
    } else {
        log_e(ANSI_BRIGHT_RED "HTTP error %d" ANSI_RESET,res);
    }
    if (csv.length() > 0) {
    */
    if (download_and_store(HTTP_BASE_URL,CSV_NAME,csv,CSV_lastModified)) {
        // got CSV file from HTTP server
        log_i(GREEN_OK "Read SR commands from server" );
    } else {
        log_w( ANSI_RED "can't download commands file %s" ANSI_RESET, 
            HTTP_BASE_URL CSV_NAME );
        // try to read from flash
        File fp = LittleFS.open("/" CSV_NAME,"r");
        if (!fp) {
            log_e(ANSI_RED "can't read commands file from FFS" ANSI_RESET);
            return false;
        }
        csv = fp.readString();
        log_i(GREEN_OK "Read SR commands from FFS" );
    }
    srCommands.parse_csv(csv.c_str());
    print_commands();
    return true;
}


void updateCommands()
{
    if (load_sr_commands())
        srCommands.fill();
}


/**
 * @brief Log message with info about the voice command that was just received
 * 
 * @param id    command id returned by SR library
 */
void reportCommand( int id )
{
//#if CONFIG_ARDUHAL_LOG_DEFAULT_LEVEL > 3
    const command_info_t* pinfo = srCommands[id];
    if (pinfo==NULL) {
        log_e("unknown command %d",id);
    } else {
        log_i(
            "Command id=%d, \n"
            " text='" ANSI_BOLD "%s" ANSI_RESET "',"
            " action='" ANSI_BOLD "%s" ANSI_RESET "',"
            " item='" ANSI_BOLD "%s" ANSI_RESET "',"
            " value='" ANSI_BOLD "%s" ANSI_RESET "'",
            id,
            pinfo->grapheme,
            pinfo->action,
            pinfo->itemname,
            pinfo->value ? pinfo->value : "(none)"
        );
    }
//#endif
}


/**
 * @brief Callback to handle events from the speech recognition machinery
 * 
 * @param event 
 * @param command_id 
 * @param phrase_id 
 */
void onSrEvent(sr_event_t event, int command_id, int phrase_id) {
  switch (event) {
    case SR_EVENT_WAKEWORD: 
      log_i("WakeWord Detected!"); 
      start_recording();
      start_playing(beep_wake);
      break;
    case SR_EVENT_WAKEWORD_CHANNEL:
      log_i("WakeWord Channel %d Verified!", command_id);
      ESP_SR.setMode(SR_MODE_COMMAND);  // Switch to Command detection
      setState(stDetected);
      break;
    case SR_EVENT_TIMEOUT:
      log_i("Timeout Detected!");
      stop_recording();
      stats.n_cmds_err++;
      start_playing(beep_error);
      ESP_SR.setMode(SR_MODE_WAKEWORD);  // Switch back to WakeWord detection
      setState(stCommandError);
      break;
    case SR_EVENT_COMMAND:
      log_i("Command %d Detected!", command_id);
      stop_recording();
      stats.n_cmds_ok++;
      reportCommand(command_id);
      publish_command(command_id);
      //ESP_SR.setMode(SR_MODE_COMMAND);  // Allow for more commands to be given, before timeout
      ESP_SR.setMode(SR_MODE_WAKEWORD); // Switch back to WakeWord detection
      setState(stCommandOk);
      break;
    default: 
      log_i("Unknown Event!");
      break;
  }
}

#if (RX_NCHANNELS == 2)
 #define INPUT_FORMAT "MM"
 #define SR_CHANNELS SR_CHANNELS_STEREO
#else 
 #define INPUT_FORMAT "M"
 #define SR_CHANNELS SR_CHANNELS_MONO
#endif


/**
 * @brief Load speech commands and initialize ESP-SR engine
 * 
 */
void init_SR()
{
    sr_cmd_t* sr_commands = NULL;

    //if (!load_sr_commands()) return;
#ifdef USE_SR_COMMANDS
    sr_commands = build_sr_commands();
    if (NULL==sr_commands) return;
#endif

    ESP_SR.onEvent(onSrEvent);
    
    bool ok = ESP_SR.begin( 
                rx_i2s, 
                sr_commands, 
                sr_commands ? srCommands.count() : 0, 
                SR_CHANNELS, 
                SR_MODE_WAKEWORD,
                INPUT_FORMAT 
            );    
    if (ok)
        log_i(GREEN_OK "ESP_SR.begin()");
    else
        log_e( ANSI_BRIGHT_RED "ESP_SR.begin() failed!" ANSI_RESET);

#ifdef USE_SR_COMMANDS
    free(sr_commands);
#else
    if (srCommands.fill())
        log_i("Sent %u commands to ESP-SR", srCommands.count() );
    else
        log_e("Error in fill_sr_commands_esp()");
#endif
    setState(stIdle);
}


#pragma endregion
//==============================================================================
#pragma region load WAV files from SPIFFS


/**
 * @brief Load a WAV file from flash file system, and turn into a buffer of samples
 * 
 * @param pathname  filename, must start with '/littlefs/'
 * @param wavbuf    reference to wave buffer structure
 * @return true     all ok, found file andcreated buffer
 * @return false    some error occured
 */
bool loadWAV( const char* pathname, wav_buffer_t& wavbuf )
{
    pcm_wav_header_t wavhdr;

    unsigned t1 = millis();

    File f = LittleFS.open(pathname,"r");
    if (!f) return false;
    if (sizeof(wavhdr) != f.readBytes( (char*) &wavhdr, sizeof(wavhdr))) {
        f.close();
        return false;
    }
    if ((wavhdr.fmt_chunk.num_of_channels != 1) || (wavhdr.fmt_chunk.bits_per_sample != 16)) {
        log_e("Bad data format in WAV file '" ANSI_BLUE "%s" ANSI_RESET "'", pathname);
        f.close();
        return false;
    }
    if (!wavbuf.alloc(wavhdr.data_chunk.subchunk_size / sizeof(int16_t))) {
        log_e("error allocating memory for WAV file '" ANSI_BLUE "%s" ANSI_RESET "'", pathname);
        f.close();
        return false;
    }
    wavbuf.samplerate = wavhdr.fmt_chunk.sample_rate;
    size_t nbytes = wavhdr.data_chunk.subchunk_size;
    if (nbytes != f.readBytes( (char*)wavbuf.buf, nbytes )) {
        log_e("error reading WAV file '" ANSI_BLUE "%s" ANSI_RESET "'", pathname);
        f.close();
        return false;
    }
    f.close();

    unsigned t2 = millis();
    log_i(GREEN_OK "read file '" ANSI_BLUE "%s" ANSI_RESET "' in %u ms, %s" ,
        pathname, 
        (unsigned)(t2-t1),
        calc_min_max(wavbuf.buf,wavbuf.end-wavbuf.buf)
        );

    if (config.gain_beep)
        amplify( wavbuf.buf, wavbuf.end-wavbuf.buf, config.gain_beep );
    
    return true;
}


#pragma endregion
//==============================================================================
#pragma region Web frontend

#define HTTPD ANSI_BRIGHT_MAGENTA "HTTP " ANSI_RESET

//#define GET_HTML_FROM_CODE
//#define GET_HTML_FROM_SERVER
#define GET_HTML_FROM_FLASH


#ifdef GET_HTML_FROM_CODE
static const char *htmlContent PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>MyVoiceBox on %HOSTNAME%</title>
  <style>
    body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; line-height: 1.1; }
    .version { color: #808080; font-size:smaller; }
  </style>
</head>
<body>
  <h2>MyVoiceBox on <strong>%HOSTNAME%</strong></h2>
  <p class="version">%VERSION%</p>
</body>
</html>
)rawliteral";
#endif // GET_HTML_FROM_CODE


/* 
    for debugging, we can get the index.html file from an external HTTP server, 
    before serving it ourselves. This way, the HTML code can be edited and tested 
    quickly without flashing anything to the ESP
*/

String get_index_html_from_server()
{
    String s;
    download_and_store( HTTP_BASE_URL, "index.html", s, HTML_lastModified );
    /*
    HTTPClient httpClient;
    String s("");
    httpClient.begin( "http://file-server/ota/index.html" );
    int res = httpClient.GET();
    if (res==200) {
        s = httpClient.getString();
        log_i("read index.html %u bytes",s.length());
    }
    */
    return s;
}


/**
 * @brief Convert template name like MICGAIN to value like '-2'
 * 
 * @param var  template name, uppercase version of label in `config_mapper`
 * @return String   text to be used on HTML page
 */
String my_processor( const String& var )
{
    String varL = var;
    varL.toLowerCase();
    std::string key = varL.c_str();

    auto item = config_mapper.find( key.c_str() );
    String out = (item != config_mapper.end()) ? item->second.toWebString() : var;
    log_d(" Template '%s' -> '%s'", var.c_str(), out.c_str() );
    return out;
}

/*
 * Checkboxes are handled like this:
 * - the HTML code contains a field <input type="checkbox" name="xyz" value="ON"> field, 
 *   and *later a field <input type="hidden" name="xyz" value="OFF"> *of the same name*.
 * - when the checkbox is unchecked, the "checkbox" field does not send a parameter,
 *   so the 1st parameter with name "xyz" in the list is the one from the "hidden"
 *   field, with the value "OFF".
 * - when the checkbox is checked, the "checkbox" field sends a parameter *before*
 *   the "hidden" field, so the 1st parameter with name "xyz" in the list is the 
 *    one from the "checkbox" field, with the value "ON".
 * - request->getParam("xyz") always returns the *first* parameter in the list.
*/

/**
 * @brief Set config elements based on HTTP request http://host/set?name=value
 * 
 * @param request  Web server request info
 */
void processParams( AsyncWebServerRequest *request )
{
    for (auto item : config_mapper) {
        const AsyncWebParameter* wp = request->getParam(item.first.c_str());
        if (wp) {
            log_i(" Param %s = '%s'",
                item.first.c_str(), 
                wp->value().c_str()
            );
            if (item.second.toValue)
                item.second.toValue(wp);
        }
    }
}


/**
 * @brief Initialize Webserver and set filters for URLs
 * 
 */
void init_Webserver()
{
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        log_i(HTTPD "request " ANSI_BLUE "/" ANSI_RESET);
#if defined(GET_HTML_FROM_SERVER) 
        request->send(200,"text/html",get_index_html_from_server(),my_processor);
#elif defined(GET_HTML_FROM_CODE)
        request->send(200,"text/html",htmlContent,my_processor);
#elif defined(GET_HTML_FROM_FLASH)
        request->send(LittleFS,"/index.html","text/html",false,my_processor);
#endif
        printMemoryInfo(DebugSerial);
    });

    //----- paths that trigger actions

    server.on("/clear", HTTP_GET, [](AsyncWebServerRequest *request) {
        log_i(HTTPD "request " ANSI_BLUE "/clear" ANSI_RESET);
        stats.clear();
        request->redirect("/");
    });
    server.on("/set", [](AsyncWebServerRequest *request) {
        log_i(HTTPD "request " ANSI_BLUE "/set" ANSI_RESET);
        processParams(request);
        request->redirect("/");
    });
    server.on("/update", [](AsyncWebServerRequest *request) {
        log_i(HTTPD "request " ANSI_BLUE "/update" ANSI_RESET);
        String s = get_index_html_from_server();
        File f = LittleFS.open("/index.html","w");
        if (f) {
            f.print(s);
            f.close();
            log_i("updated index.html in flash, %u bytes",s.length());
        }
        request->redirect("/");
    });

    //----- return JSON strings with various system info

    server.on("/env",[](AsyncWebServerRequest *request) {
        request->send(200, "application/json", reportEnvironmentString());
    });
    server.on("/mem",[](AsyncWebServerRequest *request) {
        request->send(200, "application/json", reportMemoryInfoString());
    });

    //----- catchall

    server.onNotFound([](AsyncWebServerRequest *request) {
        log_w(HTTPD "Not found: %s", request->url().c_str() );
        size_t nparams = request->params();
        for (int i=0; i<nparams; i++) {
            auto p = request->getParam(i);
            log_i(" Param #%u %s = '%s'",
                i, p->name().c_str(), p->value().c_str());
        }
        request->send(404, "text/plain", "Not found");
    });

    server.begin();
    log_i(HTTPD "HTTP server initialized");
}


#pragma endregion
//==============================================================================
#pragma region Arduino standard functions


void fail( const char* reason )
{
    log_e( ANSI_BRIGHT_RED "%s" ANSI_RESET, reason );
    state = stSystemError;
    for (;;) {delay(1);}
}


void setup() 
{
    delay(100);
    DebugSerial.begin(115200);
    DebugSerial.setDebugOutput(true);

    if (!config.is_valid()) {
        config = default_config;
        log_i("initializing config, checksum=%ux",config.checksum);
    } 

    stats.clear();

    init_LED();
    init_Button();

    DebugSerial.println();
    DebugSerial.println(VERSION);
    printResetReason(DebugSerial);

    setenv("TZ","CET-1CEST,M3.5.0,M10.5.0/3",1);    // Europe/Berlin
    tzset();

    if (!init_FS()) fail("LittleFS mount failed"); 

    //----- I2S
    if (!initVoiceRecorder()) fail("Can't init voice recorder");

    if (!i2s_tx_initialize()) fail("Can't initialize I2S TX");
    xTaskCreate(i2s_tx_task, "i2s_tx_task", 4000, NULL, 5, NULL); 

    if (!i2s_rx_initialize()) fail("Can't initialize I2S RX");

    //----- WAV jingles from SPIFFS
    bool ok = true;
    ok = ok && loadWAV("/beep_hello.wav",beep_hello);
    ok = ok && loadWAV("/beep_wake.wav", beep_wake);
    ok = ok && loadWAV("/beep_error.wav", beep_error);
    if (!ok) fail("can't load WAV files");

    //----- WiFi
    if (!setupWifi()) fail("Can't connect to WiFi");

#ifdef SYSLOG_SERVER
    syslog.deviceHostname(WiFi.getHostname());
    syslog.logMask(LOG_UPTO(LOG_INFO));
    syslog.log( LOG_NOTICE, reportEnvironmentString() );
#else
    log_w("No syslog messages will be sent, SYSLOG_SERVER is not defined")
#endif

    // now that the boring bootloader and WiFi messages are over, let's have more info
    esp_log_level_set("*", ESP_LOG_INFO);

    printEnvironment(DebugSerial);
	printSRinfo(DebugSerial);

    if (!load_sr_commands()) fail("Can't load SR commands file");

    init_Webserver();
    setupOTA( DebugSerial); 

    //----- MQTT
    if (!initMQTT()) fail("Failed to connect to MQTT broker");

    //----- NTP
#ifdef NTP_SERVER
    ntpClient.setPoolServerName(NTP_SERVER);
#endif
    ntpClient.begin(); 
    ntpClient.update(); 

    start_playing( beep_hello );

    init_SR();

    reportTasks();
    printMemoryInfo(DebugSerial);
}


void loop() 
{
    poll_Button();
    loopState();
    if (loopWifi()) {
        ntpClient.update();
        ArduinoOTA.handle();
    }

    if (hasRecord) {
        hasRecord = false;
        if (config.opt_playback)
            start_playback();
        if (config.opt_savewave)
            saveWave(WiFi.getHostname()+8, voice.buf, voice_samples);
    }

    unsigned long now = millis();
#ifdef SYSLOG_SERVER
    // once per day, report memory status
    static unsigned long t_lastlog=0;
    if ((0==t_lastlog) || ((unsigned long)(now - t_lastlog) > 24 * 60 * 60 * 1000uL)) {
        t_lastlog = now;
        syslog.log(LOG_NOTICE,reportMemoryInfoString());
    }
#endif 

    delay(100); 
}

#pragma endregion
