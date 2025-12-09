#ifndef __MYAUTH_H
#define __MYAUTH_H

// fill in your real Wifi and network information and save as "myauth.h"

#define WIFI_SSID       "my-wifi-ssid"
#define WIFI_PASSWORD   "my-secret-wifi-password"

// where to upload recorded WAV files -- if not defined, WAV files are not saved
#define FTP_SERVER  "my-ftp-server.home"
#define FTP_USER    "my-ftp-username"
#define FTP_PASS    "my-ftp-password"

// where to get speech commands .CSV file
#define HTTP_BASE_URL "http://my-http-server.home/ota/"

// MQTT broker used by Rhasspy
#define MQTT_BROKER_RHASSPY  "my-rhasspy-machine.home"

// MQTT broker used by OpenHAB (may be the same as above)
#define MQTT_BROKER_OPENHAB "my-openhab-machine.home"

// local NTP server, so this gadget does not need internet access
#define NTP_SERVER "fritz.box"

#endif // __MYAUTH_H
