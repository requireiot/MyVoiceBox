@echo off
set ESP_OTA=/.platformio/packages/framework-arduinoespressif32/tools/espota.exe

rem for ESP32S3-A = esp32s3-186B24
rem set IP=192.168.164.144

rem for ESP32S3-C = esp32s3-56BCF4
rem set IP=192.168.164.142

rem for ESP32S3-D = esp32s3-56BCFC
set IP=192.168.164.143

IF NOT [%1] == [] set IP=192.168.164.%1

echo flashing to %IP%
%ESP_OTA% -i %IP% -r -f .build/MyVoiceBox.bin