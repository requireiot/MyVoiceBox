# Requirements for voice interaction device

## Hardware

- [x] `R100` The device includes 1 or 2 microphones
- [x] `R101` The device includes 1 speaker
- [x] `R103` The device includes a LED to indicate internal status
- [x] `R104` The application firmware must work on an ESP32-S3 with 16MB flash and 8 MB PSRAM
  
## Speech recognition

### Definition of voice commands

- [x] `R200` The device recognizes spoken commands in English, from a predefined list of commands
- [x] `R201` The device reads the predefined list at startup
  - [x] `R201.1` The predefined list can be read from an HTTP server, at startup
  - [x] `R201.2` The predefined list can be read from a file in flash memory
- [x] `R202` The predefined list of voice commands can be generated automatically, on demand, by a script running on a server, from the items defined in the home automation system that are tagged as relevant for voice interaction

### Speech recognition behavior

- [x] `R203` When a voice command is detected, the device notifies the home automation system.
  - [x] `R203.1` The notification is a message published to MQTT
  - [x] `R203.2` The notification message is in a format compatible with what Rhasspy would generate
- [x] `R204` The wakeword can be selected at build time, from the words offered by the ESP-SR component, such as "Alexa" or "HI,ESP"
- [x] `R205` When the wakeword is detected, the device indicates that to the user
  - [x] `R205.1` The indication is made as an audible signal (beep)
  - [x] `R205.2` The indication is made as a visual signal, i.e. a change in color or blink pattern of the LED. *Implementation*: Blinking yellow light
- [x] `R206` When a voice command is recognized, the device indicates that to the user
  - [x] `R206.1` The indication is made as a visual signal, i.e. a change in color or blink pattern of the LED. *Implementation*: Steady green light
- [x] `R207` when no voice is detected within a set time period after detecting the wakeword (timeout), then the device indicates this error condition to the user
  - [x] `R207.1` The indication is made as an audible signal (beep)
  - [x] `R207.2` The indication is made as a visual signal, i.e. a change in color or blink pattern of the LED. *Implementation*: Steady red light
  - [x] `R207.3` The timeout period is configurable in source code
- [x] `R208` when the device is idle, i.e. is waiting for the wakeword, it indicates that to the user
  - [x] `R208.1` The indication is in the form of a visual signal, i.e. a change in color or blink pattern of the LED. *Implementation*: Breathing blue light

## Audio output

- [x] `R300` The device can receive audio files over the network, and play those on its speaker
  - [x] `R300.1` The audio files are in the format of a WAV file
  - [x] `R300.2` *Constraint*: The audio files must be mono, 16-bit
  - [x] `R300.3` The audio files can specify a sample rate, up to 44100 S/s. *Information*: Rhasspy TTS produces files at 22050 S/s.
- [x] `R301` The device receives audio files via MQTT
  - [x] `R301.1` The device expects MQTT topics in the format that Rhasspy would send to an audio output satellite
  - [x] `R301.2` The device responds to a Rhasspy "site ID" that is its own automatically generated WiFi hostname, such as "esp32s3-ABCDEF"

## Audio preprocessing

- [x] `R400` the microphone signal can be amplified or attenuated to optimize functionality
  - [x] `R400.1` the gain can be specified in steps of 6 dB, i.e. one bit shift left or right
- [x] `R401` the microphone signal is attenuated while a beep is played, to reduce the loudness and hopefully reduce interference with the speech recognition process
  - [x] `R401.1` the attenuation gain is specified in source code
  - [x] `R401.2` the attenuation gain is specified via the Web frontend
- [x] `R402`the microphone signal is amplified when no beep is being played, to increase the loudness of the speech signal presented to the speech recognition engine
  - [x] `R402.1` the amplification gain is specified in source code
  - [x] `R402.2` the amplification gain is specified via the Web frontend

## Connectivity

- [x] `R500` the device connects to the home network via WiFi
  - [x] `R500.1` the WiFi SSID and password are specified in source code
- [x] `R501` the device communicates with the MQTT broker used by Rhasspy
  - [x] `R501.1` the MQTT broker URL is specified in source code, i.e. no need for configuration at runtime
- [x] `R502` the device firmware can updated over-the-air (OTA)

## Development support

- [x] `R600` the device outputs log messages via hardware UART
- [x] `R601` the device outputs log messages to a remote *syslog* server
  - [x] `R601.1` the remote syslog server URL is specified in source code, i.e. no need for configuration at runtime
- [x] `R602` audible signals as mentioned in `R205.1` and `R206.1` are WAV formatted files
  - [x] `R602.1` the WAV files are selected at build time
  - [x] `R602.2` the WAV files are available to the firmware without access to a remote server. *Implementation*: WAV files are stored in flash memory.
- [x] `R603` the device can record the microphone signal while a command is being spoken, and store the recording as a WAV file on an external server. *Rationale*: Evaluate audio signal quality for loudness, noise, distortion etc.
  - [x] `R603.1` the recording function can be enabled or disabled in source code
  - [x] `R603.2` the recording function can be enabled or disabled via the Web frontend
  - [x] `R603.3` the recording is stored on an FTP server
  - [x] `R603.4` the FTP server name and credentials are defined in source code, i.e. no need for runtime configuration