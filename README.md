# Voice interaction satellite works with OpenHAB and Rhasspy
----
- [Objective](#objective)
- [Solution concept](#solution-concept)
- [Hardware](#hardware)
- [Build instructions](#build-instructions)
  - [Prerequisites](#prerequisites)
  - [Hardware](#hardware-1)
  - [Mechanical construction](#mechanical-construction)
  - [Firmware](#firmware)
- [Technical considerations](#technical-considerations)
  - [Microphone](#microphone)
  - [I2S data formats](#i2s-data-formats)
- [Insights](#insights)
  - [Using Espressif speech recognition in an Arduino project](#using-espressif-speech-recognition-in-an-arduino-project)
- [References](#references)

## Objective

I wanted a few small units for speech-to-text voice recognition and text-to-speech voice output for my home automation environment, which includes OpenHAB and Rhasspy.

Before, I have used Rhasspy satellites built with a Raspberry Pi and a small speaker, but now I wanted something simpler and more compact, and with the voice recognition happening in the unit itself, without streaming audio to a server. 

Espressif offers a ready-made speech recognition component for their ESP32-S3 processors called `esp-sr`, which can also be used in an Arduino project ... sort of, see below for the nasty details. 
* the *advantage* of using `esp-sr` over a Raspberry Pi based satellite is that the speech recognition happens on the local processor, so there is no need to stream raw audio over WiFi to a central Rhasspy server.
* a *disadvantage* of using `esp-sr` versus Rhasspy is that it can only detect fixed text, like "*turn on the lamp*", but not text with variable numbers, like "*dim the lamp to 50 percent*" .
* another *disadvantage* of `esp-sr` versus Rhasspy is that it only does speech-to-text (voice recognition) and not text-to-speech (voice synthesis) ... unless you understand Mandarin.

The plan was to package the ESP32-S3 module with a small speaker, and write software that integrates with my existing home automation environment.

## Solution concept

* a Python script occasionally running on a server queries OpenHAB for all voice-controlled items, builds phrases such as "*turn on kitchen lamp*" and "*turn off kitchen lamp*" , converts them to phonemes to be fed to the `esp-sr` library, and stores all information in a file on a server. 
* a small ESP32-S3 module running my software and the Espressif `esp-sr` speech recognition library retrieves the phrases file from the server, initializes the library, and then listens to voice commands.
* when the ESP32-S3 module detects a voice command, it publishes a message to MQTT, in essentially the same format that a Rhasspy satellite would produce. Therefore, I only need one OpenHAB rule to deal with commands from the new ESP32-S3 module, or an existing Rhasspy satellite.
* for voice synthesis, I use the Rhasspy text-to-speech engine, the ESP32-S3 module just receives a WAV file from Rhasspy (via MQTT) and plays that on its speaker.

## Hardware

I developed this project using a cheap ESP32-S3 dev module from Aliexpress, a clone of the Espressif ESP32-S3-DevKitC1-N16R6. This has 16MB flash, 8 MB PSRAM, and a Neopixel-esque RGB LED on board.

A MAX98357A I2S digital amplifier module is connected to I2S interface #0 and drives the speaker

Two INMP441 I2S digital microphones module are connected to I2S interface #1.

An analog RGB LED serves as a status indicator: waiting for wakeword, recognized  wakeword and waiting for command, recognized command, or timeout.

## Build instructions

### Prerequisites

You will need
* a working installation of **OpenHAB**, to receive and interpret the commands recognized by MyVoiceBox. I currently use OpenHAB 4.1.1, running under Debian on a virtual x86 machine.
* a working installation of **Rhasspy**, configured for a TTS engine of your choice. I use Rhasspy 2.5.11, running under Debian on a virtual x86 machine, withe *Larynx* test-to-speech engine and the *blizzard_lessac* voice.
* an **HTTP** server that can serve files to MyVoiceBox. I use Apache 2.4.65 running under Debian on a virtual x86 machine.

### Hardware

<img src="pictures/D-top.jpg" alt="top view" width="450"> <img src="pictures/D-bottom.jpg" alt="bottom view" width="450">

Follow the [schematic](hardware/MyVoiceBox.pdf) to build the hardware. For production units, I used a "bare" ESP32-S3-WROOM-1 N16R8 on a prototype board that has a footprint for an ESP32 module ... the pinout is similar enough so I could place the ESP32-**S3** module on the footprint intended for a "plain" ESP32. Ignore the labels on the board, the pinout of an ESP32-S3 is different from an ESP, except the GND, 3v3, TxD and RxD signals, which are on the same pins for both modules.

For a speaker, I use half of a pair of "*Amazon Basics Stereo 2.0 Speakers for PC or Laptop*". They are cheap (typically under €20 per pair), and the sound quality is quite good, for this type of application.

### Mechanical construction

I used a 100x60x25mm plastic box for the project. Here is how I placed the microphones:
- solder the pin headers to the INMP441 microphone module such that the tip of the pins are flush with the surface of the module PCB on the side away from the components.
- place the microphones on a piece of perf board, with the microphone holes ca. 5cm apart
- drill two 2mm holes in the side of the plastic box, with a distance in between that is the distance between the microphone holes
- place a piece of thick (1mm) double-sided sticky tape over each microphone module, and cut out a small hole over each microphone hole
- align the perf-board and microphones assembly over the holes in the platic box, and glue it to the plactic box

### Firmware

1. Clone the Github repository.
2. Copy `main/myauth_sample.h`to `main/myauth.h` and edit that to define your WiFi SSID and password, as well as the names of your servers in your home environment.
3. In folder `tools/`, edit `oh_sr_commands.py` to set the name of your OpenHAB server, then run `python oh_sr_commands.py`. This creates file `oh_sr_commands.csv`, copy that to your HTTP server. This is where the MyVoiceBox firmware will get its information about voice-related OpenHAB items. 
4. Build firmware and upload to your ESP32-S3 module, over USB.
5. Future updates can also be done over-the-air. Edit `ota-update.bat` to set the IP address of your device, then run it.

## Technical considerations

### Microphone

I use an INMP441 digital microphone, with the following characteristics:
- sensitivity (per datasheet): -26 dBFS for 94 dB SPL, i.e. 120 dB SPL at full scale
- 24 bit data size, sent as 32 bit words
- pin LR set to GND for left, VCC for right

I don't expect input signals beyond 100 dB SPL, or -20dB FS, so we can amplify 
the 24-bit word by 18 dB (3x left shift), and then use the top 16 bits.

If we consider the upper 16 bits only, then the theoretical noise floor is -96 dB FS 
or **24** dB SPL, which is well below the actual ambient noise floor in a quiet room.
The INMP441 datasheet specifies a noise floor of -87 db FS (A-weighed), 
or an EIN of **33** dB SPL(A).

CONCLUSION: if we just consider the top 16 bits, and then amplify by 18 dB, we 
only get bits 15:3, and bits 2:0 are always zero. That is in effect a 13-bit ADC 
with full scale at 102 dB SPL and a noise floor at 102-6*13 = 24 dB SPL, good enough.

### I2S data formats

- "Philips format" means MSB starts 1 BCLK period after LRCK edge 
- "MSB format" means MSB starts on edge or LRCK
- Arduino `I2SClass::begin()` always selects Philips format
- INMP441 produces Philips format
- MAX98357A expects Philips format
- sending MSB format when the receiver expects Philips format is equivalent to 
  left-shifting data one bit

## Insights

### Using Espressif speech recognition in an Arduino project

I wanted to write an Arduino project, using the Espressif `esp-sr` component for 
speech recognition, wrapped in the `ESP_SR` Arduino library.

However ... in an *Arduino* project, the wakeword is fixed as "Hi,ESP", and cannot be changed, because the ESP libraries are precompiled for this particular configuration. I tried the real Arduino IDE as well as Arduino CLI, VSCode with the ["Arduino Community Edition" extension](https://github.com/vscode-arduino/vscode-arduino), and VSCode with Platformio, all with the same result.

Therefore, this is an ESP-IDF project with "Arduino as a component". I use VSCode with the ESP-IDF extension. 

In the ESP-IDF project, one of several wakewords can be selected through the project configuration tool. The original `ESP_SR` Arduino library (at least for Core 3.3.0) also has the wakeword "Hi, ESP" hardcoded, so I had to clone that library and make my changes. The cloned library is called `ESP_SRx` and can be found in the `components/ESP_SRx` folder.

## References

- INMP441 digital microphone [datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/INMP441.pdf)
- MAX98357A I2S PCM class D amplifier [datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/max98357a-max98357b.pdf)