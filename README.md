# Voice satellite works with OpenHAB and Rhasspy
----
- [Objective](#objective)
- [Key features](#key-features)
- [Bird's eye view of the solution](#birds-eye-view-of-the-solution)
- [Hardware](#hardware)
- [Build instructions](#build-instructions)
  - [Prerequisites](#prerequisites)
  - [Hardware](#hardware-1)
  - [Mechanical construction](#mechanical-construction)
  - [Firmware](#firmware)
  - [OpenHAB integration](#openhab-integration)
    - [Conventions](#conventions)
    - [STT](#stt)
    - [TTS](#tts)
    - [Rules](#rules)
  - [Rhasspy integration](#rhasspy-integration)
- [Technical considerations](#technical-considerations)
  - [Microphone](#microphone)
  - [Attenuating the speaker signal](#attenuating-the-speaker-signal)
  - [Design alternatives](#design-alternatives)
    - [OpenHAB STT instead of Rhasspy](#openhab-stt-instead-of-rhasspy)
    - [Other speech recognition models](#other-speech-recognition-models)
- [Insights](#insights)
  - [Using Espressif speech recognition in an Arduino project](#using-espressif-speech-recognition-in-an-arduino-project)
  - [Acoustic "debugging"](#acoustic-debugging)
- [Alternatives](#alternatives)
- [References](#references)
- [License](#license)

## Objective

I wanted a few small units for speech-to-text voice recognition and text-to-speech voice output for my home automation environment, which includes OpenHAB and Rhasspy.

Before, I have used Rhasspy satellites built with a Raspberry Pi and a small speaker, but now I wanted something simpler and more compact, and with the voice recognition happening in the unit itself, without streaming audio to a server. 

Espressif offers a ready-made speech recognition component for their ESP32-S3 processors called `esp-sr`, which can also be used in an Arduino project ... sort of, see below for the nasty details. 
* the *advantage* of using `esp-sr` over a Raspberry Pi based satellite is that the speech recognition happens on the local processor, so there is no need to stream raw audio over WiFi to a central Rhasspy server.
* a *disadvantage* of using `esp-sr` versus Rhasspy is that it can only detect fixed text, like "*turn on the lamp*", but not text with variable numbers, like "*dim the lamp to 50 percent*" .
* another *disadvantage* of `esp-sr` versus Rhasspy is that it only does speech-to-text (voice recognition) and not text-to-speech (voice synthesis) ... unless you understand Mandarin.

The plan was to package the ESP32-S3 module with a small speaker, and write software that integrates with my existing home automation environment.

## Key features

- a small device based on an ESP32-S3, total cost of materials ca. €20
- self-contained speech recognition in the unit, no audio streaming to a server
- automatic configuration for voice-controlled items defined in OpenHAB 

## Bird's eye view of the solution

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

<img src="pictures/D-top.jpg" alt="top view" width="400"> <img src="pictures/D-bottom.jpg" alt="bottom view" width="400">

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

### OpenHAB integration

#### Conventions

* for all OpenHAB *items* to be voice controlled, the item label is the name used in the voice command
* all OpenHAB *items* that can be **switched** on and off are assigned to group `gVA`. This creates voice commands "*turn on `name`*" and "*turn off `name`*".
* all OpenHAB *items* that can be **dimmed** are assigned to group `gVD`. This creates voice commands "*dim `name` to low|medium|high|off*".
* all OpenHAB *items* that represent lighting **scenes** are assigned to group `gVS`. This create voice commands "*let's `name`".
* all OpenHAB items that can be asked about are assigned to group `gVQ`. This creates commands "*what is the `name`*"

So my OpenHAB items definition file `/etc/openhab/items/voice.items` file might look like this
```
 Switch KitchenLight    "kitchen light"        (gVA) { ... some binding ... }
 Number BedsideLampDim  "bedside lamp"         (gVD) { ... some binding ... }
 Switch sceneTV         "watch TV"             (gVS) // dummy switch watched by a rule
 Number localTemp       "temperature [%.0f°C]" (gVQ) { ... some binding ... }
```

#### STT

To capture the intent information sent by the ESP32-S3 module via MQTT, there is one *thing* and one *item* per category

In `rhasspy.things`
```
Thing mqtt:topic:rh-mq:VoiceCommands (mqtt:broker:rh-mq) {
  Channels:
    Type string: switchIntent  [ stateTopic="hermes/intent/switch" ]
    Type string: queryIntent   [ stateTopic="hermes/intent/query" ]
    Type string: sceneIntent   [ stateTopic="hermes/intent/scene" ]
    Type string: dimcatIntent  [ stateTopic="hermes/intent/dim_cat" ]
}
```
In `rhasspy.items`
```
String  Rhasspy_Intent_Switch "Intent: switch" { channel="mqtt:topic:rh-mq:VoiceCommands:switchIntent" }
String  Rhasspy_Intent_DimCat "Intent: dimmer" { channel="mqtt:topic:rh-mq:VoiceCommands:dimcatIntent" }
String  Rhasspy_Intent_Query  "Intent: query"  { channel="mqtt:topic:rh-mq:VoiceCommands:queryIntent" }
String  Rhasspy_Intent_Scene  "Intent: scene"  { channel="mqtt:topic:rh-mq:VoiceCommands:sceneIntent" }
```

#### TTS
For each text-to-speech destination, whether it is a Raspberry Pi based Rhasspy satellite, or an ESP32-S3 module from this project, there is a channel in a *thing* and an *item*, refering to the Rhasspy `siteId` or ESP hostname:

In `rhasspy.things`
```
Thing mqtt:topic:rh-mq:say (mqtt:broker:rh-mq) {
  Channels:
    Type string: raspi14  [ commandTopic="hermes/tts/say", 
                            formatBeforePublish="{\"text\":\"%s\",\"siteId\":\"raspi14\"}" ]
    Type string: esp32s3_56BCFC  [ commandTopic="hermes/tts/say", 
                            formatBeforePublish="{\"text\":\"%s\",\"siteId\":\"esp32s3-56BCFC\"}" ]
}

```
In `rhasspy.items`
```
String say_raspi14          (gSay) { channel="mqtt:topic:rh-mq:say:raspi14" }
String say_esp32s3_56BCFC   (gSay) { channel="mqtt:topic:rh-mq:say:esp32s3_56BCFC" }
```

#### Rules

One rule handles incoming voice commands to **switch** something
```
/* respond to a voice command like "turn XYZ on" */
rule "Rhasspy switchIntent message"
when 
    Item Rhasspy_Intent_Switch received update
then 
    val String json = newState.toString
    val String rawInput = transform("JSONPATH","$.rawInput", json)
    val String itemName = transform("JSONPATH","$.slots[?(@.entity=='oh_items')].value.value", json)
    val String itemState = transform("JSONPATH","$.slots[?(@.entity=='state')].value.value", json).toUpperCase
    val String siteId = transform("JSONPATH","$.siteId", json)
    logInfo("voice","Site {} heard '{}'", siteId, rawInput )

    // set the item as requested
    val theItem = gVA.members.findFirst[ t | t.name==itemName] as SwitchItem
    if (theItem !== null) {
        theItem.sendCommand(itemState)
    } else {
        logInfo("voice","ERROR: no item named '{}'", itemName)    
    }
end 
```

A similar rule handles incoming voice commands to **dim** something
```
/* handle a voice command such as "set dimmer to low" */
rule "Rhasspy dimcatIntent message"
when 
    Item Rhasspy_Intent_DimCat received update
then 
    val String json = newState.toString
    val String rawInput = transform("JSONPATH","$.rawInput", json)
    val String itemName = transform("JSONPATH","$.slots[?(@.entity=='oh_items')].value.value", json)
    val String siteId = transform("JSONPATH","$.siteId", json)
    val String itemState = transform("JSONPATH","$.slots[?(@.entity=='state')].value.value", json)
    logInfo("voice","Site {} heard '{}'", siteId, rawInput )

    val String percentS = transform("MAP","cat2num.map",itemState)
    val Number percent = Float::parseFloat(percentS).intValue
    logInfo("voice","COMMAND '{}', set dimmer {} to {} percent", rawInput, itemName, percent )

    val theItem = gVD.members.findFirst[ t | t.name==itemName] as DimmerItem
    if (theItem !== null) {
        theItem.sendCommand(percent)
    } else {
        logInfo("voice","ERROR: no item named '{}'", itemName)    
    }
end 
```
The transformation from categories low|medium|high|off to percentages is done via a map transform, defined in `/etc/openhab/transform/cat2num.map`
```
# convert dimmer category to numerical dimmer value
low=10
medium=50
high=100
off=0
=0
```

A voice command to select a lighting **scene** is handled by a simple rule that turns on the virtual switch for that scene
```
/* handle a voice scene selection such as "lets watch TV" */
rule "Rhasspy VoiceScene message"
when 
    Item Rhasspy_Intent_Scene received update
then 
    val String json = newState.toString
    val String rawInput = transform("JSONPATH","$.rawInput", json)
    val String itemName = transform("JSONPATH","$.slots[?(@.entity=='oh_items')].value.value", json)
    val String siteId = transform("JSONPATH","$.siteId", json)
    logInfo("voice","Site {} heard '{}', select scene '{}'", siteId, rawInput, itemName )

    val theItem = gVS.members.findFirst[ t | t.name==itemName] as SwitchItem
    if (theItem !== null) {
        theItem.sendCommand(ON)
    }
end 
```

Separate rules then respond when a "scene switch" is turned on, but this is outside the scope of this project.

Finally, a voice command that **asks** about an item is handled by this rule:
```
/* handle a voice query such as "what is the temperature?" */
rule "Rhasspy VoiceQuestion message"
when 
    Item Rhasspy_Intent_Query received update
then 
    val String json = newState.toString
    val String rawInput = transform("JSONPATH","$.rawInput", json)
    val String topic = transform("JSONPATH","$.slots[?(@.entity=='oh_items')].rawValue", json).toLowerCase
    val String siteId = transform("JSONPATH","$.siteId", json)
    val String roomName = transform("MAP","stt_to_room.map",siteId)
    logInfo("voice","Site {} heard '{}', ask about '{}'", siteId, rawInput, topic )

    var String answer = " I don't know"

    if (topic=="temperature") {
        answer = "the temperature is " 
        + String::format("%.0f", (localCurrentTemperature.state as DecimalType).floatValue )
        + " degrees"
    } // else { add more topics here as needed

    // where should the answer be heard?
    val sinkName = siteId.replace("-","_")
    val sinkItem = gSay.members.findFirst[ t | t.name=="say_"+sinkName] as StringItem
    if (sinkItem !== null) {
        sinkItem.sendCommand(answer)
    }
    logInfo("voice","QUESTION '{}' to '{}', ANSWER '{}' to '{}'", topic, siteId, answer, sinkName)
end 
```

### Rhasspy integration

Since we use Rhasspy to generate the audio signal for spoken voice messages, the ESP32-S3 based "satellites" must be known to Rhasspy. Browse to your Rhasspy web configuration page at `http://some-server:12101/settings`, drop down the `Text to Speech` area, select your favorite Voice, and in the `Satellite siteIds:` field, add the hostname of each MyVoiceBox module. Mine looks like this
```
Satellite siteIds: raspi13,raspi14,esp32s3-56BCF4,esp32s3-56BCFC
```
in order for "real" Rhasspy satellites to generate the same MQTT messages about recognized voice comands as the MyVoiceBox units, my `sentences.ini`, which can be edited at `http://some-server:12101/sentences` looks like this:
```
[switch]
(turn | switch) (on | off){state!upper} [the] ($oh_items,gVA){lightName} [please] 

[query]
what is the ($oh_items,gVQ){topic!lower}

[scene]
lets ($oh_items,gVS){topic}

[dim_cat]
set ($oh_items,gVD){itemName} to (low | medium | high){state}
```

This relies on a Python script at  which queries OpenHAB and assembles a list of item names assigned to one of the voice-related groups. The script is stored in `~/.config/rhasspy/profiles/en/slot_programs/oh_item` on the machien running Rhasspy and contains
``` Python
#!/usr/bin/python3
from requests import get
import sys,os
if (len(sys.argv)<2):
    print(f'No group name specified', file=sys.stderr)
    exit(1)
groupname = sys.argv[1]
print(f'Create Rhasspy slots from OpenHAB items in group {groupname} ...', file=sys.stderr)
# set OpenHAB REST API url to get a list of all items
url = "http://ha-server:8080/rest/items?recursive=false"
headers = {
    "content-type": "application/json",
}
response = get(url, headers=headers)
items = response.json()
for item in items:
    name = item['name']
    groups = item['groupNames']
    if groupname in groups:
        friendly_name = item['label']
        print(f"({friendly_name}):{name}")
```

## Technical considerations

### Microphone

I use an INMP441 digital microphone, with the following characteristics:
- sensitivity (per datasheet): -26 dBFS for 94 dB SPL, i.e. 120 dB SPL at full scale
- 24 bit data size, sent as 32 bit words

I don't expect input signals beyond 100 dB SPL, or -20dB FS, so we can amplify 
the 24-bit word by 18 dB (3x left shift), and then use the top 16 bits. If we consider the upper 16 bits only, then the theoretical noise floor is -96 dB FS or **24** dB SPL, which is well below the actual ambient noise floor in a quiet room.
The INMP441 datasheet specifies a noise floor of -87 db FS (A-weighed), or an EIN of **33** dB SPL(A).

If we just consider the top 16 bits, and then amplify by 18 dB, we only get bits 15:3, and bits 2:0 are always zero. That is in effect a 13-bit ADC with full scale at 102 dB SPL and a noise floor at 102-6*13 = 24 dB SPL, good enough.

### Attenuating the speaker signal

The ESP-SR library affords feeding the speaker output back into the input, as a third channel, to minimize feedback from the speaker back into the microphones. I implemented the poor man's version of that: as long as a signal is being played through the speaker, the microphone signals are attenuated. The level of attenuation is configurable in the web interface.

### Design alternatives

#### OpenHAB STT instead of Rhasspy

Instead of using Rhasspy for text-to-speech functionality, one could use the STT functions built into OpenHAB, have it generate a WAV file and send that to the MyVoiceBox device. I already had Rhasspy up and running, so I didn't pursue that route

#### Other speech recognition models

The firmware currently uses Multinet5, an older speech recognition model provided by Espressif. They also have the newer Multinet7 model, which is supposedly more accurate. I tried it, and actually found it to be *less* accurate, maybe I did something wrong there.

## Insights

### Using Espressif speech recognition in an Arduino project

I wanted to write an Arduino project, using the Espressif `esp-sr` component for 
speech recognition, wrapped in the `ESP_SR` Arduino library.

However ... in an *Arduino* project, the wakeword is fixed as "Hi,ESP", and cannot be changed, because the ESP libraries are precompiled for this particular configuration. I tried the real Arduino IDE as well as Arduino CLI, VSCode with the ["Arduino Community Edition" extension](https://github.com/vscode-arduino/vscode-arduino), and VSCode with Platformio, all with the same result.

Therefore, this is an ESP-IDF project with "Arduino as a component". I use VSCode with the ESP-IDF extension. 

In the ESP-IDF project, one of several wakewords can be selected through the project configuration tool. The original `ESP_SR` Arduino library (at least for Core 3.3.0) also has the wakeword "Hi, ESP" hardcoded, so I had to clone that library and make my changes. The cloned library is called `ESP_SRx` and can be found in the `components/ESP_SRx` folder.

### Acoustic "debugging"

I found it really helpful to be able to listen to the audio signal picked up by the microphone and sent to the speech recognition engine. There are two features, which can be enabled via the web interface:
- record the audio signal while a command is being spoken, and save it to a WAV file on an external server
- record the audio signal while a command is being spoken, and replay it through teh speaker immediately afterwards

These help to address issues like: is there too much echo from the room? is the signal contamiated with electrical noise?

## Alternatives

Possible alternatives to this project, with similar objectives and features are 

* a **Raspberry Pi based Rhasspy satellite**, as described in my [blog post](https://requireiot.com/rhasspy-satellite-with-raspberry-pi/). It works, but requires more power, and audio is streamed over WiFi for speech recognition at a central in-home server.
* the **[ESP32 Rhasspy Satellite](https://github.com/Romkabouter/ESP32-Rhasspy-Satellite)** project, . It uses a "regular" ESP32, I built one, see my [blog post](https://requireiot.com/basic-satellite/). I used it for output only, because for voice recognition, it requires constant streaming of the audio signal to a central server. Even the wakeword detection is done on the server.
* the **[Willow](https://heywillow.io/)** project, which also runs on an ESP32-S3 and can use the speech recognition library provided by Espressif. It is a sophisticated open-source project, but its focus appears to be on working with a central in-home server, which they call the "inference server". Also, it doesn't have voice output, as far as I can tell. The required hardware is from a small list of devices sold by Espressif, which they call "cheap" at $50 ... still more than the ~ €20 hardware cost of MyVoiceBox.

## References

- INMP441 digital microphone [datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/INMP441.pdf)
- MAX98357A I2S PCM class D amplifier [datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/max98357a-max98357b.pdf)

## License

The code and schematics I created for this project are under an MIT license, see [LICENSE.txt](LICENSE.txt). Folder `components/` contains libraries cloned from other repositries, which may be under a different license, see each folder for more details.