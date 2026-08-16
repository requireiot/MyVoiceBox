# How to setup and run a standalone TTS server 

Setting up a dedicated (virtual) TTS server to run [piper-tts](https://github.com/OHF-Voice/piper1-gpl) takes ony a few steps:

Create a virtual machine running Debian. I have tested this with Debian 13, and a VM on VirtualBox with 2GB RAM, 2 codes, 8 GB disk.

## Installation

Install prerequisites for piper-tts, and allow global `pip` installs
```
apt-get install python3-pip curl uuid-runtime
cd /usr/lib/python3.13
mv EXTERNALLY-MANAGED not-EXTERNALLY-MANAGED
```
Install `piper-tts` and download voices
```
pip install piper-tts
pip install flask
python3 -m piper.download_voices en_US-lessac-medium
```
Some of the hardware variants don't allow the samplerate for speaker output to be different from the one for microphone input. Because the speech recognition package expects 16000 Hz input samplerate, we need to force the TTS package to use the same samplerate. Edit `piper/en_US-lessac-low.onnx.json` and make sure it contains these lines
```
  "audio": {
    "sample_rate": 16000,
    "quality": "low"
  },
```
Test the installation by sending a string to the server. It will be converted to speech and stored as `test.wav`
```
curl -X POST -H 'Content-Type: application/json' -d '{ "text":"Hello world"}' -o test.wav localhost:5000
```
## Autostart the TTS service

To configure `piper-tts`to run as a service, and start automatically when the server starts,  type `systemctl edit --force --full piper.service` and then enter this text
```
[Unit]
Description=Piper TTS server
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
ExecStart=python3 -m piper.http_server -m en_US-lessac-medium
WorkingDirectory=/home/admin/piper    <<-- or wherever you installed piper-tts
Restart=always
RestartSec=5

[Install]
WantedBy=default.target
```
Now, enable the service and run it
```
sudo systemctl enable piper.service
sudo systemctl start piper.service
``` 
The next time your server starts, it will start the piper service automatically, and you can have it convert text to speech.

## Use with OpenHAB

On my OpenHAB server, I have a script `tts-speak.sh` which is called like `./tts-speak.sh <site-id> <text>`
1. sends a text to the piper TTS server (mine is named `tts-server1`)
2. retrieves the WAV file returned by the TTS server
3. publishes the WAV file contents as an MQTT message on the server that my ESP32-S3 device listens to (mine is named `stt-server1`)
So in order to have "hello world" spoken on my ESP32-S3 box named `esp32s3-C`, I enter
```
./tts-speak.sh esp32s3-C Hello world
```
The script looks like this
```
#!/usr/bin/bash
if [ -z "$1" ] ; then
    echo "Usage: say <siteId> \"text\""
    exit 1
fi

# the machine where the piper-tts service is running
TTS_SERVER=tts-server1:5000
# the machine that the VoiceBox units subscribe to
STT_SERVER=stt-server1

uuid=$(uuidgen)
topic=hermes/audioServer/$1/playBytes/$uuid
text="${@:2:10}"

curl -X POST \
 -H 'Content-Type: application/json' \
 -s \
 -d "{ \"text\":\"$text\" }" \
 $TTS_SERVER \
 | mosquitto_pub -h $STT_SERVER -t $topic -s
```
