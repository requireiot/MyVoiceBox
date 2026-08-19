#!/usr/bin/bash
if [ -z "$1" ] ; then
    echo "Usage: say <siteId> \"text\""
    exit 1
fi

# echo "Arg1:$1" >> /var/log/tts.log
# echo "Arg2:$2" >> /var/log/tts.log

TTS_SERVER=tts-server1:5000
BROKER=stt-server
WAVFILE=/etc/openhab/sounds/dingdong16.wav

uuid1=$(uuidgen)
topic1=hermes/audioServer/$1/playBytes/$uuid1
uuid2=$(uuidgen)
topic2=hermes/audioServer/$1/playBytes/$uuid2
waittopic=hermes/audioServer/$1/playFinished
text="${@:2:10}"

# play dingdong
mosquitto_pub -h $BROKER -t $topic1 -s < $WAVFILE
# wait for play complete
mosquitto_sub -h $BROKER -t $waittopic -C 1 -W 5
# play TTS text
curl -X POST \
 -H 'Content-Type: application/json' \
 -s \
 -d "{ \"text\":\"$text\" }" \
 $TTS_SERVER \
 | mosquitto_pub -h $BROKER -t $topic2 -s
