#!/usr/bin/bash
if [ -z "$1" ] ; then
    echo "Usage: say <siteId> \"text\""
    echo "Usage: say <siteId> \"text\"" >> /var/log/tts.log
    exit 1
fi

TTS_SERVER=tts-server1:5000
BROKER=stt-server

uuid=$(uuidgen)
topic=hermes/audioServer/$1/playBytes/$uuid
text="${@:2:10}"

curl -X POST \
 -H 'Content-Type: application/json' \
 -s \
 -d "{ \"text\":\"$text\" }" \
 $TTS_SERVER \
 | mosquitto_pub -h $BROKER -t $topic -s

echo "$1 $text" >> /var/log/tts.log
