#
# oh_sr_commands.py --- query OpenHAB about items relevant for voice control,
# and generate 
# - a CSV file that can be downloaded by the ESP32-S3 module
# - a TXT file needed at build time when using Multinet7
#

# This Revision: $Id: oh_sr_commands.py 1945 2025-12-09 17:34:46Z  $

#
#   Copyright (C) 2025 Bernd Waldmann
#
#   This Source Code Form is subject to the terms of the Mozilla Public License, 
#   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
#   obtain one at http://mozilla.org/MPL/2.0/
#
#   SPDX-License-Identifier: MPL-2.0
#

# pip3 install g2p_en num2words
from g2p_en import G2p
import argparse
from requests import get
from num2words import num2words
import re
import sys
import os
import string

# OpenHAB REST API url -- adjust this for your environment
OPENHAB_URL = "http://ha-server:8080/rest/items?recursive=false" 
MAX_COMMANDS          = 200
ESP_MN_MAX_PHRASE_LEN = 63


alphabet = {
    "AE1": "a",
    "N": "N",
    " ": " ",
    "OW1": "b",
    "V": "V",
    "AH0": "c",
    "L": "L",
    "F": "F",
    "EY1": "d",
    "S": "S",
    "B": "B",
    "R": "R",
    "AO1": "e",
    "D": "D",
    "AH1": "c",
    "EH1": "f",
    "OW0": "b",
    "IH0": "g",
    "G": "G",
    "HH": "h",
    "K": "K",
    "IH1": "g",
    "W": "W",
    "AY1": "i",
    "T": "T",
    "M": "M",
    "Z": "Z",
    "DH": "j",
    "ER0": "k",
    "P": "P",
    "NG": "l",
    "IY1": "m",
    "AA1": "n",
    "Y": "Y",
    "UW1": "o",
    "IY0": "m",
    "EH2": "f",
    "CH": "p",
    "AE0": "a",
    "JH": "q",
    "ZH": "r",
    "AA2": "n",
    "SH": "s",
    "AW1": "t",
    "OY1": "u",
    "AW2": "t",
    "IH2": "g",
    "AE2": "a",
    "EY2": "d",
    "ER1": "k",
    "TH": "v",
    "UH1": "w",
    "UW2": "o",
    "OW2": "b",
    "AY2": "i",
    "UW0": "o",
    "AH2": "c",
    "EH0": "f",
    "AW0": "t",
    "AO2": "e",
    "AO0": "e",
    "UH0": "w",
    "UH2": "w",
    "AA0": "n",
    "AY0": "i",
    "IY2": "m",
    "EY0": "d",
    "ER2": "k",
    "OY2": "u",
    "OY0": "u",
}

iCmd = 0
out_csv = ""
out_txt = ""
g2p = G2p()


def print_command( phrase:str, action:str, itemname:str, label:str, value:str ):
    """ create one line with information about one command, in a format expected
    by the CSV parser on the ESP32-S3 """
    global iCmd, alphabet, out_txt, out_csv
    iCmd += 1
    if (iCmd > MAX_COMMANDS): 
        print("Reached maximum number of commands",file=sys.stderr)
        return

    if len(phrase) > ESP_MN_MAX_PHRASE_LEN - 4:
        print(f"Command '{phrase}' longer than {ESP_MN_MAX_PHRASE_LEN}, dropped",file=sys.stderr)
        return

    labels = g2p(phrase)
    phoneme = ""
    for char in labels:
        if char not in alphabet:
            print("skip %s, not found in alphabet",file=sys.stderr)
            continue
        else:
            phoneme += alphabet[char]
    # grapheme,phoneme,action,itemname,label,value
    out_csv += f'"{phrase}","{phoneme}","{action}","{itemname}","{label}","{value}"\n'
    out_txt += f"{iCmd},{phrase.upper()},{phoneme}\n"


def english_g2p():
    """ Query OpenHAB for a list of items relevant for voice control, and 
    generate a CSV-formatted string with information about all commands,
    in the format expected by the CSV parser on the ESP32-S3"""
    global out_txt, out_csv
	
    headers = { "content-type": "application/json", }
    response = get(OPENHAB_URL, headers=headers)
    items = response.json()

    # Define re to remove anything but alphabet and spaces - multinet doesn't 
    # support them and too lazy to make them words
    pattern = r'[^A-Za-z ]'

    out_csv = "grapheme,phoneme,action,itemname,label,value\n"

    for item in items:
        itemname = item['name']
        groups = item['groupNames']
        # group name gVA marks items that afford a 'switch' intent, 
        # i.e. can be switched on or off with a command like "turn ON|OFF `item label`"
        if 'gVA' in groups:
            friendly_name = item['label'].lower()
            numbers = re.search(r'(\d{1,})', friendly_name)
            if numbers:
                print(f"found numbers in label {friendly_name}",file=sys.stderr)
                for number in numbers.groups():
                    friendly_name = friendly_name.replace(number, f" {num2words(int(number))} ")
            # Just in case so we don't blow up multinet
            friendly_name = friendly_name.replace('_',' ')
            friendly_name = re.sub(pattern, '', friendly_name)
            friendly_name = ' '.join(friendly_name.split())
            #friendly_name = friendly_name.upper()
            for verb in ['turn','switch']:
                for state in ['on','off']:
                    phrase = f'{verb} {state} {friendly_name}'
                    print_command( phrase,"switch",itemname,friendly_name,state)

        # group name gVQ marks items that afford a 'query' intent, 
        # i.e. can be asked about with a command like "what is `item label`""
        elif 'gVQ' in groups:
            friendly_name = item['label']
            phrase = f"what is the {friendly_name}"
            print_command(phrase,"query",itemname,friendly_name,"")

        # group name gVS marks item thats afford a 'scene' intent, 
        # i.e. can be selected with a command like "let's `scene label`""
        elif 'gVS' in groups:
            friendly_name = item['label']
            phrase = f"lets {friendly_name}"
            print_command(phrase,"scene",itemname,friendly_name,"")
            print(f"gVS: label='{friendly_name}'  name='{itemname}'",file=sys.stderr)

        # group name gVD marks items that afford a 'dim' intent, 
        # i.e. can be asked about with a command like "dim `item label` to off|low|medium|high"
        # ESP-SR can't handle numbers, so we can't have a command like "dim map to 50 percent".
        # Instead, we use categorical values low|medium|high and let OpenHAB do the 
        # transformation to 0% 10% 50% 100%
        elif 'gVD' in groups:
            friendly_name = item['label']
            for cat in ["low","medium","high","off"]:
                phrase = f"dim {friendly_name} to {cat}"
                print_command(phrase,"dim",itemname,friendly_name,f"{cat}")

    # for item in items
    with open("../managed_components/espressif__esp-sr/model/multinet_model/fst/commands_en.txt","w") as f:
        f.write(out_txt)

    with open("../data/oh_sr_commands.csv","w") as f:
        f.write(out_csv)

    print(f"----- {iCmd} commands, done",file=sys.stderr)


if __name__ == "__main__":
    english_g2p()
