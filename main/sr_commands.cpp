/**
 * @file        parse_sr_commands.cpp
 * @project     ESP_SR test projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2025-10-15
 * tabsize  4
 * 
 * This Revision: $Id: sr_commands.cpp 1930 2025-11-19 13:43:46Z  $
 */

/*
   Copyright (C) 2025 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, 
   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
   obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief   parse CSV-formatted string into speech commands information
 * 
 * CSV file is formatted like so
 * 
 *  grapheme,phoneme,action,itemname,label,value
 *  "Turn on the light","TkN nN jc LiT","switch","KitchenLight","kitchen light,"ON"
 *  "Turn off the light","TkN eF jc LiT","switch","KitchenLight","kitchen light,"OFF"
 */


#include <stddef.h>
#include <string.h>
#include <ctype.h>

#include "esp_mn_speech_commands.h"
#include "esp32-hal-sr.h"
#include "esp32-hal-log.h"
#include "sdkconfig.h"

#include "sr_commands.h"

/// @brief  array with info about all commands
//struct command_info_t *sr_command_infos = NULL;
/// @brief number of commands parsed from CSV text
//size_t n_sr_commands = 0;


/**
 * @brief Trim whilespace at the beginning and end of `line`
 * 
 * @param line    0-terminated string to trim
 * @return char*  trimmed string, a pointer into the original string
 */
static char* trim_line( char* line )
{
    if (line==NULL || *line=='\0') return line;
    char* line_end = line + strlen(line)-1;
    while (isspace(*line_end)) *line_end-- = '\0';
    while (isspace(*line)) line++;
    return line;
}


/**
 * @brief Remove double quotes at the beginning and end of `line`
 * 
 * @param line      0-terminated string to work on
 * @return char*    trimmed string, a pointer into the original string
 */
static char* trim_quotes( char* line )
{
    if (line==NULL || *line=='\0') return line;
    char* line_end = line + strlen(line)-1;
    if (*line_end=='"') *line_end-- = '\0';
    if (*line=='"') line++;
    return line;
}


/**
 * @brief Get line from multi-line string in `*text`, advance `*text` to after the line
 * Nothing is _copied_, we just insert '\0' where needed
 *
 * @param text      pointer to pointer to multi-line string, 0-terminated
 * @return char*    pointer to the first line in the multi-line string
 */
static char* next_line( char** text )
{
    char* eol = strchr(*text,'\n');
    if (eol) {
        *eol = '\0';
        char* line = *text;
        *text = eol+1;
        return line;
    } else {
        char* line = *text;
        *text = NULL;
        return line;
    }
}


/**
 * @brief Resize `sr_command_infos` to hold one one more element
 * 
 * @return command_info_t*  pointer to new element
 */
command_info_t* SR_Commands::add_command() 
{
    _n_commands++;
    _commands = (command_info_t*) realloc( _commands, _n_commands * sizeof(command_info_t) );
    return _commands + (_n_commands-1);
}


/**
 * @brief Parse CVS-formatted multi-line string in `csv` and fill `sr_command_infos` array.
 * Creates (on the heap) a copy of the `csv` source, this copy persists after return 
 * from this function, so pointers to strings within the copy remain valid.
 * 
 * @param csv 
 * @return true 
 * @return false 
 */
bool SR_Commands::parse_csv( const char* csv )
{
    // let's make a copy of the CSV text on the heap so we can chop it up
    if (_text) free(_text);
    _text = (char*) malloc(strlen(csv)+1);
    strcpy( _text, csv );

    // free array buffer, if it has been used before
    if (_commands) {
        free(_commands);
        _commands = NULL;
    }

    log_d("parsing CSV text = \n'%s'", _text );

    char* line;
    char* ptext = _text;
    while (isspace(*ptext)) ptext++;    // ignore leading empty lines
    line = trim_line(next_line(&ptext));
    log_d("header line is '%s'", line);
    if (strcmp( line, "grapheme,phoneme,action,itemname,label,value" ))
        return false;
    while ( (line=next_line(&ptext)) ) {
        log_d("Parsing line '%s'",line);
        if (line[0]==0) break;   // ignore empty line
        command_info_t* pcommand = add_command();
        pcommand->grapheme = trim_quotes( strtok(line,",\"") );
        pcommand->phoneme  = trim_quotes( strtok(NULL,",\"") );
        pcommand->action   = trim_quotes( strtok(NULL,",\"") );
        pcommand->itemname = trim_quotes( strtok(NULL,",\"") );
        pcommand->label    = trim_quotes( strtok(NULL,",\"") );
        pcommand->value    = trim_quotes( strtok(NULL,",\"") );
    }
    log_i("parsed %d commands",(int)_n_commands);
    return true;
}


/**
 * @brief Feed information about speech commands directly to ESP-SR
 * 
 * @return true 
 * @return false 
 */
bool SR_Commands::fill()
{
// if Multinet7 is selected, then we rely on specifying commands at compile time
//#if defined(CONFIG_SR_MN_EN_MULTINET7_QUANT)
//    return true;
//#endif

    bool ok;
    esp_err_t ret;
    ok = (ESP_OK==esp_mn_commands_clear());
    int i; command_info_t* pinfo;
    for (i=1, pinfo=_commands; i <= _n_commands; i++, pinfo++) {
#if CONFIG_SR_MN_EN_MULTINET7_QUANT
        ret = esp_mn_commands_phoneme_add(i,pinfo->grapheme,pinfo->phoneme);
        ok = ok && (ESP_OK==ret);
        if (ret != ESP_OK) {
            log_e("esp_mn_commands_phoneme_add(%d) returns %d",i,(int)ret);
            break;
        }
#elif CONFIG_SR_MN_EN_MULTINET6_QUANT
        ok = ok && (ESP_OK==esp_mn_commands_add(i,pinfo->grapheme));
#else
        ok = ok && (ESP_OK==esp_mn_commands_add(i,pinfo->phoneme));
#endif
    }
    log_i("defined %d commands",i-1);
    ok = ok && (NULL==esp_mn_commands_update());
    return ok;
}


/**
 * @brief Build an array of structures expected by ESP_SR::begin()
 * 
 * @return sr_cmd_t*  pointer to array, or NULL if out of memory
 */
sr_cmd_t* SR_Commands::build_sr_commands()
{
    sr_cmd_t* sr_commands = (sr_cmd_t*)malloc(_n_commands * sizeof(sr_cmd_t));
    if (sr_commands==NULL) return NULL;
    for (int i=0; i<_n_commands; i++) {
        sr_commands[i].command_id = i+1;
        strncpy( sr_commands[i].str, _commands[i].grapheme, SR_CMD_STR_LEN_MAX );
        strncpy( sr_commands[i].phoneme, _commands[i].phoneme, SR_CMD_PHONEME_LEN_MAX );
    }
    return sr_commands;
}


/**
 * @brief Get command info associated with `id`. If the ids presented to ESP-SR 
 * don't start with 0, then the result of this function is not equal to 
 * sr_command_infos[id]
 * 
 * @param id                id returned from ESP-SR
 * @return command_info_t*  pointer to corresponding command info
 */
const command_info_t* SR_Commands::get_info( int id )
{
    if ((id > 0) && (id <= _n_commands))
        return _commands+(id-1);
    else
        return NULL;
}
