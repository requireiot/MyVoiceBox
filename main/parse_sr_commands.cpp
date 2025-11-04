/**
 * @file        parse_sr_commands.cpp
 * @project     ESP_SR test projects
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2025-10-15
 * tabsize  4
 * 
 * This Revision: $Id: parse_sr_commands.cpp 1906 2025-11-01 12:01:24Z  $
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
 *  grapheme,phoneme,action,topic,value
 *  "Turn on the light","TkN nN jc LiT","switch","KitchenLight","kitchen light,"ON"
 *  "Turn off the light","TkN eF jc LiT","switch","KitchenLight","kitchen light,"OFF"
 */


#include <stddef.h>
#include <string.h>
#include <ctype.h>

//#include "ESP_SRx.h"
#include "esp_mn_speech_commands.h"
#include "esp32-hal-sr.h"
#include "esp32-hal-log.h"
#include "parse_sr_commands.h"
#include "sdkconfig.h"

/// @brief  array with info about all commands
struct command_info_t *command_infos = NULL;
/// @brief number of commands parsed from CSV text
size_t n_sr_commands = 0;


/**
 * @brief Resize `command_infos` to hold one one element
 * 
 * @return command_info_t*  pointer to new element
 */
static command_info_t* add_command() 
{
    n_sr_commands++;
    command_infos = (command_info_t*) realloc( command_infos, n_sr_commands * sizeof(command_info_t) );
    return command_infos + (n_sr_commands-1);
}


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
 * @brief Parse CVS-formatted multi-line string in `csv` and fill `command_infos` array.
 * Creates (on the heap) a coy of the `csv` source, this copy persists after return 
 * from this function, so pointers to strings within the copy remain valid.
 * 
 * @param csv 
 * @return true 
 * @return false 
 */
bool parse_commands_csv( const char* csv )
{
    // let's make a copy of the CSV text on the heap so we can chop it up
    static char* text = NULL;
    if (text) free(text);
    text = (char*) malloc(strlen(csv)+1);
    strcpy( text, csv );
    log_d("parsing CSV text = \n'%s'", text );

    char* line;
    char* ptext = text;
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
    log_i("parsed %d commands",(int)n_sr_commands);
    return true;
}


/**
 * @brief Feed information about speech commands directly to ESP-SR
 * 
 * @return true 
 * @return false 
 */
bool fill_sr_commands_esp()
{
// if Multinet7 is selected, then we rely on specifying commands at compile time
#if defined(CONFIG_SR_MN_EN_MULTINET7_QUANT)
    return true;
#endif

    bool ok;
    ok = (ESP_OK==esp_mn_commands_clear());
    int i; command_info_t* pinfo;
    for (i=1, pinfo=command_infos; i <= n_sr_commands; i++, pinfo++) {
#if defined( CONFIG_SR_MN_EN_MULTINET7_QUANT) || defined(CONFIG_SR_MN_EN_MULTINET6_QUANT)
        ok = ok && (ESP_OK==esp_mn_commands_add(i,pinfo->grapheme));
#else
        ok = ok && (ESP_OK==esp_mn_commands_add(i,pinfo->phoneme));
#endif
    }
    ok = ok && (NULL==esp_mn_commands_update());
    return ok;
}


/**
 * @brief Build an array of structures expected by ESP_SR::begin()
 * 
 * @return sr_cmd_t*  pointer to array, or NULL if out of memory
 */
sr_cmd_t* build_sr_commands()
{
    sr_cmd_t* sr_commands = (sr_cmd_t*)malloc(n_sr_commands * sizeof(sr_cmd_t));
    if (sr_commands==NULL) return NULL;
    for (int i=0; i<n_sr_commands; i++) {
        sr_commands[i].command_id = i+1;
        strncpy( sr_commands[i].str, command_infos[i].grapheme, SR_CMD_STR_LEN_MAX );
        strncpy( sr_commands[i].phoneme, command_infos[i].phoneme, SR_CMD_PHONEME_LEN_MAX );
    }
    return sr_commands;
}


/**
 * @brief Get command info associated with `id`. If the ids presented to ESP-SR 
 * don't start with 0, then the result of this function is not equal to 
 * command_infos[id]
 * 
 * @param id                id returned from ESP-SR
 * @return command_info_t*  pointer to corresponding command info
 */
command_info_t* get_sr_info_from_id( int id )
{
    if ((id > 0) && (id <= n_sr_commands))
        return command_infos+(id-1);
    else
        return NULL;
}
