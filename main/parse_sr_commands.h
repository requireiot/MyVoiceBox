#pragma once

#include "esp32-hal-sr.h"

/**
 * @brief Information about SR commands read from CSV file.
 * Contains pointers to strings owned by someone else
 * 
 */
struct command_info_t {
    char* grapheme;
    char* phoneme;
    char* action;
    char* itemname;
    char* label;
    char* value;
};

/// array with info about all commands
extern struct command_info_t *command_infos;
/// number of commands parsed from CSV text
extern size_t n_sr_commands;


bool parse_commands_csv( const char* csv );
bool fill_sr_commands_esp();
sr_cmd_t * build_sr_commands();

command_info_t* get_sr_info_from_id( int id );