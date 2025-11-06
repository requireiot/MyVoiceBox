#pragma once

#include "esp32-hal-sr.h"


/**
 * @brief Information about one SR command.
 * Contains pointers to strings owned by someone else
 */
struct command_info_t {
    char* grapheme;
    char* phoneme;
    char* action;
    char* itemname;
    char* label;
    char* value;
};


/**
 * @brief Information about SR commands read from CSV file.
 * Contains pointers to strings owned by someone else
 * 
 */
class SR_Commands {
  protected:
    size_t _n_commands;             /// number of commands parsed from CSV text
    command_info_t* _commands;      /// array with info about all commands
    char* _text;
    command_info_t* add_command();
  public:
    SR_Commands() {};
    size_t count() { return _n_commands; }
    bool parse_csv( const char* csv );
    bool fill();
    sr_cmd_t *build_sr_commands();
    const command_info_t* get_info( int id );
    const command_info_t* operator[](int id) { return get_info(id); }
};
