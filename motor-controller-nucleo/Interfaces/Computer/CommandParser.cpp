#include "CommandParser.h"
#include <cctype>

using uint8_t  = std::uint8_t;
using uint32_t = std::uint32_t;
using int32_t  = std::int32_t;

void CommandParser::add_to_buffer(const uint8_t *buff, uint32_t len)
{
  // copy the characters from the buffer into the array
  char* cpyPtr = &command_buff[command_len];

  if (cpyPtr == &command_buff.back())
  {
    // always put one new character from the input buffer into our
    // buffer
    cpyPtr = cpyPtr-1;
    command_len--;
  }

  while ( len > 0 && cpyPtr < &command_buff.back() )
  {
    // check for numeric character, enter, '-', or ESC.
    if( isdigit(*buff) )
    {
      // copy the char from the input buffer
      *cpyPtr++ = *buff;
      command_len++;
    }
    // copy in '-' if it is the first character
    else if( *buff == '-' && cpyPtr == command_buff.data())
    {
      // copy the char from the input buffer
      *cpyPtr++ = *buff;
      command_len++;
    }
    // check for control sequence
    else if (*buff== '\n'    || *buff == '\r' || 
             *buff == '\x1b' || *buff == 'u')
    {
      // copy the char from the input buffer
      *cpyPtr++ = *buff;
      command_len++;

      // exit the loop
      break;
    }
    // check for backspace or delete
    else if(*buff == '\b' || *buff == '\x7F')
    {
      // if we haven't backspaced to the beginning of the buffer, decriment the parsed 
      // command
      if (command_len > 0)
      {
        cpyPtr--;
        command_len--;
      }
    }

    // go to the next spot in the buffer 
    buff++;
    len--;
  }

  // null terminate the string thus far
  *cpyPtr = '\0';

  // make sure we null terminate
  command_buff.back() = '\0';
}

menu_cmd_t CommandParser::parse_buffer()
{

  menu_cmd_t user_cmd = { menu_commands_t::KEEP_MENU, 0 };

  // loop through the command string and make sure the characters are
  // numeric or accepted commands
  char *cmd = command_buff.data();     
  
  while (*cmd != '\0')
  {
    // check for enter or ESC
    if(*cmd == '\n'   || *cmd == '\r' || 
       *cmd == '\x1b' || *cmd == 'u')
    {
      // escape character
      if(*cmd == '\x1B' || *cmd == 'u')
        { user_cmd.cmd = menu_commands_t::UP_LEVEL; }
      else 
      {
        // grab the integer from the buffer
        if(sscanf(command_buff.data(), "%ld", &user_cmd.data) !=1 )
          { user_cmd.cmd = menu_commands_t::KEEP_MENU; }
        else 
          { user_cmd.cmd = menu_commands_t::DATA; }
      }

      // clear the buffer
      command_len = 0;
      command_buff.fill('\0');

      // done with the loop
      break;
    }

    // move on to the next command character
    cmd++;
  }

  return user_cmd;
}

const char* CommandParser::data()
{
  return command_buff.data();
}
