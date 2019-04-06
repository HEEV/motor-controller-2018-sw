#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <cstdint>
#include <array>

enum class menu_commands_t : std::uint8_t
{
  UP_LEVEL,
  KEEP_MENU,
  DATA
};

struct menu_cmd_t
{
  menu_commands_t cmd;
  std::int32_t data; 
};

class CommandParser
{
private:
  std::array<char, 64> command_buff = {0};
  std::uint8_t command_len = 0;

public:
  void add_to_buffer(const std::uint8_t* buff, std::uint32_t len);
  menu_cmd_t parse_buffer();
  const char* data();

  CommandParser() = default;
  ~CommandParser() = default;
};

#endif // COMMAND_PARSER_H