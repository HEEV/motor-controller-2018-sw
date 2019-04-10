#ifndef _COMPUTER_MENU_H_
#define _COMPUTER_MENU_H_

#include <array>
#include <cstdint>
// for the menu_cmd_t struct
#include <CommandParser.h>

// forward declaration of enum
enum class MotorControllerParameter_t : std::uint8_t;
class ComputerInterface; 

class MenuItem {
public:
  const char* name_str;
  const char* menu_description;
  MotorControllerParameter_t param;

  const MenuItem *sub_menu;
  const MenuItem *parent_menu;
  int sub_menu_items;
};

class ComputerMenu {
public:
  ComputerMenu(ComputerInterface* ci);  

  void display_menu(menu_cmd_t command);

  const static int KEEP_MENU = -1;
  const static int UP_LEVEL = -2;

private:
  // manage our position in the menu
  bool navigate_menu(menu_cmd_t command);

  // print the formatted name and value of a menu item
  const char* get_menu_item_str(const MenuItem &item, int item_num, char* buff) const;

  // print the menu title and the secondary string for the menu
  void display_menu_heading(const MenuItem &item, char* buff);

  // print some common settings live
  void display_common_settings(char* buff);

  // list sub-menu items
  void list_menu_items(const MenuItem &item, char *buff);

  // print the leaf item
  void display_leaf_item(const MenuItem &item, menu_cmd_t command, char *buff);

  MenuItem main_menu;
  std::array<MenuItem, 5> main_menu_items;

  // sub menus
  std::array<MenuItem, 4> limits_menu_items;
  std::array<MenuItem, 7> pi_menu_items;
  std::array<MenuItem, 4> open_loop_items;
  std::array<MenuItem, 7> motor_setting_items;
  std::array<MenuItem, 5> hall_setting_items;
  std::array<MenuItem, 7> general_setting_items;

  const MenuItem *current_menu;
  ComputerInterface *compInterface;
};

#endif //_COMPUTER_MENU_H_