#ifndef _COMPUTER_MENU_H_
#define _COMPUTER_MENU_H_

#include <array>
#include <cstdint>

// forward declaration of enum
enum class MotorControllerParameter_t : std::uint8_t;
class ComputerInterface; 

class MenuItem {
public:
  const char* name_str;
  const char* menu_description;
  MotorControllerParameter_t param;

  MenuItem *menu;
  int menu_items;
};

class ComputerMenu {
public:
  ComputerMenu(ComputerInterface* ci);  

  void display_menu(int menu_num);

  const static int KEEP_MENU = -1;
  const static int UP_LEVEL = -2;

private:
  const char* get_menu_item_str(const MenuItem &item, int item_num, char* buff) const;
  void list_menu_items(const MenuItem &item, char *buff);
  void display_leaf_item(const MenuItem &item, int command, char *buff);

  MenuItem main_menu;
  std::array<MenuItem, 4> main_menu_items;

  // sub menus
  std::array<MenuItem, 4> limits_menu_items;
  std::array<MenuItem, 4> open_loop_items;
  std::array<MenuItem, 6> motor_setting_items;
  std::array<MenuItem, 5> hall_setting_items;
  std::array<MenuItem, 5> general_setting_items;

  MenuItem *current_menu;
  MenuItem *leaf_item;
  ComputerInterface *compInterface;
};

#endif //_COMPUTER_MENU_H_