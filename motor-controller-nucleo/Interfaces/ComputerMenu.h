#ifndef _COMPUTER_MENU_H_
#define _COMPUTER_MENU_H_

#include <array>

class MenuItem {
public:
  const char* name_str;
  const char* menu_description;

  MenuItem *menu;
  size_t menu_items;
};

class ComputerMenu {
public:
  ComputerMenu();  

  void display_menu(int menu_num);

private:
  const char* get_menu_item_str(const MenuItem &item, int item_num, char* buff) const;
  void list_menu_items(const MenuItem &item, char *buff);
  MenuItem main_menu;
  std::array<MenuItem, 4> main_menu_items;

  // sub menus
  std::array<MenuItem, 4> limits_menu_items;
  std::array<MenuItem, 4> open_loop_items;
  std::array<MenuItem, 5> motor_setting_items;
  std::array<MenuItem, 5> hall_setting_items;

  MenuItem *current_menu;
};

#endif //_COMPUTER_MENU_H_