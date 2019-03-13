#ifndef _COMPUTER_MENU_H_
#define _COMPUTER_MENU_H_

#include <array>

class MenuItem;

class MenuItem {
public:
  const char* name_str;
  const char* menu_description;  
  MenuItem *sub_menu;
};

class ComputerMenu {
  
};

#endif //_COMPUTER_MENU_H_