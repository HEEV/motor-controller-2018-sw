#include "ComputerMenu.h"
#include "usbd_cdc_if.h"
#include <algorithm>
#include <functional>
#include "settings_structs.h"
#include "ComputerInterface.h"

extern "C" {
  #include "tiny_printf.h"
}

#define my_sprintf sprintf

// constructor in ComputerMenu_init.cpp

void ComputerMenu::display_menu(int menu_num)
{
  // Make my buffer the same length as the USB buffer (defined in usbd_cdc_if.c)
  const uint8_t BUFFSIZE = 128; // same length as the USB buffer
  char buff[BUFFSIZE] = {0};

  strcpy(buff, "\f");
  CDC_Transmit_FS((uint8_t *) buff, strlen(buff)+1);
  HAL_Delay(1);

  auto is_leaf_item = current_menu->sub_menu == nullptr;

  // what to do if the recieved command is invalid or KEEP_MENU
  if (menu_num == KEEP_MENU || 
      menu_num < UP_LEVEL   ||
      menu_num >= current_menu->sub_menu_items)
  {
    // keep the current menu
    current_menu = current_menu;
  }
  // handle the UP_MENU case
  else if (menu_num == UP_LEVEL)
  {
    // if not the main menu, go up a level
    auto is_main_menu = current_menu->parent_menu == nullptr;
    if(!is_main_menu)
    {
      // go up
      current_menu = current_menu->parent_menu;
    }
  }
  // what to do if the data is valid and not on a leaf node
  else if (!is_leaf_item) {
    // go to the user selected menu
    current_menu = &current_menu->sub_menu[menu_num];

    // make sure we don't set the value accidently
    menu_num = KEEP_MENU;

    // if the user selected the "UP" option, go to the actual menu
    if (current_menu->parent_menu == current_menu->sub_menu){
      current_menu = current_menu->parent_menu;
    }
  }

  if (is_leaf_item)
  {
    display_leaf_item(*current_menu, menu_num, buff);
  }
  else 
  {
    list_menu_items(*current_menu, buff);
  }
}

const char* ComputerMenu::get_menu_item_str(const MenuItem &item, int item_num, char *buff) const
{
  if(item.sub_menu == nullptr)
  {
    char value_buff[24] = {0};
    auto menu_val = compInterface->access_setting_value(value_buff, item.param, false, 0);
    my_sprintf(buff, "%d) %s:\t%s\n\r", item_num, item.name_str, menu_val);
  }
  else
  {
    my_sprintf(buff, "%d) %s->\n\r", item_num, item.name_str);
  }
  
  return buff;
}

void ComputerMenu::display_menu_heading(const MenuItem& menu, char *buff)
{

  // recursively get the name of the menu
  std::function<char* (const MenuItem&, char*)> 
  get_menu_name = [&] (const MenuItem& menu, char* buff) 
  { 
    if (menu.parent_menu != nullptr){
      // return the name of the parent and my name
      my_sprintf(buff, "%s->%s", get_menu_name(*menu.parent_menu, buff), menu.name_str);
      return buff;
    }
    else
    {
      // return the menu's name
      my_sprintf(buff, "%s", menu.name_str);
      return buff; 
    }
  }; 

  get_menu_name(menu, buff);
  compInterface->println(buff);
  std::fill(buff, buff+79, '-');
  buff[79] = '\0';
  compInterface->println(buff);
}

void ComputerMenu::list_menu_items(const MenuItem& menu, char *buff) {
  auto print = [](const char* buff) {
    CDC_Transmit_FS((uint8_t *) buff, strlen(buff)+1);
    HAL_Delay(1);
  };

  display_menu_heading(menu, buff);

  for(int i = 0; i < menu.sub_menu_items; i++)
  {
    get_menu_item_str(menu.sub_menu[i], i, buff);
    print(buff);
  }
  strcpy(buff, "\n\r");
  print(buff);
}

void ComputerMenu::display_leaf_item(const MenuItem& menu, int command, char *buff)
{
  display_menu_heading(menu, buff);

  bool write_setting = false;
  int32_t write_value = 0;
  if (command != KEEP_MENU)
  {
    write_setting = true;
    write_value = command;
  }
  
  strcpy(buff, compInterface->access_setting_value(buff, menu.param, write_setting, write_value));
  compInterface->println(buff);
}