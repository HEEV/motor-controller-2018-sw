#include "ComputerMenu.h"
#include "usbd_cdc_if.h"
#include <algorithm>
#include <functional>
#include "settings_structs.h"
#include "ComputerInterface.h"
#include "access_settings.h"

extern "C" {
  #include "tiny_printf.h"
}

#define my_sprintf sprintf


// constructor in ComputerMenu_init.cpp


void ComputerMenu::display_menu(menu_cmd_t command)
{
  // Make my buffer the same length as the USB buffer (defined in usbd_cdc_if.c)
  const uint8_t BUFFSIZE = 128; // same length as the USB buffer
  char buff[BUFFSIZE] = {0};

  // clear the page
  strcpy(buff, "\f");
  compInterface->println(buff);

  if (navigate_menu(command))
  {
    // make sure we don't try and change a setting when 
    // we don't want to (changing settings looks like a command)
    // this might get passed on to the display_leaf_item function
    // and would do undesireable things
    command.cmd = menu_commands_t::KEEP_MENU;
  }

  // check that we are a leaf item
  auto is_leaf_item = current_menu->sub_menu == nullptr;
  if (is_leaf_item)
  {
    display_leaf_item(*current_menu, command, buff);
  }
  else 
  {
    list_menu_items(*current_menu, buff);
  }
}

bool ComputerMenu::navigate_menu(menu_cmd_t command)
{
  auto is_leaf_item = current_menu->sub_menu == nullptr;
  auto is_cmd_menu_range = !(command.data >= current_menu->sub_menu_items || command.data < 0);

  bool menu_changed = false;

  // what to do if the recieved command is invalid or KEEP_MENU
  if (command.cmd == menu_commands_t::KEEP_MENU || 
     (command.cmd == menu_commands_t::DATA && !is_cmd_menu_range)) 
  {
    // keep the current menu
    current_menu = current_menu;
  }
  // handle the UP_MENU case
  else if (command.cmd == menu_commands_t::UP_LEVEL)
  {
    // if not the main menu, go up a level
    auto is_main_menu = current_menu->parent_menu == nullptr;
    if(!is_main_menu)
    {
      // go up
      current_menu = current_menu->parent_menu;
      menu_changed = true;
    }
  }
  // what to do if the data is valid and not on a leaf node
  else if (!is_leaf_item) {
    // go to the user selected menu
    current_menu = &current_menu->sub_menu[command.data];
    menu_changed = true;

    // if the user selected the "UP" option, go to the actual menu
    if (current_menu->parent_menu == current_menu->sub_menu){
      current_menu = current_menu->parent_menu;
    }
  }

  return menu_changed;
}

const char* ComputerMenu::get_menu_item_str(const MenuItem &item, int item_num, char *buff) const
{
  char name_buff[64] = {0};
  strcpy(name_buff, item.name_str);

  if(item.sub_menu == nullptr)
  {
    char value_buff[24] = {0};
    auto menu_val = compInterface->access_setting_value(value_buff, item.param, false, 0);
    
    strcat(name_buff, ":");

    my_sprintf(buff, "%d) %-25s\t%s", item_num, name_buff, menu_val);
  }
  else
  {
    strcat(name_buff, "->");
    my_sprintf(buff, "%d) %-25s", item_num, name_buff);
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

  // print a horizontal line
  auto hline = [](char* buff)
  {
    std::fill(buff, buff+79, '-');
    buff[79] = '\0';
    return buff;
  };

  compInterface->println(get_menu_name(menu, buff));
  compInterface->println(hline(buff));

  auto is_leaf_node = menu.sub_menu == nullptr;
  if (is_leaf_node) {
    my_sprintf(buff, "%s", menu.menu_description);
    compInterface->println(buff);
  }
  else {
    my_sprintf(
      buff, 
      "Enter a number between 0 and %d to select a submenu item", 
      menu.sub_menu_items-1
    );

    compInterface->println(buff);
  }
  strcpy(buff, "Use the ESC or \'u\' key to go up");
  compInterface->println(buff);
  compInterface->println(hline(buff));
}

void ComputerMenu::display_common_settings(char* buff)
{
  char buff1[10];
  char buff2[10];
  char buff3[10];
  my_sprintf(buff, "Setpoint: %-15s Velocity: %-15s Current: %-15s", 
    get_setting_as_string(buff1, MotorControllerParameter_t::SETPOINT),
    get_setting_as_string(buff2, MotorControllerParameter_t::VELOCITY),
    get_setting_as_string(buff3, MotorControllerParameter_t::CURRENT));
  compInterface->println(buff);

  my_sprintf(buff, "Motor Temperature: %-15s Transistor Temperature: %-15s", 
    get_setting_as_string(buff1, MotorControllerParameter_t::MOTOR_TEMPERATURE),
    get_setting_as_string(buff2, MotorControllerParameter_t::TRANSISTOR_TEMPERATURE));
  compInterface->println(buff);
}

void ComputerMenu::list_menu_items(const MenuItem& menu, char *buff) {
  display_menu_heading(menu, buff);

  for(int i = 0; i < menu.sub_menu_items; i++)
  {
    get_menu_item_str(menu.sub_menu[i], i, buff);
    compInterface->println(buff);
  }
}

void ComputerMenu::display_leaf_item(const MenuItem& menu, menu_cmd_t command, char *buff)
{
  display_menu_heading(menu, buff);

  bool write_setting = false;
  int32_t write_value = 0;
  if (command.cmd == menu_commands_t::DATA)
  {
    write_setting = true;
    write_value = command.data;
  }

  // special case for live values item 
  if(menu.param == MotorControllerParameter_t::LIVE_VALUES){
    display_common_settings(buff);
  }
  else
  {
    strcpy(buff, compInterface->access_setting_value(buff, menu.param, write_setting, write_value));
    compInterface->println(buff);
  }
}