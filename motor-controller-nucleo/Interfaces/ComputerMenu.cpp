#include "ComputerMenu.h"
#include "usbd_cdc_if.h"
#include <algorithm>

extern "C" {
  #include "tiny_printf.h"
}

#define my_sprintf sprintf

ComputerMenu::ComputerMenu(data_access_fun access_fun)
{
  hall_setting_items[0] = MenuItem{"Up", "", &main_menu_items[1], 1};
  hall_setting_items[1] = MenuItem{"Hall Polatrity", "", nullptr, 0};
  hall_setting_items[2] = MenuItem{"Hall Direction", "", nullptr, 0};
  hall_setting_items[3] = MenuItem{"Mechanical Offset", "", nullptr, 0};
  hall_setting_items[4] = MenuItem{"Electrical Offset", "", nullptr, 0};

  limits_menu_items[0] = MenuItem{"Up", "", &main_menu, 1};
  limits_menu_items[1] = MenuItem{"Max Current", "", nullptr, 0};
  limits_menu_items[2] = MenuItem{"Max Velocity", "", nullptr, 0};
  limits_menu_items[3] = MenuItem{"Max Acceleration", "", nullptr, 0};

  motor_setting_items[0] = MenuItem{"Up", "", &main_menu, 1}; 
  motor_setting_items[1] = MenuItem{"Motor Type", "", nullptr, 0};
  motor_setting_items[2] = MenuItem{"Pole Pairs", "", nullptr, 0};
  motor_setting_items[3] = MenuItem{"Hall Effect Settings", "", hall_setting_items.data(), hall_setting_items.size()};
  motor_setting_items[4] = MenuItem{"Hall Effect Auto Setup", "", nullptr, 0};

  open_loop_items[0] = MenuItem{"Up", "", &main_menu, 1};
  open_loop_items[1] = MenuItem{"Max Current", "", nullptr, 0};
  open_loop_items[2] = MenuItem{"Max Voltage", "", nullptr, 0};
  open_loop_items[3] = MenuItem{"Acceleration", "", nullptr, 0};

  main_menu_items[0] = MenuItem{"Limits", "", limits_menu_items.data(), limits_menu_items.size()};
  main_menu_items[1] = MenuItem{"Motor Settings", "", motor_setting_items.data(), motor_setting_items.size()};
  main_menu_items[2] = MenuItem{"Open Loop Settings", "", open_loop_items.data(), open_loop_items.size()};
  main_menu_items[3] = MenuItem{"Save Settings", "Save settings to flash", nullptr, 0};
  //main_menu_items[4] = MenuItem{"", "", nullptr, 0};

  main_menu = MenuItem{"Main Menu", "", main_menu_items.data(), main_menu_items.size()};

  current_menu = &main_menu;
  leaf_item = nullptr;
  data_access = access_fun;
}


void ComputerMenu::display_menu(int menu_num)
{
  // Make my buffer the same length as the USB buffer (defined in usbd_cdc_if.c)
  const uint8_t BUFFSIZE = 128; // same length as the USB buffer
  char buff[BUFFSIZE] = {0};

  //lambda function for going up
  auto go_up = [&, this]()
  {
    return current_menu;
  };

  strcpy(buff, "\f");
  CDC_Transmit_FS((uint8_t *) buff, strlen(buff)+1);
  HAL_Delay(1);

  MenuItem* display_menu = current_menu;
  // what to do if the recieved command is invalid or KEEP_MENU
  if (menu_num == KEEP_MENU || 
      menu_num < UP_LEVEL   ||
      menu_num >= current_menu->menu_items)
  {
    display_menu = (leaf_item != nullptr) ?  leaf_item : current_menu;
  }
  // handle the UP_MENU case
  else if (menu_num == UP_LEVEL)
  {
    if(leaf_item != nullptr){
      // escape the leaf menu
      leaf_item = nullptr;
    }
    else
    {
      // if not the main menu, go up a level
      auto is_main_menu = strcmp(current_menu->name_str, "Main Menu") == 0;
      if(!is_main_menu)
      {
        // go up
        current_menu = current_menu->menu[0].menu;
      }
    }
    display_menu = current_menu;
  }
  // what to do if the data is valid 
  else {
    // check if a leaf item is selected
    if (leaf_item != nullptr)
    {
      display_menu = leaf_item;
    }
    else if (current_menu->menu[menu_num].menu == nullptr)
    {
      leaf_item = &current_menu->menu[menu_num];
      display_menu = leaf_item;
      //current menu remains unchanged
    }
    else 
    {
      leaf_item = nullptr;
      current_menu = &current_menu->menu[menu_num];

      // check if the selected menu is up
      auto up_selected = strcmp(current_menu->name_str, "Up") == 0;
      if (up_selected){
        current_menu = current_menu->menu;
      }

      display_menu = current_menu;
    }
  }

  if (leaf_item != nullptr)
  {
    display_leaf_item(*leaf_item, buff);
  }
  else 
  {
    list_menu_items(*display_menu, buff);
  }
}

const char* ComputerMenu::get_menu_item_str(const MenuItem &item, int item_num, char *buff) const
{
  if(item.menu == nullptr)
  {
    char value_buff[10] = {0};
    auto menu_val = data_access(value_buff, item.param, false, 0);
    my_sprintf(buff, "%d) %s:\t%s\n\r", item_num, item.name_str, menu_val);
  }
  else
  {
    my_sprintf(buff, "%d) %s->\n\r", item_num, item.name_str);
  }
  
  return buff;
}

void ComputerMenu::list_menu_items(const MenuItem& menu, char *buff) {
  auto print = [](const char* buff) {
    CDC_Transmit_FS((uint8_t *) buff, strlen(buff)+1);
    HAL_Delay(1);
  };

  my_sprintf(buff, "%s:\n\r", menu.name_str); 
  print(buff);
  std::fill(buff, buff+20, '-');
  buff[20] = '\0';
  strcat(buff, "\n\r");
  print(buff);

  for(int i = 0; i < menu.menu_items; i++)
  {
    get_menu_item_str(menu.menu[i], i, buff);
    print(buff);
  }
  strcpy(buff, "\n\r");
  print(buff);
}

void ComputerMenu::display_leaf_item(const MenuItem& menu, char *buff)
{
  auto print = [](const char* buff) {
    CDC_Transmit_FS((uint8_t *) buff, strlen(buff)+1);
    HAL_Delay(1);
  };

  my_sprintf(buff, "%s:\n\r", menu.name_str); 
  print(buff);
  std::fill(buff, buff+20, '-');
  buff[20] = '\0';
  strcat(buff, "\n\r");
  print(buff);

  data_access(buff, menu.param, false, 0);
  strcat(buff, "\n\r");
  print(buff);
}