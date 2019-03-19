#include "ComputerMenu.h"
#include "usbd_cdc_if.h"
#include <algorithm>

extern "C" {
  #include "tiny_printf.h"
}

#define my_sprintf sprintf

ComputerMenu::ComputerMenu()
{
  hall_setting_items[0] = MenuItem{"Up", "", &main_menu_items[1], 1};
  hall_setting_items[1] = MenuItem{"Hall Polatrity", "", nullptr, 0};
  hall_setting_items[2] = MenuItem{"Hall Direction", "", nullptr, 0};
  hall_setting_items[3] = MenuItem{"Mechanical Offset", "", nullptr, 0};
  hall_setting_items[4] = MenuItem{"Electrical Offset", "", nullptr, 0};

  limits_menu_items[0] = MenuItem{"Up", "", &main_menu, 1};
  limits_menu_items[1] = MenuItem{"Max Current", "", nullptr, 0};
  limits_menu_items[2] = MenuItem{"Max Voltage", "", nullptr, 0};
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
}

const char* ComputerMenu::get_menu_item_str(const MenuItem &item, int item_num, char *buff) const
{
  auto sub_menu_char = [](const MenuItem &item) {
    return (item.menu == nullptr) ? "" : "->"; 
  };

  my_sprintf(buff, "%d) %s%s\n\r", item_num, item.name_str, sub_menu_char(item));

  return buff;
}

void ComputerMenu::display_menu(int menu_num)
{
  // Make my buffer the same length as the USB buffer (defined in usbd_cdc_if.c)
  const uint8_t BUFFSIZE = 128; // same length as the USB buffer
  char buff[BUFFSIZE] = {0};

  strcpy(buff, "\f");
  CDC_Transmit_FS((uint8_t *) buff, strlen(buff)+1);
  HAL_Delay(1);

  // check that the recieved menu number is valid or we were told to keep current menu
  if (menu_num == KEEP_MENU || 
      menu_num <  UP_LEVEL  || 
      menu_num >= current_menu->menu_items)
  {
    list_menu_items(*current_menu, buff);
  }
  else {
    auto is_main_menu = strcmp(current_menu->name_str, "Main Menu") == 0;
    if(!is_main_menu  && menu_num == UP_LEVEL){
      menu_num = 0; // up level is the first item in each menu
    }
    else {
      strcpy(buff, "\a");
      CDC_Transmit_FS((uint8_t *) buff, strlen(buff)+1);
      HAL_Delay(1);
    }
    // go to the selected menu
    current_menu = current_menu->menu[menu_num].menu != nullptr ? &current_menu->menu[menu_num] : current_menu;

    auto up_selected = strcmp(current_menu->name_str, "Up") == 0;
    if (up_selected){
      current_menu = current_menu->menu;
    }

    list_menu_items(*current_menu, buff);
  }
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

  for(size_t i = 0; i < menu.menu_items; i++)
  {
    get_menu_item_str(menu.menu[i], i, buff);
    print(buff);

    //if(menu.menu[i].menu != nullptr) list_menu_items(menu.menu[i], buff);
  }
  strcpy(buff, "\n\r");
  print(buff);
}