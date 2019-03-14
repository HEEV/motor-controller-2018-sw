#include "ComputerMenu.h"
#include "usbd_cdc_if.h"
#include <algorithm>

extern "C" {
  #include "tiny_printf.h"
}

#define my_sprintf sprintf

ComputerMenu::ComputerMenu()
{
  hall_setting_items[0] = MenuItem{"Up", "", nullptr, 0};
  hall_setting_items[1] = MenuItem{"Hall Polatrity", "", nullptr, 0};
  hall_setting_items[2] = MenuItem{"Hall Direction", "", nullptr, 0};
  hall_setting_items[3] = MenuItem{"Mechanical Offset", "", nullptr, 0};
  hall_setting_items[4] = MenuItem{"Electrical Offset", "", nullptr, 0};

  limits_menu_items[0] = MenuItem{"Up", "", nullptr, 0};
  limits_menu_items[1] = MenuItem{"Max Current", "", nullptr, 0};
  limits_menu_items[2] = MenuItem{"Max Voltage", "", nullptr, 0};
  limits_menu_items[3] = MenuItem{"Max Acceleration", "", nullptr, 0};

  motor_setting_items[0] = MenuItem{"Up", "", nullptr, 0};
  motor_setting_items[1] = MenuItem{"Motor Type", "", nullptr, 0};
  motor_setting_items[2] = MenuItem{"Pole Pairs", "", nullptr, 0};
  motor_setting_items[3] = MenuItem{"Hall Effect Settings", "", hall_setting_items.data(), hall_setting_items.size()};
  motor_setting_items[4] = MenuItem{"Hall Effect Auto Setup", "", nullptr, 0};

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
  if (menu_num == -1){
    list_menu_items(*current_menu, buff);
  }
  else {
    list_menu_items(*current_menu, buff);
    current_menu = &current_menu->menu[menu_num];
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
  buff[21] = '\0';
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