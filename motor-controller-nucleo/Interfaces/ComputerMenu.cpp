#include "ComputerMenu.h"
#include "usbd_cdc_if.h"
#include <algorithm>
#include "settings_structs.h"
#include "ComputerInterface.h"

extern "C" {
  #include "tiny_printf.h"
}

#define my_sprintf sprintf

ComputerMenu::ComputerMenu(ComputerInterface *ci)
{
  using mc_param = MotorControllerParameter_t;
  auto default_param = mc_param::NO_ACTION;

  hall_setting_items[0] = MenuItem{"Up", "", default_param, &main_menu_items[1], 1};
  hall_setting_items[1] = MenuItem{"Hall Polatrity", "", mc_param::HALL_POLARITY, nullptr, 0};
  hall_setting_items[2] = MenuItem{"Hall Direction", "", mc_param::HALL_DIRECTION, nullptr, 0};
  hall_setting_items[3] = MenuItem{"Mechanical Offset", "", mc_param::HALL_MECH_OFFSET, nullptr, 0};
  hall_setting_items[4] = MenuItem{"Electrical Offset", "", mc_param::HALL_ELEC_OFFSET, nullptr, 0};

  limits_menu_items[0] = MenuItem{"Up", "", default_param, &main_menu, 1};
  limits_menu_items[1] = MenuItem{"Max Current", "", mc_param::CURRENT_LIMIT, nullptr, 0};
  limits_menu_items[2] = MenuItem{"Max Velocity", "", mc_param::VELOCITY_LIMIT, nullptr, 0};
  limits_menu_items[3] = MenuItem{"Max Acceleration", "", mc_param::ACCELERATION_LIMIT, nullptr, 0};

  motor_setting_items[0] = MenuItem{"Up", "", default_param, &main_menu, 1}; 
  motor_setting_items[1] = MenuItem{"Motor Type", "", mc_param::MOTOR_TYPE, nullptr, 0};
  motor_setting_items[2] = MenuItem{"Pole Pairs", "", mc_param::POLE_PAIRS_KV, nullptr, 0};
  motor_setting_items[3] = MenuItem{"Open Loop Settings", "", default_param, open_loop_items.data(), open_loop_items.size()};
  motor_setting_items[4] = MenuItem{"Hall Effect Settings", "", default_param, 
                                    hall_setting_items.data(), hall_setting_items.size()};
  motor_setting_items[5] = MenuItem{"Hall Effect Auto Setup", "", mc_param::HALL_AUTO_SETUP, nullptr, 0};

  open_loop_items[0] = MenuItem{"Up", "", default_param, &main_menu, 1};
  open_loop_items[1] = MenuItem{"Max Current", "", mc_param::OPEN_LOOP_MAX_I, nullptr, 0};
  open_loop_items[2] = MenuItem{"Max Voltage", "", mc_param::OPEN_LOOP_MAX_V, nullptr, 0};
  open_loop_items[3] = MenuItem{"Acceleration", "", mc_param::OPEN_LOOP_ACCELERATION, nullptr, 0};

  general_setting_items[0] = MenuItem{"Up", "", default_param, &main_menu, 1};
  general_setting_items[1] = MenuItem{"Motor Control Mode", "", mc_param::MOTOR_MODE, nullptr, 0};
  general_setting_items[2] = MenuItem{"Motor Direction", "", mc_param::MOTOR_DIRECTION, nullptr, 0};
  general_setting_items[3] = MenuItem{"Motor Controller CAN ID", "", default_param, nullptr, 0};
  general_setting_items[4] = MenuItem{"Throttle CAN ID", "", default_param, nullptr, 0};

  main_menu_items[0] = MenuItem{"Limits", "", default_param, limits_menu_items.data(), limits_menu_items.size()};
  main_menu_items[1] = MenuItem{"General Settings", "", default_param, general_setting_items.data(), general_setting_items.size()};
  main_menu_items[2] = MenuItem{"Motor Settings", "", default_param, motor_setting_items.data(), motor_setting_items.size()};
  main_menu_items[3] = MenuItem{"Save Settings", "Save settings to flash", mc_param::SAVE_SETTINGS, nullptr, 0};

  main_menu = MenuItem{"Main Menu", "", default_param, main_menu_items.data(), main_menu_items.size()};

  current_menu = &main_menu;
  leaf_item = nullptr;
  compInterface = ci;
}


void ComputerMenu::display_menu(int menu_num)
{
  // Make my buffer the same length as the USB buffer (defined in usbd_cdc_if.c)
  const uint8_t BUFFSIZE = 128; // same length as the USB buffer
  char buff[BUFFSIZE] = {0};

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
    // this case is the case if we are trying to change the setting's value
    if (leaf_item != nullptr)
    {
      display_menu = leaf_item;
    }
    // this is the case where the user selected a setting to change
    else if (current_menu->menu[menu_num].menu == nullptr)
    {
      leaf_item = &current_menu->menu[menu_num];
      display_menu = leaf_item;

      // make sure we don't set the item to the menu number
      menu_num = KEEP_MENU;
      //current menu remains unchanged
    }
    // otherwise they are selecting a submenu
    else 
    {
      leaf_item = nullptr;
      current_menu = &current_menu->menu[menu_num];

      // if the user selected up, skip the intermediate menu
      auto up_selected = strcmp(current_menu->name_str, "Up") == 0;
      if (up_selected){
        current_menu = current_menu->menu;
      }

      display_menu = current_menu;
    }
  }

  if (leaf_item != nullptr)
  {
    display_leaf_item(*leaf_item, menu_num, buff);
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

void ComputerMenu::display_leaf_item(const MenuItem& menu, int command, char *buff)
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


  bool write_setting = false;
  int32_t write_value = 0;
  if (command != KEEP_MENU)
  {
    write_setting = true;
    write_value = command;
  }
  
  sprintf(buff, 
          "%s\n\r", 
          compInterface->access_setting_value(buff, menu.param, write_setting, write_value));
  print(buff);
}