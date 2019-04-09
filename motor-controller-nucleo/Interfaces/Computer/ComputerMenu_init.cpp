#include "ComputerMenu.h"
#include "settings_structs.h"
#include "ComputerInterface.h"

ComputerMenu::ComputerMenu(ComputerInterface *ci)
{
  using mc_param = MotorControllerParameter_t;
  auto default_param = mc_param::NO_ACTION;

  main_menu = MenuItem{"Main Menu", "", default_param, main_menu_items.data(), nullptr, main_menu_items.size()};
  main_menu_items[0] = MenuItem{"Limits", "", default_param, limits_menu_items.data(), &main_menu, limits_menu_items.size()};
  main_menu_items[1] = MenuItem{"General Settings", "", default_param, general_setting_items.data(), &main_menu, general_setting_items.size()};
  main_menu_items[2] = MenuItem{"Motor Settings", "", default_param, motor_setting_items.data(), &main_menu, motor_setting_items.size()};
  main_menu_items[3] = MenuItem{"Save Settings", "Save settings to flash\n\r 0 to exit, 1 to save current settings",
                          mc_param::SAVE_SETTINGS, nullptr, &main_menu, 0};

  // make references to the main menu items (to the sub menus)
  const auto limits_menu  = &main_menu_items[0];
  const auto general_menu = &main_menu_items[1];
  const auto motor_menu   = &main_menu_items[2];

  // setup the limits menu
  limits_menu_items[0] = MenuItem{"Up", "", default_param, &main_menu, &main_menu, 1};
  limits_menu_items[1] = MenuItem{"Max Current", "Maximum motor current in mA (only gets changed in 2mA incriments)", 
                            mc_param::CURRENT_LIMIT, nullptr, limits_menu, 0};
  limits_menu_items[2] = MenuItem{"Max Velocity", "Maximum motor RPM (only in velocity mode)", 
                            mc_param::VELOCITY_LIMIT, nullptr, limits_menu, 0};
  limits_menu_items[3] = MenuItem{"Max Acceleration", "Maximum motor acceleration (only in velocity mode)", 
                            mc_param::ACCELERATION_LIMIT, nullptr, limits_menu, 0};

  // setup the general settings menu
  general_setting_items[0] = MenuItem{"Up", "", default_param, &main_menu, &main_menu, 1};
  general_setting_items[1] = MenuItem{"Motor Control Mode", "0 for Torque, 1 for Velocity", mc_param::MOTOR_MODE, nullptr, general_menu, 0};
  general_setting_items[2] = MenuItem{"Motor Direction", "0 for Forward, 1 for Reverse", mc_param::MOTOR_DIRECTION, nullptr, general_menu, 0};
  general_setting_items[3] = MenuItem{"Motor Controller CAN ID", "Select a base ID for sending out and recieving data (requires a restart)", 
                              mc_param::CONTROLLER_CAN_ID, nullptr, general_menu, 0};
  general_setting_items[4] = MenuItem{"Throttle CAN ID", "Select an ID to listen to for setpoints (requires a restart)",
                              mc_param::THROTTLE_CAN_ID, nullptr, general_menu, 0};
  general_setting_items[5] = MenuItem{"Use Analog Input", "1 for CAN setpoint, 0 for Throttle input", mc_param::USE_ANALOG, nullptr, general_menu, 0};
  general_setting_items[6] = MenuItem{"Throttle Setup", "Put throttle at minimum, enter 1.\n\rThen put throttle at maximum and enter 1", 
                              mc_param::ANALOG_SETUP, nullptr, general_menu, 0};

  // setup the motor menu
  motor_setting_items[0] = MenuItem{"Up", "", default_param, &main_menu, &main_menu, 1}; 
  motor_setting_items[1] = MenuItem{"Motor Type", "0 for BLDC, 1 for Brushed", mc_param::MOTOR_TYPE, nullptr, motor_menu, 0};
  motor_setting_items[2] = MenuItem{"Pole Pairs", 
                             "If in BLDC mode, this is the number of pole pairs\n\rIf Brushed, then it is the motor constant",
                             mc_param::POLE_PAIRS_KV, nullptr, motor_menu, 0};
  motor_setting_items[3] = MenuItem{"PI constants", "", default_param, pi_menu_items.data(), motor_menu, pi_menu_items.size()};
  motor_setting_items[4] = MenuItem{"Open Loop Settings", "", default_param, open_loop_items.data(), motor_menu, open_loop_items.size()};
  motor_setting_items[5] = MenuItem{"Hall Effect Settings", "", default_param, hall_setting_items.data(), motor_menu, hall_setting_items.size()};
  motor_setting_items[6] = MenuItem{"Hall Effect Auto Setup", "", mc_param::HALL_AUTO_SETUP, nullptr, motor_menu, 0};

  //make references to the submenus
  const auto pi_menu        = &motor_setting_items[3];
  const auto open_loop_menu = &motor_setting_items[4];
  const auto hall_menu      = &motor_setting_items[5];

  // PI constants submenu
  pi_menu_items[0] = MenuItem{"Up", "", default_param, motor_menu, motor_menu, 1}; 
  pi_menu_items[1] = MenuItem{"Torque P constant", "Default: 256\n\r Range: [0, 32767]", 
                        mc_param::TORQUE_P, nullptr, pi_menu, 0};
  pi_menu_items[2] = MenuItem{"Torque I constant", "Default: 256\n\r Range: [0, 32767]", 
                        mc_param::TORQUE_I, nullptr, pi_menu, 0};
  pi_menu_items[3] = MenuItem{"Velocity P constant", "Default: 256\n\r Range: [0, 32767]", 
                        mc_param::VELOCITY_P, nullptr, pi_menu, 0};
  pi_menu_items[4] = MenuItem{"Velocity I constant", "Default: 256\n\r Range: [0, 32767]", 
                        mc_param::VELOCITY_I, nullptr, pi_menu, 0};
  pi_menu_items[5] = MenuItem{"Flux P constant", "Default: 256\n\r Range: [0, 32767]", 
                        mc_param::FLUX_P, nullptr, pi_menu, 0};
  pi_menu_items[6] = MenuItem{"Flux I constant", "Default: 256\n\r Range: [0, 32767]", 
                        mc_param::FLUX_I, nullptr, pi_menu, 0};

  open_loop_items[0] = MenuItem{"Up", "", default_param, motor_menu, motor_menu, 1};
  open_loop_items[1] = MenuItem{"Max Current", "", mc_param::OPEN_LOOP_MAX_I, nullptr, open_loop_menu, 0};
  open_loop_items[2] = MenuItem{"Max Voltage", "", mc_param::OPEN_LOOP_MAX_V, nullptr, open_loop_menu, 0};
  open_loop_items[3] = MenuItem{"Acceleration", "", mc_param::OPEN_LOOP_ACCELERATION, nullptr, open_loop_menu, 0};

  hall_setting_items[0] = MenuItem{"Up", "", default_param, motor_menu, motor_menu, 1};
  hall_setting_items[1] = MenuItem{"Hall Polarity", "0 for default polarity, 1 to invert polarity", mc_param::HALL_POLARITY, nullptr, hall_menu, 0};
  hall_setting_items[2] = MenuItem{"Hall Direction", "0 for default direction, 1 for reverse direction", mc_param::HALL_DIRECTION, nullptr, hall_menu, 0};
  hall_setting_items[3] = MenuItem{"Mechanical Offset", "Default: 0\n\r Range: [-32768, 32767]", 
                            mc_param::HALL_MECH_OFFSET, nullptr, hall_menu, 0};
  hall_setting_items[4] = MenuItem{"Electrical Offset", "Compensate for hall electrical offset\n\r Range: [-32768, 32767]", 
                            mc_param::HALL_ELEC_OFFSET, nullptr, hall_menu, 0};

  current_menu = &main_menu;
  compInterface = ci;
}