#include "ComputerInterface.h"
#include <stm32f3xx_hal.h>

// for tfp_sprintf()
#define PRINTF_LONG_SUPPORT

#ifdef __cplusplus
extern "C" {
  #include "tiny_printf.h"
}
#else
  #include "tiny_printf.h"
#endif

#define my_sprintf sprintf

// from USB_DEVICE/App directory
#include <usbd_cdc_if.h>
#include <algorithm>
#include "TMC4671Interface.h"
#include "settings_structs.h"

// define commonly used types
using int8_t = std::int8_t;
using uint8_t = std::uint8_t;
using int16_t = std::int16_t;
using uint16_t = std::uint16_t;
using int32_t = std::int32_t;
using uint32_t = std::uint32_t;

// wrapper for a class function
void computerInterface_update_buffer(void* piface, const uint8_t* buff, uint32_t len)
{
  auto hcomp_iface = reinterpret_cast<ComputerInterface*>(piface);
  hcomp_iface->add_to_buffer(buff, len);
}

ComputerInterface::ComputerInterface(MotorControllerValues_t *Settings_, TMC4671Interface* mc_handle, SettingsManager* manager_) :
  menu(this)
{
  //keep the address of the settings
  Settings = Settings_;
  htmc4671 = mc_handle;
  hsettings_manager = manager_;
}

void ComputerInterface::add_to_buffer(const std::uint8_t* buff, std::uint32_t len)
{
  command_parser.add_to_buffer(buff, len); 
}

menu_cmd_t ComputerInterface::parse_command() 
{
  auto command = command_parser.parse_buffer();

  const uint8_t BUFFSIZE = 125; // same length as the USB buffer
  char buff[BUFFSIZE] = {0};

  // print the command buffer
  my_sprintf(buff,"> %s", command_parser.data());
  println(buff);

  return command;
}

void ComputerInterface::display_settings()
{
  static menu_cmd_t command = {menu_commands_t::KEEP_MENU, 0};

  menu.display_menu(command);
  command = parse_command();
  
}

void ComputerInterface::println(char *buff)
{
  strcat(buff, "\n\r");
  CDC_Transmit_FS((uint8_t*) buff, strlen(buff)+1);
  HAL_Delay(1);
}

const char* ComputerInterface::access_setting_value(char *buff, MotorControllerParameter_t param, bool write, std::int32_t value)
{
  if(write)
  {
    hsettings_manager->write_setting(param, value);
  }
  return hsettings_manager->get_setting_as_string(buff, param);
}

void ComputerInterface::display_can_ids(char* buff)
{
  //get the base can id
  auto base_id = Settings->General.ControllerCanId;
  sprintf(buff, 
          "Base ID: %-5u (0x%03X)\n\r"
          "Transmitting ID's",
          base_id, base_id);
  println(buff);

  sprintf(buff, 
          "Temperature:            %-5u (0x%03X)\n\r"
          "Motor Current (Amps):   %-5u (0x%03X)",
          base_id+1, base_id+1,
          base_id+2, base_id+2);
  println(buff);

  sprintf(buff, 
          "Motor RPM:              %-5u (0x%03X)\n\r"
          "Battery Current (Amps): %-5u (0x%03X)\n\r"
          "Battery Voltage:        %-5u (0x%03X)\n\r",
          base_id+3, base_id+3,
          base_id+4, base_id+4,
          base_id+5, base_id+5);
  println(buff);

  sprintf(buff, 
          "Recieving ID's\n\r"
          "Direction ID:    %-5u (0x%03X)\n\r"
          "Control Mode ID: %-5u (0x%03X)\n\r",
          base_id+6, base_id+6,
          base_id+7, base_id+7);
  println(buff);

  sprintf(buff,
          "Max Current ID:  %-5u (0x%03X)\n\r"
          "Max RPM ID:      %-5u (0x%03X)\n\r"
          "Max Accel ID:    %-5u (0x%03X)\n\r",
          base_id+8, base_id+8,
          base_id+9, base_id+9,
          base_id+10, base_id+10);
  println(buff);
}