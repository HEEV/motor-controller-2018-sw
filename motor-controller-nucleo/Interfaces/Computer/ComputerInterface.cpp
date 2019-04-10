#include "ComputerInterface.h"
#include <stm32f3xx_hal.h>
#include "SettingsManager.h"
#include <Actions.h>

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

extern UART_HandleTypeDef huart2;
extern SettingsManager* hsettings_manager;

// wrapper for a class function
void computerInterface_update_buffer(void* piface, const uint8_t* buff, uint32_t len)
{
  auto hcomp_iface = reinterpret_cast<ComputerInterface*>(piface);
  hcomp_iface->add_to_buffer(buff, len);
}

ComputerInterface::ComputerInterface(MotorControllerValues_t *Settings_, TMC4671Interface* mc_handle) :
  menu(this)
{
  //keep the address of the settings
  Settings = Settings_;
  htmc4671 = mc_handle;
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
  HAL_Delay(2);
}

const char* ComputerInterface::access_setting_value(char *buff, MotorControllerParameter_t param, bool write, std::int32_t value)
{
  // so I don't have to type MotorControllerParameter_t every time
  using MotorParams = MotorControllerParameter_t;

  auto& tmc4671 = Settings->tmc4671;
  auto& gen_settings = Settings->General;

  // make a lambda function to convert a bit slice into a string
  auto bit2Str = [](uint8_t bit) { return bit ? "On" : "Off";};

  switch (param)
  {
  // general motor tmc4671
  case MotorParams::MOTOR_DIRECTION:
    if (write)
    {
      tmc4671.MotorDir = (MotorDirection_t) value;
      htmc4671->set_direction(tmc4671.MotorDir);
    }
    switch(tmc4671.MotorDir) {
      default:
      case MotorDirection_t::FORWARD:
        return "Forward";
      case MotorDirection_t::REVERSE:
        return "Reverse";
    } 
    break;
    
  case MotorParams::MOTOR_MODE:
    if (write)
    {
      tmc4671.ControlMode = (ControlMode_t) value;
      htmc4671->set_control_mode(tmc4671.ControlMode);
    }
    switch(tmc4671.ControlMode) {
      default:
      case ControlMode_t::VELOCITY:
        return "Velocity";
      case ControlMode_t::TORQUE:
        return "Torque";
      case ControlMode_t::OPEN_LOOP:
        return "Open Loop";
    } 
    break;

  case MotorParams::SETPOINT:
  {
    if (write) 
    { 
      tmc4671.Setpoint = (std::uint32_t) value; 
      htmc4671->set_setpoint(tmc4671.Setpoint);
    }
    auto units = [](ControlMode_t cm) {
      return (cm == ControlMode_t::TORQUE) ? "mA" : "RPM";
    };
    my_sprintf(buff, "%lu %s", tmc4671.Setpoint, units(tmc4671.ControlMode));
    return buff; 
  }

  // CAN Settings
  case MotorParams::CONTROLLER_CAN_ID:
  {
    if (write)
    {
      gen_settings.ControllerCanId = (std::uint16_t) value;
      // TODO notify that this requires a save and restart to take effect
    } 
    my_sprintf(buff, "%u (0x%X)", 
      gen_settings.ControllerCanId,
      gen_settings.ControllerCanId);
    return buff;
  }

  case MotorParams::THROTTLE_CAN_ID:
  {
    if (write)
    {
      gen_settings.ThrottleCanId = (std::uint16_t) value;
      // TODO notify that this requires a save and restart to take effect
    } 
    my_sprintf(buff, "%u (0x%X)", 
      gen_settings.ThrottleCanId, 
      gen_settings.ThrottleCanId);
    return buff;
  }

  // Torque, velocity, and acceleration limits
  case MotorParams::CURRENT_LIMIT:
    if (write) 
    { 
      tmc4671.CurrentLimit = ((std::uint32_t) value > GLOBAL_MAX_CURRENT) ?
          GLOBAL_MAX_CURRENT : (std::uint32_t) value; 
      htmc4671->change_settings(&tmc4671);
    }
    my_sprintf(buff, "%lu mA", tmc4671.CurrentLimit);
    return buff; 

  case MotorParams::VELOCITY_LIMIT :
    if (write) 
    { 
      tmc4671.VelocityLimit = (std::uint32_t) value; 
      htmc4671->change_settings(&tmc4671);
    }
    my_sprintf(buff, "%ld RPM", tmc4671.VelocityLimit);
    return buff; 

  case MotorParams::ACCELERATION_LIMIT :
    if (write) 
    { 
      tmc4671.AccelerationLimit = (std::uint32_t) value; 
      htmc4671->change_settings(&tmc4671);
    }
    my_sprintf(buff, "%ld RPM/Sec", tmc4671.AccelerationLimit);
    return buff; 

  // Motor type tmc4671
  case MotorParams::MOTOR_TYPE:
    if (write)
    {
      switch(value)
      {
        default:
        case 0:
          tmc4671.MotorType = MotorType_t::BLDC_MOTOR;
          break;
        case 1:
          tmc4671.MotorType = MotorType_t::BRUSHED_MOTOR;
          break;
      }
      htmc4671->change_settings(&tmc4671);
    }
    switch(tmc4671.MotorType) {
      default:
      case MotorType_t::BLDC_MOTOR :
        return "BLDC Motor";
      case MotorType_t::BRUSHED_MOTOR :
        return "Brushed Motor";
    } 
    break;

  case MotorParams::POLE_PAIRS_KV:
  {
    if (write) 
    { 
      tmc4671.PolePairs_KV = (std::uint8_t) value; 
      htmc4671->change_settings(&tmc4671);
    }
    auto units = [](MotorType_t mt) {
      return (mt == MotorType_t::BLDC_MOTOR) ? "Pole Pairs" : "RPM/Volt";
    };
    my_sprintf(buff, "%d %s", tmc4671.PolePairs_KV, units(tmc4671.MotorType));
    return buff; 
  }

  // Hall effect tmc4671
  case MotorParams::HALL_POLARITY:
    if (write) 
    { 
      tmc4671.HallMode.HallPolarity = static_cast<std::uint8_t>(value == 1); 
      htmc4671->change_settings(&tmc4671);
    }
    return bit2Str( tmc4671.HallMode.HallPolarity );

  case MotorParams::HALL_INTERPOLATE:
    if (write) 
    { 
      tmc4671.HallMode.HallInterpolate = static_cast<std::uint8_t>(value == 1); 
      htmc4671->change_settings(&tmc4671);
    }
    return bit2Str( tmc4671.HallMode.HallInterpolate );

  case MotorParams::HALL_DIRECTION:
    if (write) 
    { 
      tmc4671.HallMode.HallDirection = static_cast<std::uint8_t>(value == 1); 
      htmc4671->change_settings(&tmc4671);
    }
    return bit2Str( tmc4671.HallMode.HallDirection );

  case MotorParams::HALL_MECH_OFFSET:
    if (write) 
    { 
      tmc4671.HallMechOffset = (std::int16_t) value; 
      htmc4671->change_settings(&tmc4671);
    }
    my_sprintf(buff, "%d", tmc4671.HallMechOffset);
    return buff; 

  case MotorParams::HALL_ELEC_OFFSET:
    if (write) 
    { 
      tmc4671.HallElecOffset = (std::int16_t) value; 
      htmc4671->change_settings(&tmc4671);
    }
    my_sprintf(buff, "%d", tmc4671.HallElecOffset);
    return buff; 

  // PI settings
  case MotorParams::FLUX_P:
  {
    if(write)
    {
      tmc4671.FluxP = (std::uint16_t) value;
      htmc4671->change_settings(&tmc4671);
    }
    my_sprintf(buff, "%u", tmc4671.FluxP);
    return buff;
  }
  case MotorParams::FLUX_I:

  {
    if(write)
    {
      tmc4671.FluxI = (std::uint16_t) value;
      htmc4671->change_settings(&tmc4671);
    }
    my_sprintf(buff, "%u", tmc4671.FluxI);
    return buff;
  }

  case MotorParams::TORQUE_P:
  {
    if(write)
    {
      tmc4671.TorqueP = (std::uint16_t) value;
      htmc4671->change_settings(&tmc4671);
    }
    my_sprintf(buff, "%u", tmc4671.TorqueP);
    return buff;
  }
  
  case MotorParams::TORQUE_I:
  {
    if(write)
    {
      tmc4671.TorqueI = (std::uint16_t) value;
      htmc4671->change_settings(&tmc4671);
    }
    my_sprintf(buff, "%u", tmc4671.TorqueI);
    return buff;
  }

  case MotorParams::VELOCITY_P:
  {
    if(write)
    {
      tmc4671.VelocityP = (std::uint16_t) value;
      htmc4671->change_settings(&tmc4671);
    }
    my_sprintf(buff, "%u", tmc4671.VelocityP);
    return buff;
  }

  case MotorParams::VELOCITY_I:
  {
    if(write)
    {
      tmc4671.VelocityI = (std::uint16_t) value;
      htmc4671->change_settings(&tmc4671);
    }
    my_sprintf(buff, "%u", tmc4671.VelocityI);
    return buff;
  }

  // open loop tmc4671
  case MotorParams::OPEN_LOOP_ACCELERATION: /// Acceleration in RPM/s
    if (write) 
    { 
      tmc4671.OpenAccel = (std::uint16_t) value; 
      htmc4671->change_settings(&tmc4671);
    }
    my_sprintf(buff, "%u RPM/Sec", tmc4671.OpenAccel);
    return buff; 

  case MotorParams::OPEN_LOOP_MAX_I: /// Max current in mili-Amps
    if (write) 
    { 
      tmc4671.OpenMaxI = (std::uint16_t) value; 
      htmc4671->change_settings(&tmc4671);
    }
    my_sprintf(buff, "%lu mA", tmc4671.OpenMaxI);
    return buff; 

  case MotorParams::OPEN_LOOP_MAX_V: /// Max voltage in Volts
    if (write) 
    { 
      tmc4671.OpenMaxV = (std::uint16_t) value; 
      htmc4671->change_settings(&tmc4671);
    }
    my_sprintf(buff, "%u V", tmc4671.OpenMaxV);
    return buff; 
  
  case MotorParams::USE_ANALOG :
  {
    if (write) 
    { 
      gen_settings.bool_settings.useAnalog = static_cast<std::uint8_t>(value == 1); 
    }
    return bit2Str( gen_settings.bool_settings.useAnalog );
  }

  case MotorParams::ANALOG_SETUP:
  {
    auto& throttle_range = gen_settings.ThrottleRange;
    auto_throttle_setup(value);
    my_sprintf(buff, "Min: %ld, Max: %ld", throttle_range.min, throttle_range.max);
    return buff;
  }

  case MotorParams::SAVE_SETTINGS :
    if (write) 
    { 
      hsettings_manager->save_settings();
      return "saved";
    }
    return "";
    break;

  case MotorParams::HALL_AUTO_SETUP:
    if (write)
    {
      // call the hall auto setup function
    }
    return "";

  case MotorParams::LIVE_VALUES:
    return "";

  // unknown parameter, return early
  default:
    return "Unknown Parameter";
  }
}