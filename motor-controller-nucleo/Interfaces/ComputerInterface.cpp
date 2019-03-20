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
#include <cctype>
#include "settings_structs.h"

// define commonly used types
using int8_t = std::int8_t;
using uint8_t = std::uint8_t;
using int16_t = std::int16_t;
using uint16_t = std::uint16_t;
using int32_t = std::int32_t;
using uint32_t = std::uint32_t;

extern UART_HandleTypeDef huart2;

// wrapper for a class function
void computerInterface_update_buffer(void* piface, const uint8_t* buff, uint32_t len)
{
  auto hcomp_iface = reinterpret_cast<ComputerInterface*>(piface);
  hcomp_iface->add_to_buffer(buff, len);
}

ComputerInterface::ComputerInterface(MotorControllerSettings_t *Settings_) :
  menu(ComputerInterface::access_setting_value)
{
  //keep the address of the settings
  Settings = Settings_;
}

/** 
 * Send the current value of one of the motor controller settings to the host PC
 * The input to the function is the parameter that should be transmitted
 */
void ComputerInterface::transmit_setting(MotorControllerParameter_t param) const
{
  MotorControllerPacket_t packet;

  //put the setting we want to copy into the packet
  packet.rw_address = static_cast<uint8_t>(param);

  // copy the setting from the class settings struct to the packet
  // setting we want to copy is stored in the packet.
  copy_setting(packet, *Settings, false);

  // send the packet to the host computer
  transmit_packet(packet);
}

/**
 * Change one of the motor controller settings based on a packet (assumed to have
 * been recieved from the host computer).
 */
void ComputerInterface::change_setting(MotorControllerPacket_t &packet)
{
  //copy the setting from the packet into the class settings struct
  copy_setting(packet, *Settings, true);

  //TODO update the TMC4671 hardware
}

/**
 * This function is meant to be called from the interrupt service routine for
 * the serial device in stm32f3xx_it.c.
 */
void ComputerInterface::recieve_packet(MotorControllerPacket_t &packet)
{
}

void ComputerInterface::add_to_buffer(const std::uint8_t* buff, std::uint32_t len)
{
  // copy the characters from the buffer into the array
  char* cpyPtr = &command_buff[command_len];

  if (cpyPtr == &command_buff.back())
  {
    // always put one new character from the input buffer into our
    // buffer
    cpyPtr = cpyPtr-1;
    command_len--;
  }

  while ( len > 0 && cpyPtr < &command_buff.back() )
  {
    // check for numeric character, enter, '-', or ESC.
    if( isdigit(*buff) )
    {
      // copy the char from the input buffer
      *cpyPtr++ = *buff;
      command_len++;
    }
    // copy in '-' if it is the first character
    else if( *buff == '-' && cpyPtr == command_buff.data())
    {
      // copy the char from the input buffer
      *cpyPtr++ = *buff;
      command_len++;
    }
    // check for control sequence
    else if (*buff== '\n' || *buff == '\r' || *buff == '\x1b')
    {
      // copy the char from the input buffer
      *cpyPtr++ = *buff;
      command_len++;

      // exit the loop
      break;
    }
    // check for backspace or delete
    else if(*buff == '\b' || *buff == '\x7F')
    {
      // if we haven't backspaced to the beginning of the buffer, decriment the parsed 
      // command
      if (command_len > 0)
      {
        cpyPtr--;
        command_len--;
      }
    }

    // go to the next spot in the buffer 
    buff++;
    len--;
  }

  // null terminate the string thus far
  *cpyPtr = '\0';

  // make sure we null terminate
  command_buff.back() = '\0';
}

int ComputerInterface::parse_command() 
{
  int menu_num = -1;

  // loop through the command string and make sure the characters are
  // numeric or accepted commands
  char *cmd = command_buff.data();     
  int cmd_len = 0;
  
  while (*cmd != '\0')
  {
    // check for enter or ESC
    if(*cmd == '\n' || *cmd == '\r' || *cmd == '\x1B')
    {
      // escape character
      if(*cmd == '\x1B')
      {
        menu_num = -2;
      }
      else 
      {
        // grab the integer from the buffer
        if(sscanf(command_buff.data(), "%d", &menu_num) !=1 )
          menu_num = -1;
      }

      // clear the buffer
      command_len = 0;
      command_buff.fill('\0');

      // done with the loop
      break;
    }

    // move on to the next command character
    cmd++;
    cmd_len++;
  }

  const uint8_t BUFFSIZE = 125; // same length as the USB buffer
  char buff[BUFFSIZE] = {0};

  // print the command buffer
  HAL_Delay(1);
  my_sprintf(buff,
  "%d\n\r"
  "%s\n\r",
  menu_num,
  command_buff.data());
  CDC_Transmit_FS((uint8_t*) buff, strlen(buff)+1);

  return menu_num;
}

void ComputerInterface::display_settings()
{
  static int command = -1;

  menu.display_menu(command);
  command = parse_command();
  
  /*
  // Make my buffer the same length as the USB buffer (defined in usbd_cdc_if.c)
  const uint8_t BUFFSIZE = 128; // same length as the USB buffer
  char buff[BUFFSIZE] = {0};
  
  char enum_name[10];
  switch(Settings->MotorDir)
  {
    default:
    case MotorDirection_t::FORWARD:
      strcpy(enum_name, "Forward");
      break;
    case MotorDirection_t::REVERSE:
      strcpy(enum_name, "Reverse");
      break;
  } 
  my_sprintf(buff, 
  "\fMotor Settings\n"
  "Motor Direction: %s\n", enum_name);
  CDC_Transmit_FS((uint8_t *) buff, strlen(buff)+1);

  switch(Settings->ControlMode)
  {
    default:
    case ControlMode_t::VELOCITY:
      strcpy(enum_name, "Velocity");
      break;
    case ControlMode_t::TORQUE:
      strcpy(enum_name, "Torque");
      break;
    case ControlMode_t::OPEN_LOOP:
      strcpy(enum_name, "Open Loop");
      break;
  } 

  #warning("Should probably add units here")
  my_sprintf(buff, 
  "Control Mode: %s\n"
  "Setpoint: %ld\n" // I should add units here
  "Current Limit: %d mA\n",
  enum_name, Settings->Setpoint, Settings->CurrentLimit);
  HAL_Delay(1);
  CDC_Transmit_FS((uint8_t *) buff, strlen(buff)+1);

  my_sprintf(buff, 
  "Velocity Limit: %d RPM\n"
  "Acceleration Limit: %lu RPM/sec\n",
  Settings->CurrentLimit, Settings->AccelerationLimit);
  HAL_Delay(1);
  CDC_Transmit_FS((uint8_t *) buff, strlen(buff)+1);
  
  // make a lambda function to convert a bit slice into a string
  auto bit2Str = [](uint8_t bit) { return bit ? "On" : "Off";};

  // Print the hall effect settings
  my_sprintf(buff, 
  "Hall effect sensor settings:"
  "Invert Hall Polarity: %s\n"
  "Interpolate Hall Readings: %s\n"
  "Reverse Hall Direction: %s\n",
  bit2Str(Settings->HallMode.HallPolarity), 
  bit2Str(Settings->HallMode.HallInterpolate),
  bit2Str(Settings->HallMode.HallDirection)); 
  HAL_Delay(1);
  CDC_Transmit_FS((uint8_t *) buff, strlen(buff)+1);

  my_sprintf(buff, 
  "Hall Mechanical Offset: %d\n"
  "Hall Electrical Offset: %d\n",
  Settings->HallMechOffset, Settings->HallElecOffset);
  HAL_Delay(1);
  CDC_Transmit_FS((uint8_t *) buff, strlen(buff)+1);

  // print the command buffer
  HAL_Delay(1);
  my_sprintf(buff,
  "Command Length: %d\n"
  "%s\n",
  command_len,
  command_buff.data());
  CDC_Transmit_FS((uint8_t*) buff, strlen(buff)+1);
  parse_command();
  */

  /*
  // Print the open loop settings
  my_sprintf(buff, 
  "Open Loop Settings\n"
  "Open Loop Startup: %s\n",
  bit2Str(Settings->OpenStartup));
  CDC_Transmit_FS((uint8_t *) buff, strlen(buff)+1);

  my_sprintf(buff, 
  "Open Loop Transistion: %u RPM\n"
  "Open Loop Acceleration: %u RPM/sec\n"
  "Open Loop Velocity: %u RPM\n",
  Settings->OpenTransistionVel,
  Settings->OpenAccel,
  Settings->OpenVel);
  CDC_Transmit_FS((uint8_t *) buff, strlen(buff)+1);

  my_sprintf(buff, 
  "Open Loop Max Current: %lu mA\n"
  "Open Loop Max Voltage: %u V\n",
  Settings->OpenMaxI,
  Settings->OpenMaxV);
  CDC_Transmit_FS((uint8_t *) buff, strlen(buff)+1);
  */
}

/**
 * Write a packet to the host PC using either the USB or USART interface
 */
void ComputerInterface::transmit_packet(const MotorControllerPacket_t &packet) const
{
  const uint8_t *packetPtr = reinterpret_cast<const uint8_t *>(&packet);
  const uint16_t packetSz = static_cast<uint16_t>(sizeof(MotorControllerPacket_t));
#if PC_INTERFACE == UART
  HAL_UART_Transmit(&huart2, (uint8_t *)packetPtr, packetSz, 50);
#elif PC_INTERFACE == USB
  CDC_Transmit_FS(packetPtr, packetSz);
#endif
}

void ComputerInterface::copy_setting(MotorControllerPacket_t &packet, MotorControllerSettings_t &settings, bool toSettings)
{
  // so I don't have to type MotorControllerParameter_t every time
  using MotorParams = MotorControllerParameter_t;

  auto param = static_cast<MotorParams>(packet.rw_address);
  auto& tmc4671 = settings.tmc4671; 
  
  switch (param)
  {
  // general motor tmc4671
  case MotorParams::MOTOR_DIRECTION:
    if (!toSettings) { packet.u8 = static_cast<uint8_t>(tmc4671.MotorDir); }
    else { tmc4671.MotorDir = static_cast<MotorDirection_t>(packet.u8); }
    break;
  case MotorParams::MOTOR_MODE:
    if (!toSettings) { packet.u8 = static_cast<uint8_t>(tmc4671.ControlMode); }
    else { tmc4671.ControlMode = static_cast<ControlMode_t>(packet.u8); }
    break;
  case MotorParams::SETPOINT:
    if (!toSettings) { packet.i32 = tmc4671.Setpoint; }
    else { tmc4671.Setpoint = packet.i32; }
    break;
  // Torque, velocity, and acceleration limits
  case MotorParams::CURRENT_LIMIT:
    if (!toSettings) { packet.i16 = tmc4671.CurrentLimit; }
    else { tmc4671.CurrentLimit = packet.i16; }
    break;

  case MotorParams::VELOCITY_LIMIT :
    if (!toSettings) { packet.i32 = tmc4671.VelocityLimit; }
    else { tmc4671.VelocityLimit= packet.i32; }
    break;

  case MotorParams::ACCELERATION_LIMIT :
    if (!toSettings) { packet.i32 = tmc4671.AccelerationLimit; }
    else { tmc4671.AccelerationLimit = packet.i32; }
    break;

  // Motor type tmc4671
  case MotorParams::MOTOR_TYPE:
    if (!toSettings) { packet.u8 = static_cast<uint8_t>(tmc4671.MotorType); }
    else { tmc4671.MotorType = static_cast<MotorType_t>(packet.u8); }
    break;
  case MotorParams::POLE_PAIRS_KV:
    if (!toSettings) { packet.u8 = tmc4671.PolePairs_KV; }
    else { tmc4671.PolePairs_KV = packet.u8; }
    break;

  // Hall effect tmc4671
  case MotorParams::HALL_POLARITY:
    if (!toSettings) { packet.u8 = tmc4671.HallMode.HallPolarity; }
    else { tmc4671.HallMode.HallPolarity = (packet.u8 != 0); }
    break;
  case MotorParams::HALL_INTERPOLATE:
    if (!toSettings) { packet.u8 = tmc4671.HallMode.HallInterpolate; }
    else { tmc4671.HallMode.HallInterpolate = (packet.u8 != 0); }
    break;
  case MotorParams::HALL_DIRECTION:
    if (!toSettings) { packet.u8 = tmc4671.HallMode.HallDirection; }
    else { tmc4671.HallMode.HallDirection = (packet.u8 != 0); }
    break;
  case MotorParams::HALL_MECH_OFFSET:
    if (!toSettings) { packet.i16 = tmc4671.HallMechOffset; }
    else { tmc4671.HallMechOffset = packet.i16; }
    break;
  case MotorParams::HALL_ELEC_OFFSET:
    if (!toSettings) { packet.i16 = tmc4671.HallElecOffset; }
    else { tmc4671.HallElecOffset = packet.i16; }
    break;

  // open loop tmc4671
  case MotorParams::OPEN_LOOP_ACCELERATION: /// Acceleration in RPM/s
    if (!toSettings) { packet.u16 = tmc4671.OpenAccel; }
    else { tmc4671.OpenAccel = packet.u16; }
    break;

  case MotorParams::OPEN_LOOP_MAX_I: /// Max current in mili-Amps
    if (!toSettings) { packet.u16 = tmc4671.OpenMaxI; }
    else { tmc4671.OpenMaxI = packet.u16; }
    break;

  case MotorParams::OPEN_LOOP_MAX_V: /// Max voltage in Volts
    if (!toSettings) { packet.u16 = tmc4671.OpenMaxV; }
    else { tmc4671.OpenMaxV = packet.u16; }
    break;

  // unknown parameter, return early
  default:
    return;
  }
}
