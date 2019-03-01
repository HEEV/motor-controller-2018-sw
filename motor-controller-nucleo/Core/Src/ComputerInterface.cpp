#include "ComputerInterface.h"
#include <stm32f3xx_hal.h>
// from USB_DEVICE/App directory
#include <usbd_cdc_if.h>

// define commonly used types
using int8_t = std::int8_t;
using uint8_t = std::uint8_t;
using int16_t = std::int16_t;
using uint16_t = std::uint16_t;
using int32_t = std::int32_t;
using uint32_t = std::uint32_t;

extern UART_HandleTypeDef huart2;

ComputerInterface::ComputerInterface(MotorControllerSettings_t *Settings_)
{
  //keep the address of the settings
  Settings = Settings_;
}

/** 
 * Send the current value of one of the motor controller settings to the host PC
 * The input to the function is the parameter that should be transmitted
 */
void ComputerInterface::transmit_setting(MotorControllerParameter_t param)
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

/**
 * Write a packet to the host PC using either the USB or USART interface
 */
void ComputerInterface::transmit_packet(const MotorControllerPacket_t &packet)
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
  switch (param)
  {
  // general motor settings
  case MotorParams::MOTOR_DIRECTION:
    if (!toSettings) { packet.u8 = static_cast<uint8_t>(settings.MotorDir); }
    else { settings.MotorDir = static_cast<MotorDirection_t>(packet.u8); }
    break;
  case MotorParams::MOTOR_MODE:
    if (!toSettings) { packet.u8 = static_cast<uint8_t>(settings.ControlMode); }
    else { settings.ControlMode = static_cast<ControlMode_t>(packet.u8); }
    break;
  case MotorParams::SETPOINT:
    if (!toSettings) { packet.i32 = settings.Setpoint; }
    else { settings.Setpoint = packet.i32; }
    break;
  // Torque, velocity, and acceleration limits
  case MotorParams::CURRENT_LIMIT:
    if (!toSettings) { packet.i16 = settings.CurrentLimit; }
    else { settings.CurrentLimit = packet.i16; }
    break;

  case MotorParams::VELOCITY_LIMIT :
    if (!toSettings) { packet.i32 = settings.VelocityLimit; }
    else { settings.VelocityLimit= packet.i32; }
    break;

  case MotorParams::ACCELERATION_LIMIT :
    if (!toSettings) { packet.i32 = settings.AccelerationLimit; }
    else { settings.AccelerationLimit = packet.i32; }
    break;

  // Motor type settings
  case MotorParams::MOTOR_TYPE:
    if (!toSettings) { packet.u8 = static_cast<uint8_t>(settings.MotorType); }
    else { settings.MotorType = static_cast<MotorType_t>(packet.u8); }
    break;
  case MotorParams::POLE_PAIRS_KV:
    if (!toSettings) { packet.u8 = settings.PolePairs_KV; }
    else { settings.PolePairs_KV = packet.u8; }
    break;

  // Hall effect settings
  case MotorParams::HALL_POLARITY:
    if (!toSettings) { packet.u8 = settings.HallMode.HallPolarity; }
    else { settings.HallMode.HallPolarity = (packet.u8 != 0); }
    break;
  case MotorParams::HALL_INTERPOLATE:
    if (!toSettings) { packet.u8 = settings.HallMode.HallInterpolate; }
    else { settings.HallMode.HallInterpolate = (packet.u8 != 0); }
    break;
  case MotorParams::HALL_DIRECTION:
    if (!toSettings) { packet.u8 = settings.HallMode.HallDirection; }
    else { settings.HallMode.HallDirection = (packet.u8 != 0); }
    break;
  case MotorParams::HALL_MECH_OFFSET:
    if (!toSettings) { packet.i16 = settings.HallMechOffset; }
    else { settings.HallMechOffset = packet.i16; }
    break;
  case MotorParams::HALL_ELEC_OFFSET:
    if (!toSettings) { packet.i16 = settings.HallElecOffset; }
    else { settings.HallElecOffset = packet.i16; }
    break;

  // open loop settings
  case MotorParams::OPEN_LOOP_MODE_ENABLED: /// startup in open loop mode
    if (!toSettings) { packet.u8 = settings.OpenStartup; }
    else { settings.OpenStartup = packet.u8; }
    break;

  case MotorParams::OPEN_LOOP_TRANSITION_VELOCITY: /// RPM to transition from open loop to closed loop mode
    if (!toSettings) { packet.u16 = settings.OpenTransistionVel; }
    else { settings.OpenTransistionVel = packet.u16; }
    break;

  case MotorParams::OPEN_LOOP_ACCELERATION: /// Acceleration in RPM/s
    if (!toSettings) { packet.u16 = settings.OpenAccel; }
    else { settings.OpenAccel = packet.u16; }
    break;

  case MotorParams::OPEN_LOOP_MAX_I: /// Max current in mili-Amps
    if (!toSettings) { packet.u16 = settings.OpenMaxI; }
    else { settings.OpenMaxI = packet.u16; }
    break;

  case MotorParams::OPEN_LOOP_MAX_V: /// Max voltage in Volts
    if (!toSettings) { packet.u16 = settings.OpenMaxV; }
    else { settings.OpenMaxV = packet.u16; }
    break;

  // unknown parameter, return early
  default:
    return;
  }
}
