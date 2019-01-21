#include "MotorControllerInterface.h"
#include <stm32f3xx_hal.h>
// from USB_DEVICE/App directory
#include <usbd_cdc_if.h>

// define commonly used types
using int8_t   = std::int8_t;
using uint8_t  = std::uint8_t;
using int16_t  = std::int16_t;
using uint16_t = std::uint16_t;
using int32_t  = std::int32_t;
using uint32_t = std::uint32_t;


MotorControllerInterface::MotorControllerInterface(MotorControllerSettings_t &Settings_)
{
  //keep the address of the settings
  Settings = &Settings_;
  // load the settings into hardware from the settings struct
  /*
  Settings->MotorDir;
  Settings->OpenStartup;
  Settings->OpenTransistionVel;
  Settings->OpenAccel;
  Settings->OpenVel;
  Settings->OpenMaxI;
  Settings->OpenMaxV;
  */
}

/** 
 * Send the current value of one of the motor controller settings to the host PC
 * The input to the function is the parameter that should be transmitted
 */
void MotorControllerInterface::transmit_setting(MotorControllerParameter_t param)
{
  // so I don't have to type MotorControllerParameter_t every time
  using MotorParams = MotorControllerParameter_t;

  MotorControllerPacket_t packet;

  switch(param) {
    // general motor settings
    case MotorParams::MOTOR_DIRECTION: 
      packet.rw_address = static_cast<uint8_t>(param);
      packet.data.u8 = Settings->MotorDir;
    break;

    // open loop settings
    case MotorParams::OPEN_LOOP_MODE_ENABLED:         /// startup in open loop mode 
      packet.rw_address = static_cast<uint8_t>(param);
      packet.data.u8 = Settings->OpenStartup;
    break;

    case MotorParams::OPEN_LOOP_TRANSITION_VELOCITY:  /// RPM to transition from open loop to closed loop mode
      packet.rw_address = static_cast<uint8_t>(param);
      packet.data.u16 = Settings->OpenTransistionVel;
    break;
    
    case MotorParams::OPEN_LOOP_ACCELERATION:         /// Acceleration in RPM/s
      packet.rw_address = static_cast<uint8_t>(param);
      packet.data.u16 = Settings->OpenAccel;
    break;

    case MotorParams::OPEN_LOOP_VELOCITY:             /// Velocity in RPM
      packet.rw_address = static_cast<uint8_t>(param);
      packet.data.u16 = Settings->OpenVel;
    break;
    
    case MotorParams::OPEN_LOOP_MAX_I:                /// Max current in mili-Amps
      packet.rw_address = static_cast<uint8_t>(param);
      packet.data.u16 = Settings->OpenMaxI;
    break;

    case MotorParams::OPEN_LOOP_MAX_V:                /// Max voltage in Volts 
      packet.rw_address = static_cast<uint8_t>(param);
      packet.data.u16 = Settings->OpenMaxV;
    break;

    // unknown parameter, return early 
    default:
      return;
  }

  // send the packet to the host computer
  transmit_packet(packet);
}

/**
 * Change one of the motor controller settings based on a packet (assumed to have
 * been recieved from the host computer).
 */
void MotorControllerInterface::change_setting(MotorControllerPacket_t setting)
{

}

/**
 * This function is meant to be called from the interrupt service routine for
 * the serial device in stm32f3xx_it.c.
 */
void MotorControllerInterface::recieve_packet(MotorControllerPacket_t &packet)
{

}

void MotorControllerInterface::transmit_packet(const MotorControllerPacket_t &packet)
{

}