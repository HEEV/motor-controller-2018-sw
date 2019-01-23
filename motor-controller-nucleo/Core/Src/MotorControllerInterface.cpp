#include "MotorControllerInterface.h"
#include <stm32f3xx_hal.h>
// from USB_DEVICE/App directory
#include <usbd_cdc_if.h>
#include <TMC4671.h>

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

    case MotorParams::MOTOR_TYPE:
      packet.rw_address = static_cast<uint8_t>(param);
      packet.data.u8 = Settings->MotorType;
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

void MotorControllerInterface::init_tmc4671(uint8_t motor_type, uint16_t pole_pairs) 
{
  // setup the default register values and write them to the tmc4671
  for(uint8_t i = 0; i < 0x7D; ++i)
  {
    tmc4671_writeInt(TMC_DEFAULT_MOTOR, i, tmc4671Registers[i]);
  }

  // now update the motor type and pole pairs
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, (motor_type << 16) | (pole_pairs) );

  // stuff to setup hall effect sensors here (default config good for now)
  // turn on polarity flipping, interpolation, and reverse direction
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_MODE, 0x00001101);
  0x2AAA0000, //0x34: 
  0x80005555, //0x35: 
  0xD555AAAA, //0x36: 
  0x04000000, //0x37: 
  0x00002AAA, //0x38: 
  // addresses 0x33 - 0x3A
  
}