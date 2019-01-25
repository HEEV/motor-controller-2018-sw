#include "MotorControllerInterface.h"
#include <stm32f3xx_hal.h>
// from USB_DEVICE/App directory
#include <usbd_cdc_if.h>
#include <ic/TMC4671/TMC4671.h>

// define commonly used types
using int8_t   = std::int8_t;
using uint8_t  = std::uint8_t;
using int16_t  = std::int16_t;
using uint16_t = std::uint16_t;
using int32_t  = std::int32_t;
using uint32_t = std::uint32_t;

extern UART_HandleTypeDef huart2;

MotorControllerInterface::MotorControllerInterface(MotorControllerSettings_t *Settings_)
{
  //keep the address of the settings
  Settings = Settings_;

  // get the TMC4671 into a known state

  // setup the default register values and write them to the tmc4671
  for(uint8_t i = 0; i < 0x7D; ++i)
  {
    tmc4671_writeInt(TMC_DEFAULT_MOTOR, i, tmc4671Registers[i]);
  }

  // now update the motor type and pole pairs
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, (Settings->MotorType << 16) | (pole_pairs) );

  // stuff to setup hall effect sensors here (default config good for now)
  // turn on polarity flipping, interpolation, and reverse direction
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_MODE, 0x00001101);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_POSITION_060_000, 0x02AAA000);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_POSITION_180_120, 0x80005555);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_POSITION_300_240, 0xD555AAAA);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_PHI_E_PHI_M_OFFSET, 0x04000000);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_DPHI_MAX, 0x00002AAA);

}

/** 
 * Send the current value of one of the motor controller settings to the host PC
 * The input to the function is the parameter that should be transmitted
 */
void MotorControllerInterface::transmit_setting(MotorControllerParameter_t param)
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
void MotorControllerInterface::change_setting(MotorControllerPacket_t &packet)
{
  //copy the setting from the packet into the class settings struct
  copy_setting(packet, *Settings, true);

  //TODO update the TMC4671 hardware
}

/**
 * This function is meant to be called from the interrupt service routine for
 * the serial device in stm32f3xx_it.c.
 */
void MotorControllerInterface::recieve_packet(MotorControllerPacket_t &packet)
{

}

/**
 * Write a packet to the host PC using either the USB or USART interface
 */
void MotorControllerInterface::transmit_packet(const MotorControllerPacket_t &packet)
{
  const uint8_t* packetPtr = reinterpret_cast<const uint8_t*>(&packet);
  const uint16_t packetSz = static_cast<uint16_t>(sizeof(MotorControllerPacket_t));
#if PC_INTERFACE == UART
  HAL_UART_Transmit(&huart2, (uint8_t*) packetPtr, packetSz, 50);
#elif PC_INTERFACE == USB
  CDC_Transmit_FS(packetPtr, packetSz);
#endif
}

void MotorControllerInterface::init_tmc4671(uint8_t motor_type, uint16_t pole_pairs) 
{
  
}

void MotorControllerInterface::copy_setting(MotorControllerPacket_t &packet, MotorControllerSettings_t &settings, bool toSettings)
{
  // so I don't have to type MotorControllerParameter_t every time
  using MotorParams = MotorControllerParameter_t;

  auto param = static_cast<MotorParams>(packet.rw_address);
  switch(param) {
    // general motor settings
    case MotorParams::MOTOR_DIRECTION: 
      if(!toSettings) packet.u8 = static_cast<uint8_t>(settings.MotorDir);
      else settings.MotorDir = static_cast<MotorDirection_t>(packet.u8);
    break;

    case MotorParams::MOTOR_TYPE:
      if(!toSettings) packet.u8 = static_cast<uint8_t>(settings.MotorType);
      else settings.MotorType = static_cast<MotorType_t>(packet.u8);
    break;
    // open loop settings
    case MotorParams::OPEN_LOOP_MODE_ENABLED:         /// startup in open loop mode 
      if(!toSettings) packet.u8 = settings.OpenStartup;
      else settings.OpenStartup = packet.u8;
    break;

    case MotorParams::OPEN_LOOP_TRANSITION_VELOCITY:  /// RPM to transition from open loop to closed loop mode
      if(!toSettings) packet.u16 = settings.OpenTransistionVel;
      else settings.OpenTransistionVel = packet.u16;
    break;
    
    case MotorParams::OPEN_LOOP_ACCELERATION:         /// Acceleration in RPM/s
      if(!toSettings) packet.u16 = settings.OpenAccel;
      else settings.OpenAccel = packet.u16;
    break;

    case MotorParams::OPEN_LOOP_VELOCITY:             /// Velocity in RPM
      if(!toSettings) packet.u16 = settings.OpenVel;
      else settings.OpenVel = packet.u16;
    break;
    
    case MotorParams::OPEN_LOOP_MAX_I:                /// Max current in mili-Amps
      if(!toSettings) packet.u16 = settings.OpenMaxI;
      else settings.OpenMaxI = packet.u16;
    break;

    case MotorParams::OPEN_LOOP_MAX_V:                /// Max voltage in Volts 
      if(!toSettings) packet.u16 = settings.OpenMaxV;
      else settings.OpenMaxV = packet.u16;
    break;

    // unknown parameter, return early 
    default:
      return;
  }
}

const uint32_t MotorControllerInterface::tmc4671Registers [] =
{
  0x00000000, //0x00: (read only)
  0x00000000, //0x01:
  0x00000000, //0x02: (read only)
  0x00000000, //0x03: 
  0x00100010, //0x04: 
  0x20000000, //0x05: 
  0x00000000, //0x06:
  0x014E014E, //0x07: 
  0x01008001, //0x08: 
  0x01008001, //0x09: 
  0x18000100, //0x0A: 
  0x00000000, //0x0B: 
  0x00044400, //0x0C: 
  0x01000000, //0x0D: 
  0x01000000, //0x0E: 
  0x01000000, //0x0F: 
  0x00000000, //0x10: (reserved)
  0x03020100, //0x11: 
  0x00000000, //0x12: (read only)
  0x00000000, //0x13: (read only)
  0x00000000, //0x14: (read only)
  0x00000000, //0x15: (read only)
  0x00000000, //0x16: (read only)
  0x00000000, //0x17: 
  0x00000F9F, //0x18: 
  0x00000505, //0x19: 
  0x00000007, //0x1A: 
  0x0003000E, //0x1B: 
  0x00000000, //0x1C: 
  0x00000000, //0x1D: 
  0x00000000, //0x1E: 
  0x00000000, //0x1F: 
  0x0000003C, //0x20: 
  0xFFFFFFF6, //0x21: 
  0xFFFFFFFB, //0x22: 
  0x00008BAD, //0x23: 
  0x00000678, //0x24: 
  0x00000000, //0x25: 
  0x00010000, //0x26: 
  0x002641C9, //0x27: 
  0x002641C9, //0x28: 
  0x00000000, //0x29: 
  0x00000000, //0x2A: (read only)
  0x00000000, //0x2B: (reserved)
  0x00000000, //0x2C: 
  0x00010000, //0x2D: 
  0x0025DAC7, //0x2E: 
  0x0025DAC7, //0x2F: 
  0x00000000, //0x30: 
  0x00000000, //0x31: (read only)
  0x00000000, //0x32: (reserved)
  0x00001101, //0x33: 
  0x2AAA0000, //0x34: 
  0x80005555, //0x35: 
  0xD555AAAA, //0x36: 
  0x04000000, //0x37: 
  0x00002AAA, //0x38: 
  0x00000000, //0x39: (read only)
  0x00000000, //0x3A: (read only)
  0x00000000, //0x3B: 
  0x00000000, //0x3C: 
  0x00000000, //0x3D: (read only)
  0x00000000, //0x3E: 
  0x00000000, //0x3F: (read only)
  0x00000001, //0x40: 
  0x00000000, //0x41: (read only)
  0x00000000, //0x42: 
  0x00000000, //0x43: (reserved)
  0x00000000, //0x44: (reserved)
  0x00000000, //0x45: 
  0x00000000, //0x46: (read only)
  0x00000000, //0x47: (read only)
  0x00000000, //0x48: (reserved)
  0x00000000, //0x49: (reserved)
  0x00000000, //0x4A: (reserved)
  0x00000000, //0x4B: (read only)
  0x00000000, //0x4C: (read only)
  0x00000000, //0x4D: 
  0x00000000, //0x4E: 
  0x00000000, //0x4F: (reserved)
  0x00000000, //0x50: 
  0x00000000, //0x51: 
  0x00000005, //0x52: 
  0x00000000, //0x53: (read only)
  0x012C0100, //0x54: 
  0x00000000, //0x55: (reserved)
  0x01000100, //0x56: 
  0x00000000, //0x57: (reserved)
  0x02000100, //0x58: 
  0x00000000, //0x59: (reserved)
  0x00000000, //0x5A: 
  0x00000000, //0x5B: (reserved)
  0x00007FFF, //0x5C: 
  0x00005A81, //0x5D: 
  0x00000FA0, //0x5E: 
  0x00000BB8, //0x5F: 
  0x00001F40, //0x60: 
  0x80000001, //0x61: 
  0x7FFFFFFF, //0x62: 
  0x00000008, //0x63: 
  0x05DC0000, //0x64: 
  0x00000000, //0x65: 
  0x00000000, //0x66: 
  0x00000000, //0x67: 
  0x365A40D8, //0x68: 
  0x00000000, //0x69: (read only)
  0x00000000, //0x6A: (read only)
  0x365A40D8, //0x6B: 
  0x00000000, //0x6C: (read only)
  0x00000000, //0x6D: 
  0x000000D2, //0x6E: 
  0x00000012, //0x6F: 
  0x00000000, //0x70: (reserved)
  0x00000000, //0x71: (reserved)
  0x00000000, //0x72: (reserved)
  0x00000000, //0x73: (reserved)
  0x00000000, //0x74: 
  0xFFFFFFFF, //0x75: 
  0x00000000, //0x76: (read only)
  0x00000000, //0x77: (read only)
  0x00000000, //0x78: 
  0x00009600, //0x79: 
  0x00000000, //0x7A: 
  0x00000000, //0x7B: 
  0xC4788080, //0x7C: 
  0x00000000, //0x7D: 
};