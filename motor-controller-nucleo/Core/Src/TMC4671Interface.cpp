#include "TMC4671Interface.h"
#include <ic/TMC4671/TMC4671.h>

// used for pin the SPI functions for pin useage
#include "main.h"

using uint8_t = std::uint8_t;
using int32_t = std::int32_t;
using uint32_t = std::uint32_t;

// Use the global variable defined in main.cpp so we can implement the trinamic readwriteByte function
extern SPI_HandleTypeDef *TMC4671_SPI;

// define some helper functions (implemented after the class function implementations)
void hall_effect_init();

// define SPI functions for the trinamic API (implementation near the bottom of file)
extern "C" {
  u8 tmc4671_readwriteByte(u8 motor, u8 data, u8 lastTransfer);
}

TMC4671Interface::TMC4671Interface(const MotorControllerSettings_t *settings)
{
  // get the TMC4671 into a known state

  // setup the default register values and write them to the tmc4671
  for(uint8_t i = 0; i < 0x7D; ++i)
  {
    tmc4671_writeInt(TMC_DEFAULT_MOTOR, i, tmc4671Registers[i]);
  }

  // initilize hall effect registers
  hall_effect_init();

  // intilize variables
  ControlMode = ControlMode_t::VELOCITY;
  Direction   = MotorDirection_t::FORWARD;
  Setpoint    = 0;
  MotorConstant = 0;

  change_settings(settings);
}

void TMC4671Interface::change_settings(const MotorControllerSettings_t *settings) 
{
  // go through the settings, update the TMC4671 (or internal state) based on them

  Direction = settings->MotorDir;

  const uint8_t motor_type = static_cast<uint8_t>(settings->MotorType);

  // Use the Pole-pairs/motor constant as the motor constant
  MotorConstant = settings->PolePairs_KV;

  // if using a brushed motor, there is always only one pole. ALWAYS!!!
  const uint8_t pole_pairs = 
      (settings->MotorType == MotorType_t::BRUSHED_MOTOR) ? 1 : MotorConstant;

  // now update the motor type and pole pairs
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, (motor_type << 16) | (pole_pairs) );

  // set the mode of the motor controller
  set_control_mode(settings->ControlMode);
  // set the motor direction
  set_direction(settings->MotorDir);
  // set the setpoint
  set_setpoint(settings->Setpoint);
}

void TMC4671Interface::set_control_mode(ControlMode_t mode) 
{
  uint8_t tmc_mode = TMC4671_MOTION_MODE_VELOCITY; 
  switch (mode) {
    default:
    case ControlMode_t::VELOCITY :
      // this statement is redundant
      tmc_mode = TMC4671_MOTION_MODE_VELOCITY;
    break; 

    case ControlMode_t::TORQUE :
      tmc_mode = TMC4671_MOTION_MODE_TORQUE; 
    break;

    case ControlMode_t::OPEN_LOOP :
      tmc_mode = TMC4671_MOTION_MODE_UQ_UD_EXT; 
    break;
  }
  // set the class state
  ControlMode = mode;
  // configure the TMC4671
  tmc4671_switchToMotionMode(TMC_DEFAULT_MOTOR, tmc_mode);
}

void TMC4671Interface::set_direction(MotorDirection_t dir)
{
  Direction = dir;
  // Start the motor going in the new direction
  set_setpoint(Setpoint);
}

void TMC4671Interface::set_setpoint(int32_t set_point)
{
  // take the absolute value of the set point (direction determined by the Direction variable)
  set_point = (set_point < 0) ? -set_point : set_point;
  Setpoint = set_point;

  switch (ControlMode) {
    default:
    case ControlMode_t::VELOCITY :
    break;

    case ControlMode_t::TORQUE :
    break;

    case ControlMode_t::OPEN_LOOP :
    break;
  }
}

// Initilize hall TMC4671 hall effect registers 
void hall_effect_init()
{
  // stuff to setup hall effect sensors here (default config good for now)
  const uint32_t HALL_POSITION[] = {0x55557FFF, 0x00012AAB, 0xAAADD557};
  const uint16_t ELECTRICAL_OFFSET = 0x2328;
  const uint16_t MECHANICAL_OFFSET = 0x0000;
  const uint32_t HALL_OFFSET = (ELECTRICAL_OFFSET << 16) | MECHANICAL_OFFSET;
  const uint32_t MAX_INTERPOLATION = 0x00002AAA;

  // turn on interpolation, and reverse direction
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_MODE, 0x00000101);

  // define the "position" of the rotor with each hall effect pulse
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_POSITION_060_000, HALL_POSITION[0]);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_POSITION_180_120, HALL_POSITION[1]);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_POSITION_300_240, HALL_POSITION[2]);
  // define the electrical offset between the hall effect senors and the motor
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_PHI_E_PHI_M_OFFSET, HALL_OFFSET);
  // define the maximum ammount to interpolate
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_DPHI_MAX, MAX_INTERPOLATION);
}


u8 tmc4671_readwriteByte(u8 motor, u8 data, u8 lastTransfer)
{
  UNUSED(motor);
  const uint32_t SPI_TIMEOUT = 50; // spi timeout in ms
  uint8_t data_rx;

  //clear the SS pin
  HAL_GPIO_WritePin(TMC4671_SS_GPIO_Port, TMC4671_SS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(TMC4671_SPI, &data, &data_rx, 1, SPI_TIMEOUT);

  //if the last transfer set the SS pin
  if(lastTransfer){
    HAL_GPIO_WritePin(TMC4671_SS_GPIO_Port, TMC4671_SS_Pin, GPIO_PIN_SET);
  }
  return data_rx;
}

// register values for startup
const uint32_t TMC4671Interface::tmc4671Registers [] =
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