#include "TMC4671Interface.h"
#include <ic/TMC4671/TMC4671.h>
#include "settings_structs.h"

// used for pin the SPI functions for pin useage
#include "main.h"

using uint8_t = std::uint8_t;
using int32_t = std::int32_t;
using uint32_t = std::uint32_t;

// Use the global variable defined in main.cpp so we can implement the trinamic readwriteByte function
extern SPI_HandleTypeDef *TMC4671_SPI;

// define some helper functions (implemented after the class function implementations)


// define SPI functions for the trinamic API (implementation near the bottom of file)
extern "C" {
  u8 tmc4671_readwriteByte(u8 motor, u8 data, u8 lastTransfer);
}

// detailed documentation in the .h file
TMC4671Interface::TMC4671Interface(TMC4671Settings_t *settings)
{
  // get the TMC4671 into a known state

  // setup the default register values and write them to the tmc4671
  for(uint8_t i = 0; i < 0x7D; ++i)
  {
    tmc4671_writeInt(TMC_DEFAULT_MOTOR, i, tmc4671Registers[i]);
  }

  // initilize important TMC4671 registers
  pwm_init();
  adc_init();

  // intilize variables
  Settings = settings;
  ControlMode = ControlMode_t::VELOCITY;
  Direction   = MotorDirection_t::FORWARD;
  Setpoint    = 0;
  MotorConstant = 0;

  // initilize the user defined settings
  change_settings(settings);
}

// detailed information in the .h file
void TMC4671Interface::change_settings(const TMC4671Settings_t *settings) 
{
  // initilize hall effect sensors
  hall_effect_init(*settings);

  const uint8_t motor_type = static_cast<uint8_t>(settings->MotorType);

  // Use the Pole-pairs as the motor constant if this is a BLDC motor. This might be stupid.
  // If this is a brushed motor, then this will be the KV constant (RPM/voltage) of the motor 
  MotorConstant = settings->PolePairs_KV;

  // if using a brushed motor, there is always only one pole. ALWAYS!!!
  const uint8_t pole_pairs = 
      (settings->MotorType == MotorType_t::BRUSHED_MOTOR) ? 1 : MotorConstant;

  // now update the motor type and pole pairs
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, (motor_type << 16) | (pole_pairs) );

  //update the limits (current, velocity, and acceleration)
  tmc4671_writeRegister16BitValue(TMC_DEFAULT_MOTOR, TMC4671_PID_TORQUE_FLUX_LIMITS, BIT_0_TO_15, settings->CurrentLimit/CURRENT_DIVISOR);

  // multiply the velocity and acceleration limits times the motor constant to get accurate velocity and acceleration limits
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_PID_VELOCITY_LIMIT, MotorConstant*settings->VelocityLimit);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_PID_ACCELERATION_LIMIT, MotorConstant*settings->AccelerationLimit);

  // change the PID values
  uint32_t pi_value = (settings->FluxP << TMC4671_PID_FLUX_P_SHIFT) | settings->FluxI;
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_PID_FLUX_P_FLUX_I, pi_value);

  pi_value = (settings->TorqueP << TMC4671_PID_TORQUE_P_SHIFT) | settings->TorqueI;
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_PID_TORQUE_P_TORQUE_I, pi_value);

  pi_value = (settings->VelocityP << TMC4671_PID_VELOCITY_P_SHIFT) | settings->VelocityI;
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_PID_VELOCITY_P_VELOCITY_I, pi_value);

  // now initilize the settings that will be changed the most frequently
  set_control_mode(settings->ControlMode);
  set_direction(settings->MotorDir);
  set_setpoint(Setpoint);
}

// documented in the .h file
void TMC4671Interface::set_control_mode(ControlMode_t mode) 
{
  uint8_t tmc_mode = TMC4671_MOTION_MODE_VELOCITY; 
  switch (mode) {
    case ControlMode_t::VELOCITY :
      // this statement is redundant
      tmc_mode = TMC4671_MOTION_MODE_VELOCITY;
    break; 

    default:
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

// documented in the .h file
void TMC4671Interface::set_direction(MotorDirection_t dir)
{
  Direction = dir;
  Settings->MotorDir = dir;
  // Start the motor going in the new direction
  set_setpoint(Setpoint);
}

// documented int the .h file
void TMC4671Interface::set_setpoint(uint32_t set_point)
{
  // take the absolute value of the set point (direction determined by the Direction variable)

  // update settings struct
  Settings->Setpoint = Setpoint;

  switch (ControlMode) {
    default:
    case ControlMode_t::VELOCITY :
    {
      set_point = (set_point > INT32_MAX) ? INT32_MAX : set_point;
      Setpoint = (Direction == MotorDirection_t::REVERSE) ? -set_point : set_point;

      // Note: this could be very dumb (or it could be briliant)
      int32_t tmc_setpoint = MotorConstant * Setpoint;
      tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_PID_VELOCITY_TARGET, tmc_setpoint);
      break;
    }

    case ControlMode_t::TORQUE :
    {
      // make sure we don't overflow
      set_point = (set_point >= (INT16_MAX-1)*CURRENT_DIVISOR) ? (INT16_MAX-1)*CURRENT_DIVISOR : set_point;
      Setpoint = (Direction == MotorDirection_t::REVERSE) ? -set_point : set_point;

      // divide by 2 to get into 2mA incriments
      int16_t tmc_setpoint = Setpoint / CURRENT_DIVISOR;
      tmc4671_writeRegister16BitValue(TMC_DEFAULT_MOTOR, TMC4671_PID_TORQUE_FLUX_TARGET, BIT_16_TO_31, tmc_setpoint);
      break;
    }

    case ControlMode_t::OPEN_LOOP :
    break;
  }
}

// documented in the .h file
void TMC4671Interface::enable()
{
  HAL_GPIO_WritePin(TMC4671_EN_GPIO_Port, TMC4671_EN_Pin, GPIO_PIN_SET);
}

// documented in the .h file
void TMC4671Interface::disable(){
  HAL_GPIO_WritePin(TMC4671_EN_GPIO_Port, TMC4671_EN_Pin, GPIO_PIN_RESET);
}

float TMC4671Interface::get_motor_current()
{
  // get the raw current values (2mA incriments) 
  int32 raw_current = tmc4671_getActualTorque_raw(TMC_DEFAULT_MOTOR);

  // rescale to get the current into mA
  return static_cast<float>(raw_current) * CURRENT_DIVISOR;
}

// Deal with the error in the 4671 datasheet 
// (0-2v input instead of a 0-5v input)
float TMC4671Interface::get_battery_current()
{
  // select which adc register to read
  const int AGPI_1_VM = 1;
  //setup to read the ADC value
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_ADC_RAW_ADDR, AGPI_1_VM);
  uint16_t value = tmc4671_readRegister16BitValue(TMC_DEFAULT_MOTOR, TMC4671_ADC_RAW_DATA, BIT_16_TO_31);

  //return static_cast<float>(value) * static_cast<float>(BATTERY_CURRENT_SCALE) - BATTERY_CURRENT_OFFSET;
  auto current_ma = static_cast<float>(value - BATTERY_CURRENT_OFFSET) * BATTERY_CURRENT_SCALE;
  return current_ma/1000.0;
}

// Deal with the error in the 4671 datasheet 
// (0-2v input instead of a 0-5v input)
float TMC4671Interface::get_battery_voltage()
{

  // select which adc register to read
  const int AGPI_1_VM = 1;
  //setup to read the ADC value
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_ADC_RAW_ADDR, AGPI_1_VM);
  uint16_t value = tmc4671_readRegister16BitValue(TMC_DEFAULT_MOTOR, TMC4671_ADC_RAW_DATA, BIT_0_TO_15);

  const float fudge = -2.88;
  return ((value - BATTERY_ADC_MIN) * BATTERY_MAX)/ static_cast<float>(BATTERY_ADC_MAX - BATTERY_ADC_MIN) + fudge;
}

int32_t TMC4671Interface::get_motor_RPM()
{
  return tmc4671_getActualVelocity(TMC_DEFAULT_MOTOR)/MotorConstant;
}

// -------------------------------- Helper functions -------------------------

/** Initilize hall TMC4671 hall effect registers 
 * This is a helper function to initilize the hall effect sensors. The function
 * takes the motor controller settings as an input, because the electrical and
 * mechanical offsets are dependent on the motor used, and therefore need to
 * be easily modifiable by the end user. 
 */
void TMC4671Interface::hall_effect_init(const TMC4671Settings_t &motor_settings)
{
  // stuff to setup hall effect sensors here (default config good for now)
  const uint32_t HALL_POSITION[] = {0x55557FFF, 0x00012AAB, 0xAAADD557};
  const uint32_t MAX_INTERPOLATION = 0x00002AAA;

  uint32_t HALL_OFFSET = (motor_settings.HallElecOffset << 16) | motor_settings.HallMechOffset;

  uint32_t HALL_MODE = (motor_settings.HallMode.HallDirection << 12) 
                     | (motor_settings.HallMode.HallInterpolate << 8)
                     | (motor_settings.HallMode.HallPolarity);

  // turn on interpolation, and reverse polarity 
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_MODE, HALL_MODE);

  // define the "position" of the rotor with each hall effect pulse
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_POSITION_060_000, HALL_POSITION[0]);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_POSITION_180_120, HALL_POSITION[1]);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_POSITION_300_240, HALL_POSITION[2]);
  // define the electrical offset between the hall effect senors and the motor
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_PHI_E_PHI_M_OFFSET, HALL_OFFSET);
  // define the maximum ammount to interpolate
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_HALL_DPHI_MAX, MAX_INTERPOLATION);
}

void TMC4671Interface::adc_init() 
{
  const uint8_t ADC_I0_SELECT = 0;
  const uint8_t ADC_I1_SELECT = 1;
  const uint8_t ADC_I_UX_SELECT = 0;
  const uint8_t ADC_I_V_SELECT = 1;
  const uint8_t ADC_I_WY_SELECT = 2;

  // constants for Trinamic's power board
  // const uint8_t ADC_I0_SELECT = 0;
  // const uint8_t ADC_I1_SELECT = 1;
  // const uint8_t ADC_I_UX_SELECT = 0;
  // const uint8_t ADC_I_V_SELECT = 2;
  // const uint8_t ADC_I_WY_SELECT = 1;

  // setup which registers the ADC reads for each phase current
  const uint32_t adc_selection = 
                            (ADC_I0_SELECT   << TMC4671_ADC_I0_SELECT_SHIFT)   | 
                            (ADC_I1_SELECT   << TMC4671_ADC_I1_SELECT_SHIFT)   |
                            (ADC_I_UX_SELECT << TMC4671_ADC_I_UX_SELECT_SHIFT) |
                            (ADC_I_V_SELECT  << TMC4671_ADC_I_V_SELECT_SHIFT)  |
                            (ADC_I_WY_SELECT << TMC4671_ADC_I_WY_SELECT_SHIFT);

  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_ADC_I_SELECT, adc_selection);

  //setup the ADC scaling and offset
  tmc4671_writeRegister16BitValue(TMC_DEFAULT_MOTOR, TMC4671_ADC_I0_SCALE_OFFSET, BIT_0_TO_15, ADC_PHASE1_OFFSET);
  tmc4671_writeRegister16BitValue(TMC_DEFAULT_MOTOR, TMC4671_ADC_I0_SCALE_OFFSET, BIT_16_TO_31, ADC_PHASE1_SCALE);

  tmc4671_writeRegister16BitValue(TMC_DEFAULT_MOTOR, TMC4671_ADC_I1_SCALE_OFFSET, BIT_0_TO_15, ADC_PHASE2_OFFSET);
  tmc4671_writeRegister16BitValue(TMC_DEFAULT_MOTOR, TMC4671_ADC_I1_SCALE_OFFSET, BIT_16_TO_31, ADC_PHASE2_SCALE);
}

void TMC4671Interface::pwm_init() 
{

  //set the PWM polarities to be 1-on, 0-off
  uint8_t PWM_POLARITIES = 0;

  // set pwm frequency to be 50khz
  // pwm frequency (hz) = 100 (Mhz) / (PWM_MAXCNT + 1)
  // (p72 in the TMC4671 datasheet)
  uint16_t PWM_MAXCNT = 1999;

  // 30 * 10 ns break before make time
  // (p72 in the TMC4671 datasheet)
  uint8_t BBM_TIME = 30; 

  // put the pwm into centered pwm for FOC 
  // (p73 on the TMC4671 datasheet)
  uint8_t PWM_CHOP_MODE = 7;

  // temperarory variable for writting to 32bit registers
  uint32_t temp_reg = 0;

  // set the PWM polarities register
  temp_reg = PWM_POLARITIES;
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_PWM_POLARITIES, temp_reg);

  // set the PWM max count register
  temp_reg = PWM_MAXCNT;
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_PWM_MAXCNT, temp_reg);

  // set the PWM break before make times
  temp_reg = (BBM_TIME << 8) | (BBM_TIME);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_PWM_BBM_H_BBM_L, temp_reg);

  //set the PWM chopping mode
  temp_reg = PWM_CHOP_MODE;
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_PWM_SV_CHOP, temp_reg);
}


// --------------------- Trinamic Library Helper function -------------------------

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

// -------------------------Register initilization values -----------------------

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