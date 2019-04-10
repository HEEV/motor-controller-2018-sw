#include "SettingsManager.h"

void SettingsManager::write_setting(MotorControllerParameter_t param, int32_t value)
{
  // so I don't have to type MotorControllerParameter_t every time
  using MotorParams = MotorControllerParameter_t;

  auto& tmc4671 = user_settings->tmc4671;
  auto& gen_settings = user_settings->General;

  switch (param)
  {
  // general motor tmc4671
  case MotorParams::MOTOR_DIRECTION:
    tmc4671.MotorDir = (MotorDirection_t) value;
    htmc4671->set_direction(tmc4671.MotorDir);
    break;
    
  case MotorParams::MOTOR_MODE:
    tmc4671.ControlMode = (ControlMode_t) value;
    htmc4671->set_control_mode(tmc4671.ControlMode);
    break;

  case MotorParams::SETPOINT:
    tmc4671.Setpoint = (std::uint32_t) value; 
    htmc4671->set_setpoint(tmc4671.Setpoint);
    break;

  // CAN Settings
  case MotorParams::CONTROLLER_CAN_ID:
    value = (value > 0x7FF) ? 0x7FF : value;
    gen_settings.ControllerCanId = (std::uint16_t) value;
    break;

  case MotorParams::THROTTLE_CAN_ID:
    value = (value > 0x7FF) ? 0x7FF : value;
    gen_settings.ThrottleCanId = (std::uint16_t) value;
    break;

  // Torque, velocity, and acceleration limits
  case MotorParams::CURRENT_LIMIT:
    tmc4671.CurrentLimit = ((std::uint32_t) value > GLOBAL_MAX_CURRENT) ?
        GLOBAL_MAX_CURRENT : (std::uint32_t) value; 
    htmc4671->change_settings(&tmc4671);
    break;

  case MotorParams::VELOCITY_LIMIT :
    tmc4671.VelocityLimit = (std::uint32_t) value; 
    htmc4671->change_settings(&tmc4671);
    break;

  case MotorParams::ACCELERATION_LIMIT :
    tmc4671.AccelerationLimit = (std::uint32_t) value; 
    htmc4671->change_settings(&tmc4671);

  // Motor type tmc4671
  case MotorParams::MOTOR_TYPE:
    switch(value) {
      default: // fall through
      case 0: tmc4671.MotorType = MotorType_t::BLDC_MOTOR; break;
      case 1: tmc4671.MotorType = MotorType_t::BRUSHED_MOTOR; break;
    }
    htmc4671->change_settings(&tmc4671);
    break;

  case MotorParams::POLE_PAIRS_KV:
    tmc4671.PolePairs_KV = (std::uint8_t) value; 
    htmc4671->change_settings(&tmc4671);
    break;

  // Hall effect tmc4671
  case MotorParams::HALL_POLARITY:
    tmc4671.HallMode.HallPolarity = static_cast<std::uint8_t>(value == 1); 
    htmc4671->change_settings(&tmc4671);
    break;

  case MotorParams::HALL_INTERPOLATE:
    tmc4671.HallMode.HallInterpolate = static_cast<std::uint8_t>(value == 1); 
    htmc4671->change_settings(&tmc4671);
    break;

  case MotorParams::HALL_DIRECTION:
    tmc4671.HallMode.HallDirection = static_cast<std::uint8_t>(value == 1); 
    htmc4671->change_settings(&tmc4671);
    break;

  case MotorParams::HALL_MECH_OFFSET:
    tmc4671.HallMechOffset = (std::int16_t) value; 
    htmc4671->change_settings(&tmc4671);
    break;

  case MotorParams::HALL_ELEC_OFFSET:
    tmc4671.HallElecOffset = (std::int16_t) value; 
    htmc4671->change_settings(&tmc4671);
    break;

  // PI settings
  case MotorParams::FLUX_P:
    tmc4671.FluxP = (std::uint16_t) value;
    htmc4671->change_settings(&tmc4671);
    break;

  case MotorParams::FLUX_I:
    tmc4671.FluxI = (std::uint16_t) value;
    htmc4671->change_settings(&tmc4671);
    break;

  case MotorParams::TORQUE_P:
    tmc4671.TorqueP = (std::uint16_t) value;
    htmc4671->change_settings(&tmc4671);
    break;
  
  case MotorParams::TORQUE_I:
    tmc4671.TorqueI = (std::uint16_t) value;
    htmc4671->change_settings(&tmc4671);
    break;

  case MotorParams::VELOCITY_P:
    tmc4671.VelocityP = (std::uint16_t) value;
    htmc4671->change_settings(&tmc4671);
    break;

  case MotorParams::VELOCITY_I:
    tmc4671.VelocityI = (std::uint16_t) value;
    htmc4671->change_settings(&tmc4671);
    break;

  // open loop tmc4671
  case MotorParams::OPEN_LOOP_ACCELERATION: /// Acceleration in RPM/s
    tmc4671.OpenAccel = (std::uint16_t) value; 
    htmc4671->change_settings(&tmc4671);
    break;

  case MotorParams::OPEN_LOOP_MAX_I: /// Max current in mili-Amps
    tmc4671.OpenMaxI = (std::uint16_t) value; 
    htmc4671->change_settings(&tmc4671);
    break;

  case MotorParams::OPEN_LOOP_MAX_V: /// Max voltage in Volts
    tmc4671.OpenMaxV = (std::uint16_t) value; 
    htmc4671->change_settings(&tmc4671);
    break;
  
  case MotorParams::USE_ANALOG :
    gen_settings.bool_settings.useAnalog = static_cast<std::uint8_t>(value == 1); 
    break;

  case MotorParams::ANALOG_SETUP:
    auto_throttle_setup(value);
    break;

  case MotorParams::SAVE_SETTINGS :
    if(value == 1){
      save_settings();
    }
    break;

  case MotorParams::HALL_AUTO_SETUP:
    // call the hall auto setup function
    break;

  // unknown parameter, return early
  default:
    break;
  }
}