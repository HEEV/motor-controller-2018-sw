#include "SettingsManager.h"

const char* SettingsManager::get_setting_as_string(char* buff, MotorControllerParameter_t param)
{
  // so I don't have to type MotorControllerParameter_t every time
  using MotorParams = MotorControllerParameter_t;

  auto& tmc4671 = user_settings->tmc4671;
  auto& gen_settings = user_settings->General;

  // make a lambda function to convert a bit slice into a string
  auto bit2Str = [](uint8_t bit) { return bit ? "On" : "Off";};

  switch (param)
  {
  // general motor tmc4671
  case MotorParams::MOTOR_DIRECTION:
    switch(tmc4671.MotorDir) {
      default:
      case MotorDirection_t::FORWARD: return "Forward";
      case MotorDirection_t::REVERSE: return "Reverse";
    } 
    
  case MotorParams::MOTOR_MODE:
    switch(tmc4671.ControlMode) {
      default:
      case ControlMode_t::VELOCITY:   return "Velocity";
      case ControlMode_t::TORQUE:     return "Torque";
      case ControlMode_t::OPEN_LOOP:  return "Open Loop";
    } 

  case MotorParams::SETPOINT:
  {
    auto units = [](ControlMode_t cm) {
      return (cm == ControlMode_t::TORQUE) ? "mA" : "RPM";
    };
    my_sprintf(buff, "%5ld %s", tmc4671.Setpoint, units(tmc4671.ControlMode));
    return buff; 
  }

  // CAN Settings
  case MotorParams::CONTROLLER_CAN_ID:
    my_sprintf(buff, "%4u (0x%03X)", gen_settings.ControllerCanId, gen_settings.ControllerCanId);
    return buff;

  case MotorParams::THROTTLE_CAN_ID:
    my_sprintf(buff, "%4u (0x%03X)", gen_settings.ThrottleCanId, gen_settings.ThrottleCanId);
    return buff;

  // Torque, velocity, and acceleration limits
  case MotorParams::CURRENT_LIMIT:
    my_sprintf(buff, "%lu mA", tmc4671.CurrentLimit);
    return buff; 

  case MotorParams::VELOCITY_LIMIT :
    my_sprintf(buff, "%ld RPM", tmc4671.VelocityLimit);
    return buff; 

  case MotorParams::ACCELERATION_LIMIT :
    my_sprintf(buff, "%ld RPM/Sec", tmc4671.AccelerationLimit);
    return buff; 

  // Motor type tmc4671
  case MotorParams::MOTOR_TYPE:
    switch(tmc4671.MotorType) {
      default:
      case MotorType_t::BLDC_MOTOR :    return "BLDC Motor";
      case MotorType_t::BRUSHED_MOTOR : return "Brushed Motor";
    } 

  case MotorParams::POLE_PAIRS_KV:
  {
    auto units = [](MotorType_t mt) {
      return (mt == MotorType_t::BLDC_MOTOR) ? "Pole Pairs" : "RPM/Volt";
    };
    my_sprintf(buff, "%d %s", tmc4671.PolePairs_KV, units(tmc4671.MotorType));
    return buff; 
  }

  // Hall effect tmc4671
  case MotorParams::HALL_POLARITY:
    return bit2Str( tmc4671.HallMode.HallPolarity );

  case MotorParams::HALL_INTERPOLATE:
    return bit2Str( tmc4671.HallMode.HallInterpolate );

  case MotorParams::HALL_DIRECTION:
    return bit2Str( tmc4671.HallMode.HallDirection );

  case MotorParams::HALL_MECH_OFFSET:
    my_sprintf(buff, "%d", tmc4671.HallMechOffset);
    return buff; 

  case MotorParams::HALL_ELEC_OFFSET:
    my_sprintf(buff, "%d", tmc4671.HallElecOffset);
    return buff; 

  // PI settings
  case MotorParams::FLUX_P:
    my_sprintf(buff, "%u", tmc4671.FluxP);
    return buff;

  case MotorParams::FLUX_I:
    my_sprintf(buff, "%u", tmc4671.FluxI);
    return buff;

  case MotorParams::TORQUE_P:
    my_sprintf(buff, "%u", tmc4671.TorqueP);
    return buff;
  
  case MotorParams::TORQUE_I:
    my_sprintf(buff, "%u", tmc4671.TorqueI);
    return buff;

  case MotorParams::VELOCITY_P:
    my_sprintf(buff, "%u", tmc4671.VelocityP);
    return buff;

  case MotorParams::VELOCITY_I:
    my_sprintf(buff, "%u", tmc4671.VelocityI);
    return buff;

  // open loop tmc4671
  case MotorParams::OPEN_LOOP_ACCELERATION: /// Acceleration in RPM/s
    my_sprintf(buff, "%u RPM/Sec", tmc4671.OpenAccel);
    return buff; 

  case MotorParams::OPEN_LOOP_MAX_I: /// Max current in mili-Amps
    my_sprintf(buff, "%lu mA", tmc4671.OpenMaxI);
    return buff; 

  case MotorParams::OPEN_LOOP_MAX_V: /// Max voltage in Volts
    my_sprintf(buff, "%u V", tmc4671.OpenMaxV);
    return buff; 
  
  case MotorParams::USE_ANALOG :
    return bit2Str( gen_settings.bool_settings.useAnalog );

  case MotorParams::ANALOG_SETUP:
  {
    auto& throttle_range = gen_settings.ThrottleRange;
    my_sprintf(buff, "Min: %ld, Max: %ld", throttle_range.min, throttle_range.max);
    return buff;
  }

  case MotorParams::SAVE_SETTINGS :
    return "";

  case MotorParams::HALL_AUTO_SETUP:
    return "";

  case MotorParams::CURRENT:
  {
    auto motor_current = htmc4671->get_motor_current()/1000;
    my_sprintf(buff, "%6.3f A", motor_current);
    return buff;
  }

  case MotorParams::VELOCITY:
  {
    auto motor_RPM = htmc4671->get_motor_RPM();
    my_sprintf(buff, "%5ld RPM", motor_RPM);
    return buff;
  }

  case MotorParams::MOTOR_TEMPERATURE:
  {
    float motor_temp = get_motor_temperature();
    my_sprintf(buff, "%6.1f C", motor_temp/100);
    return buff;
  }

  case MotorParams::TRANSISTOR_TEMPERATURE:
  {
    float transistor_temp = get_transistor_temperature();
    my_sprintf(buff, "%6.1f C", transistor_temp/100);
    return buff;
  }

  case MotorParams::LIVE_VALUES:
    return "";

  // unknown parameter, return early
  default:
    return "Unknown Parameter";
  }
}