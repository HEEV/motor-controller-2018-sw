#include "Actions.h"
#include "settings_structs.h"
#include <TMC4671Interface.h>

// external ADC global variables
extern volatile std::uint16_t Throttle_ADCVal;
extern volatile std::uint16_t A_In1_ADCVal;
extern volatile std::uint16_t A_In2_ADCVal;
extern volatile std::uint16_t MotorTemp_ADCVal;
extern volatile std::uint16_t TransistorTemp_ADCVal;

extern MotorControllerValues_t* hmc_settings;
extern TMC4671Interface* htmc4671;

using uint16_t = std::uint16_t;
using uint32_t = std::uint32_t;
using int32_t  = std::int32_t;

uint32_t get_analog_setpoint(uint16_t value)
{

  auto map = [](range_t x_range, range_t y_range, uint16_t x) -> uint32_t
  {
    return ((y_range.max - y_range.min) * (x - x_range.min)) /
      (x_range.max - x_range.min) + y_range.min;
  };

  auto torque_mode = hmc_settings->tmc4671.ControlMode == ControlMode_t::TORQUE;
  auto output_max = (torque_mode) ? 
    hmc_settings->tmc4671.CurrentLimit : hmc_settings->tmc4671.VelocityLimit;

  range_t output_range = { 0, output_max};
  auto input_range = hmc_settings->General.ThrottleRange;

  return map(input_range, output_range, value);
}

void auto_throttle_setup(int32_t value)
{
  static enum{
    MIN_VALUE,
    MAX_VALUE
  } state = MIN_VALUE;

  if (value != 1) return;

  auto& throttle_range = hmc_settings->General.ThrottleRange;

  switch(state)
  {
    default:
    case MIN_VALUE:
      throttle_range.min = Throttle_ADCVal;
      state = MAX_VALUE;
    break;

    case MAX_VALUE:
      throttle_range.max = Throttle_ADCVal;
      state = MIN_VALUE;
    break;
  }

}