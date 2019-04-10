#include "Actions.h"
#include "settings_structs.h"
#include <TMC4671Interface.h>
#include <cstdlib>
#include <cmath>

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

int16_t thermistorTemperature(uint16_t adcVal);

int16_t get_motor_temperature()
{
  return thermistorTemperature(MotorTemp_ADCVal);
}

int16_t get_transistor_temperature()
{
  return thermistorTemperature(TransistorTemp_ADCVal);
}

uint32_t get_analog_setpoint(uint16_t value)
{

  auto map = [](range_t x_range, range_t y_range, uint16_t x) -> uint32_t
  {
    float del_y = (y_range.max - y_range.min);
    float del_x = (x_range.max - x_range.min);
    float dif_x = abs(x - x_range.min);

    return ((uint32_t) (del_y * dif_x/del_x)) + y_range.min;
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

int16_t thermistorTemperature(uint16_t adcVal)
{
  const float THERMISTOR_NOMINAL = 10000.0; // 10K
  const float BCOEFFICIENT = 3950; // a number picked out of thin air
  const auto TEMPERATURE_NOMINAL = 25;

  float resistance  = (4096 / static_cast<float>(adcVal)) - 1;
  resistance = THERMISTOR_NOMINAL / resistance;

  float steinhart;
  steinhart = resistance / THERMISTOR_NOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C

  int16_t tempurature = steinhart * 100;
  return tempurature;
}
