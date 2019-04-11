#include <CanCallbacks.h>
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_conf.h"
#include "settings_structs.h"
#include "TMC4671Interface.h"

// external ADC global variables
extern volatile std::uint16_t Throttle_ADCVal;
extern volatile std::uint16_t A_In1_ADCVal;
extern volatile std::uint16_t A_In2_ADCVal;
extern volatile std::uint16_t MotorTemp_ADCVal;
extern volatile std::uint16_t TransistorTemp_ADCVal;

// external setting structs
extern MotorControllerValues_t* hmc_settings;
extern TMC4671Interface* htmc4671;
extern volatile uint16_t CAN_watchdog;

void rtrHandle(CanMessage* msg) {
  UNUSED(msg);
}

void mc_dir_handle(CanMessage* msg)
{
  HAL_GPIO_TogglePin(CAN_Status_GPIO_Port, CAN_Status_Pin);
  // switch the direction of the motor
  uint8_t data = static_cast<uint8_t>(hmc_settings->tmc4671.MotorDir);

  // get the data
  CanNode::getData_uint8(msg, &data);

  // set the settings
  hmc_settings->tmc4671.MotorDir = (data == 0) ?
      MotorDirection_t::FORWARD : MotorDirection_t::REVERSE;

  // update the motor direction
  htmc4671->set_direction(hmc_settings->tmc4671.MotorDir);
}

void mc_cmode_handle(CanMessage* msg)
{
  //HAL_GPIO_TogglePin(CAN_Status_GPIO_Port, CAN_Status_Pin);
  // switch the direction of the motor
  ControlMode_t data = ControlMode_t::TORQUE;

  // GET TO THE CHOPPER!!!
  CanNode::getData_uint8(msg, reinterpret_cast<uint8_t*>(&data));

  // put the data into a valid state
  switch (data)
  {
    default:
    data = hmc_settings->tmc4671.ControlMode; 
    break;

    // in the case where the data is valid, do nothing
    case ControlMode_t::TORQUE:
    case ControlMode_t::VELOCITY:
    case ControlMode_t::OPEN_LOOP:
    break;
  }
  hmc_settings->tmc4671.ControlMode = data;

  // change the control mode
  htmc4671->set_control_mode(data);
}

void mc_maxCurrent_handle(CanMessage* msg)
{
  HAL_GPIO_TogglePin(CAN_Status_GPIO_Port, CAN_Status_Pin);
  // initilize with known good values
  uint32_t data = hmc_settings->tmc4671.CurrentLimit;

  CanNode::getData_uint32(msg, &data);

  // error check the current setting
  hmc_settings->tmc4671.CurrentLimit = (data > GLOBAL_MAX_CURRENT) ? 
    GLOBAL_MAX_CURRENT : data; 
}

void mc_maxVel_handle(CanMessage* msg)
{
  HAL_GPIO_TogglePin(CAN_Status_GPIO_Port, CAN_Status_Pin);
  // initilize with known good values
  uint32_t data = hmc_settings->tmc4671.VelocityLimit;

  CanNode::getData_uint32(msg, &data);

  // error check the current setting
  hmc_settings->tmc4671.VelocityLimit = data;
}

void mc_maxAcc_handle(CanMessage* msg)
{
  HAL_GPIO_TogglePin(CAN_Status_GPIO_Port, CAN_Status_Pin);
  // initilize with known good values
  uint32_t data = hmc_settings->tmc4671.AccelerationLimit;

  CanNode::getData_uint32(msg, &data);

  // error check the current setting
  hmc_settings->tmc4671.AccelerationLimit = data;
}

void mc_enable_handle(CanMessage* msg)
{
  UNUSED(msg);
  HAL_GPIO_TogglePin(CAN_Status_GPIO_Port, CAN_Status_Pin);
  // reset "watchdog" counter
  CAN_watchdog = 0;
  // re-enable the TMC4671
  htmc4671->enable();
}

void mc_throttle_handle(CanMessage* msg)
{
  HAL_GPIO_TogglePin(CAN_Status_GPIO_Port, CAN_Status_Pin);

  uint32_t setpoint = 0;
  if(!hmc_settings->General.bool_settings.useAnalog)
  {
    CanNode::getData_uint32(msg, &setpoint);
    htmc4671->set_setpoint(setpoint);
  }
}