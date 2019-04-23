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

void mc_dir_handle(CanMessage* msg)
{
  HAL_GPIO_TogglePin(CAN_Status_GPIO_Port, CAN_Status_Pin);
  // switch the direction of the motor
  uint8_t data = static_cast<uint8_t>(hmc_settings->tmc4671.MotorDir);

  // get the data
  CanNode::getData_uint8(msg, &data);

  // set the settings
  switch (data) {
  case 0:
    hmc_settings->tmc4671.MotorDir = MotorDirection_t::FORWARD;
    break;
  case 1:
    hmc_settings->tmc4671.MotorDir = MotorDirection_t::REVERSE;
    break;
  default:
    break;
  }

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
    // don't go into open loop mode (or any other invalid mode)
    default:
    case ControlMode_t::OPEN_LOOP:
      data = hmc_settings->tmc4671.ControlMode; 
    break;

    // in the case where the data is valid, do nothing
    case ControlMode_t::TORQUE: // fall through
    case ControlMode_t::VELOCITY:
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

  htmc4671->change_settings(&hmc_settings->tmc4671);
}

void mc_maxVel_handle(CanMessage* msg)
{
  HAL_GPIO_TogglePin(CAN_Status_GPIO_Port, CAN_Status_Pin);
  // initilize with known good values
  uint32_t data = hmc_settings->tmc4671.VelocityLimit;

  CanNode::getData_uint32(msg, &data);

  // error check the current setting
  hmc_settings->tmc4671.VelocityLimit = data;
  htmc4671->change_settings(&hmc_settings->tmc4671);
}

void mc_maxAcc_handle(CanMessage* msg)
{
  HAL_GPIO_TogglePin(CAN_Status_GPIO_Port, CAN_Status_Pin);
  // initilize with known good values
  uint32_t data = hmc_settings->tmc4671.AccelerationLimit;

  CanNode::getData_uint32(msg, &data);

  // error check the current setting
  hmc_settings->tmc4671.AccelerationLimit = data;
  htmc4671->change_settings(&hmc_settings->tmc4671);
}

void mc_enable_handle(CanMessage* msg)
{
  UNUSED(msg);
  HAL_GPIO_TogglePin(CAN_Status_GPIO_Port, CAN_Status_Pin);
  // reset "watchdog" counter
  CAN_watchdog = 0;
  // the chip will be enabled in the main function
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