/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_conf.h"

#include "can.h"
#include "spi.h"
#include "usart.h"
#include "adc.h"
#include "tim.h"
#include "wwdg.h"
#include "gpio.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <cstring>
#include <cmath>

#include <helpers/API_Header.h>
#include <ic/TMC4671/TMC4671.h>

#include "settings_structs.h"
#include <ComputerInterface.h>
#include <TMC4671Interface.h>

#include <CanNode.h>
#include <Startup.h>

#define MC_DIR_ID     (static_cast<uint16_t>(hmc_settings->General.ControllerCanId + 4) )
#define MC_CMODE_ID   (static_cast<uint16_t>(MC_DIR_ID + 1 ) )
#define MC_MAX_I_ID   (static_cast<uint16_t>(MC_CMODE_ID + 1) )
#define MC_MAX_VEL_ID (static_cast<uint16_t>(MC_MAX_I_ID + 1) )
#define MC_MAX_ACC_ID (static_cast<uint16_t>(MC_MAX_VEL_ID + 1) )
#define MC_ENABLE_ID  (static_cast<uint16_t>(MC_MAX_ACC_ID + 1 ) )

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef *TMC4671_SPI;
void* hcomp_iface;
MotorControllerValues_t* hmc_settings;
TMC4671Interface* htmc4671;

// global variables for the analog inputs
volatile uint16_t Throttle_ADCVal;
volatile uint16_t A_In1_ADCVal;
volatile uint16_t A_In2_ADCVal;
volatile uint16_t MotorTemp_ADCVal;
volatile uint16_t TransistorTemp_ADCVal;

// global CAN watchdog
volatile uint16_t CAN_watchdog;

// maximum value is 100ms
const uint16_t MAX_CAN_WATCHDOG = 100;

/* Private function prototypes -----------------------------------------------*/
int thermistorTemperature(uint16_t adcVal);

void rtrHandle(CanMessage* msg) {
  UNUSED(msg);
}

void mc_dir_handle(CanMessage* msg)
{
  HAL_GPIO_TogglePin(CAN_Status_GPIO_Port, CAN_Status_Pin | User_LED_Pin);
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
  uint16_t data = hmc_settings->tmc4671.CurrentLimit;

  CanNode::getData_uint16(msg, &data);

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
  //HAL_GPIO_TogglePin(CAN_Status_GPIO_Port, CAN_Status_Pin);
  // reset "watchdog" counter
  CAN_watchdog = 0;
  // re-enable the TMC4671
  htmc4671->enable();
}

void mc_throttle_handle(CanMessage* msg)
{
  UNUSED(msg);
  //HAL_GPIO_TogglePin(CAN_Status_GPIO_Port, CAN_Status_Pin);
}
/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  MotorControllerValues_t mc_settings;
  hmc_settings = &mc_settings;

  // initilize most of the global variables and
  // brings up the motor controller settings struct
  motor_controller_init();
  // intilize CAN variables
  auto base_id = mc_settings.General.ControllerCanId;
  auto throttle_id = mc_settings.General.ThrottleCanId;

  CanNode node(base_id, rtrHandle);

  // add filters
  node.addFilter_id(
    {MC_ENABLE_ID, false},
    {throttle_id, false},
    {MC_DIR_ID, false},
    {MC_CMODE_ID, false},
    mc_enable_handle,
    mc_throttle_handle,
    mc_dir_handle,
    mc_cmode_handle
  );

  // add more filters
  node.addFilter_id(
    {MC_MAX_I_ID, false}, 
    {MC_MAX_VEL_ID, false}, 
    {MC_MAX_ACC_ID, false}, 
    {MC_MAX_ACC_ID, false}, 
    mc_maxCurrent_handle,
    mc_maxVel_handle,
    mc_maxAcc_handle,
    mc_maxAcc_handle
  );

  // setup the two main hardware interfaces
  TMC4671Interface  tmc4671(&mc_settings.tmc4671);
  htmc4671 = &tmc4671;

  ComputerInterface comp_interface(&mc_settings, &tmc4671);

  // initilize pointer for the USB interface
  hcomp_iface = &comp_interface;

  uint32_t time = 0;
  uint32_t prev_time = 0;

  // Use some variables as counters.
  // These variable should be read as milisecond count 25, and milisecond count 50, etc.
  // Counter variables are used because of their increased reliablitiy over simply using the mod
  // operator on the time variable (i.e. time % 100 == 0). This is because the time variable
  // might not get updated exactly every milisecond (or the main loop doesn't quite run that fast).
  // This will cause some missed updates or double updates.
  // The counter variable solves this by triggering when it is at or above the ammount of time required,
  // then clearing once the event has happened.
  int8_t ms_cnt25 = 0;
  int8_t ms_cnt50 = 0;
  int8_t ms_cnt100 = 0;


  // enable the outputs from the tmc4671
  tmc4671.enable();

  // enable window watchdog
  __HAL_WWDG_ENABLE(&hwwdg);
  while (1) {
    // Get the time difference (if this stops working for some reason put in a 1ms delay)
    prev_time = time;
    time = HAL_GetTick();
    int8_t time_diff = static_cast<uint8_t> (time - prev_time);

    //update counters
    ms_cnt25 += time_diff;
    ms_cnt50 += time_diff;
    ms_cnt100 += time_diff;

    if (ms_cnt25 >= 25) {
      // clear the watchdog counter
      HAL_WWDG_Refresh(&hwwdg);
      CanNode::checkForMessages();

      CAN_watchdog += 25;
      bool use_analog = mc_settings.General.bool_settings.useAnalog;
      if (CAN_watchdog > MAX_CAN_WATCHDOG && !use_analog)
      {
        // disable TMC4671 outputs
        tmc4671.disable();
      }
      ms_cnt25 = 0;
    }
    if (ms_cnt50 >= 50) {
      tmc4671.set_setpoint(Throttle_ADCVal - 723);

      //reset count
      ms_cnt50 = 0;
    }
    if (ms_cnt100 >= 100) {
      comp_interface.display_settings();

      HAL_GPIO_TogglePin(Heartbeat_GPIO_Port, Heartbeat_Pin);
      //can_data.data = Throttle_ADCVal;
      // HAL_CAN_AddTxMessage(&hcan, &tx_header, can_data.u8_data, &can_mailbox);

      //reset count
      ms_cnt100 = 0;
    }
  }
}


int thermistorTemperature(uint16_t adcVal)
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

  int tempurature = steinhart * 100;
  return tempurature;
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  UNUSED(file);
  UNUSED(line);
  /* User can add his own implementation to report the HAL error return state */
  // make all of the leds on
  HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin | Heartbeat_Pin | CAN_Status_Pin, GPIO_PIN_SET);
  while(1)
  {
    // toggle LEDS
    HAL_GPIO_TogglePin(User_LED_GPIO_Port, User_LED_Pin | Heartbeat_Pin | CAN_Status_Pin);
    HAL_Delay(250);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
