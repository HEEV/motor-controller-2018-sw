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

#include <helpers/API_Header.h>
#include <ic/TMC4671/TMC4671.h>

#include "settings_structs.h"
#include <ComputerInterface.h>
#include <TMC4671Interface.h>
#include <SettingsManager.h>

#include <CanNode.h>
#include <Startup.h>
#include "CanCallbacks.h"
#include "Actions.h"

#define MC_DIR_ID     (static_cast<uint16_t>(hmc_settings->General.ControllerCanId + 6) )
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

// CAN variables 
volatile uint16_t CAN_watchdog;
const uint16_t MAX_CAN_WATCHDOG = 100;

CanNode* hmc_node;
CanNode* htemperature_node;
CanNode* hmc_current_node;
CanNode* hmc_rpm_node;
CanNode* hbatt_current_node;
CanNode* hbatt_voltage_node;
/* Private function prototypes -----------------------------------------------*/

enum class can_group_t : uint8_t {
  TEMPERATURE,
  MOTOR_CURRENT_RPM,
  BATTERY_CURRENT_VOLTAGE
};

void can_send_data(can_group_t group);

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

  // load settings from flash
  SettingsManager settings_manager(hmc_settings);

  // intilize CAN variables
  auto base_id = mc_settings.General.ControllerCanId;
  auto throttle_id = mc_settings.General.ThrottleCanId;
  // setup the nodes the +4 incriment is because each node
  // reserves 4 ids
  CanNode mc_node(base_id, rtrHandle);
  CanNode temperature_node (base_id+1, rtrHandle);
  CanNode mc_current_node  (base_id+2, rtrHandle);
  CanNode mc_rpm_node      (base_id+3, rtrHandle);
  CanNode batt_current_node(base_id+4, rtrHandle);
  CanNode batt_voltage_node(base_id+5, rtrHandle);
  
  //setup node references
  htemperature_node   = &temperature_node;
  hmc_current_node    = &mc_current_node;
  hmc_rpm_node        = &mc_rpm_node;
  hbatt_current_node  = &batt_current_node;
  hbatt_voltage_node  = &batt_voltage_node;

  // add filters
  mc_node.addFilter_id(
    {MC_ENABLE_ID, false},
    {throttle_id,  false},
    {MC_DIR_ID,    false},
    {MC_CMODE_ID,  false},
    mc_enable_handle,
    mc_throttle_handle,
    mc_dir_handle,
    mc_cmode_handle
  );

  // add more filters
  mc_node.addFilter_id(
    {MC_MAX_I_ID,   false}, 
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

  ComputerInterface comp_interface(&mc_settings, &tmc4671, &settings_manager);

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
  int8_t  ms_cnt25 = 0;
  int8_t  ms_cnt50 = 0;
  int16_t ms_cnt200 = 0;
  int16_t ms_cnt300 = 0;

  //enum for simple state machine
  can_group_t group = can_group_t::TEMPERATURE;


  // enable the outputs from the tmc4671
  tmc4671.enable();

  // enable window watchdog
  __HAL_WWDG_ENABLE(&hwwdg);
  while (1) {
    // Get the time difference (if this stops working for some reason put in a 1ms delay)
    prev_time = time;
    time = HAL_GetTick();
    int8_t time_diff = static_cast<uint8_t> (time - prev_time);
    bool use_analog = mc_settings.General.bool_settings.useAnalog;

    //update counters
    ms_cnt25 += time_diff;
    ms_cnt50 += time_diff;
    ms_cnt200 += time_diff;
    ms_cnt300 += time_diff;

    if (ms_cnt25 >= 25) {
      // clear the watchdog counter
      HAL_WWDG_Refresh(&hwwdg);
      CanNode::checkForMessages();

      CAN_watchdog += 25;
      if (CAN_watchdog > MAX_CAN_WATCHDOG && !use_analog)
      {
        // disable TMC4671 outputs
        //tmc4671.disable();
      }
      ms_cnt25 = 0;
    }
    if (ms_cnt50 >= 50) {
      uint32_t DEADBAND = 25;
      if(use_analog)
      {
        auto setpoint = get_analog_setpoint(Throttle_ADCVal);
        if(setpoint < DEADBAND) setpoint = 0;
        tmc4671.set_setpoint(setpoint);
      } 

      // check if the output is enabled or not
      if (mc_settings.General.bool_settings.enableOutputs == 1){
        HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin, GPIO_PIN_SET);
        tmc4671.enable();
      }
      else {
        HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin, GPIO_PIN_RESET);
        tmc4671.disable();
      }

      // send CAN values
      can_send_data(group);

      //select which group to send next
      switch(group)
      {
        default:
        case can_group_t::TEMPERATURE: 
          group = can_group_t::MOTOR_CURRENT_RPM;
        break;

        case can_group_t::MOTOR_CURRENT_RPM:
          group = can_group_t::BATTERY_CURRENT_VOLTAGE;
        break;

        case can_group_t::BATTERY_CURRENT_VOLTAGE:
          group = can_group_t::TEMPERATURE;
        break;
      }

      //reset count
      ms_cnt50 = 0;
    }
    if (ms_cnt200 >= 200) {

      HAL_GPIO_TogglePin(Heartbeat_GPIO_Port, Heartbeat_Pin);

      //reset count
      ms_cnt200 = 0;
    }
    if (ms_cnt300 >= 300) {
      comp_interface.display_settings();

      //reset count
      ms_cnt300 = 0;
    }
  }
}

void can_send_data(can_group_t group){
  switch(group)
  {
    case can_group_t::TEMPERATURE:
    {
      int16_t temperatures[] = 
      { get_motor_temperature(), get_transistor_temperature() };

      htemperature_node->sendDataArr_int16(temperatures, 2);
      break;
    }

    default:
    case can_group_t::MOTOR_CURRENT_RPM:
    {
      // get the values from the tmc4671
      int32_t rpm     = htmc4671->get_motor_RPM();
      float   current = htmc4671->get_motor_current();
      
      hmc_rpm_node->sendData_int32(rpm);
      hmc_current_node->sendData_float(current/1000);
      break;
    }

    case can_group_t::BATTERY_CURRENT_VOLTAGE:
    {
      // get the values from the tmc4671
      float voltage = htmc4671->get_battery_voltage();
      float current = htmc4671->get_battery_current();

      hbatt_voltage_node->sendData_float(voltage/1000);
      hbatt_current_node->sendData_float(current/1000);
      break;
    }
  }  
}


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
