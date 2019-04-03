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

#define MC_DIR_ID (hmc_settings->General.ControllerCanId + 4)
#define MC_CMODE_ID (MC_DIR_ID + 1)
#define MC_MAX_VAL_ID (MC_CMODE_ID + 1)
#define MC_ENABLE_ID  (MC_MAX_VAL_ID + 1)

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

void nodeHandle(CanMessage* msg)
{
  UNUSED(msg);
  // blink led
  HAL_GPIO_TogglePin(User_LED_GPIO_Port, User_LED_Pin);
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

  CanNode node(1000, nodeHandle);
  node.addFilter_id({1004, false}, {1005, false}, {1006, false}, {1007, false}, nodeHandle);

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

  CAN_TxHeaderTypeDef tx_header = {1004, 0, CAN_ID_STD, CAN_RTR_DATA, 2, DISABLE};
  union {
    uint8_t u8_data[8];
    uint16_t data;
  } can_data;
  uint32_t can_mailbox;

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
      //CanNode::checkForMessages();

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
      CanNode::checkForMessages();
      tmc4671.set_setpoint(Throttle_ADCVal - 723);

      //reset count
      ms_cnt50 = 0;
    }
    if (ms_cnt100 >= 100) {
      comp_interface.display_settings();

      HAL_GPIO_TogglePin(Heartbeat_GPIO_Port, Heartbeat_Pin);
      can_data.data = Throttle_ADCVal;
      HAL_CAN_AddTxMessage(&hcan, &tx_header, can_data.u8_data, &can_mailbox);

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
