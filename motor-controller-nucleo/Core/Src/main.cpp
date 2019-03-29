
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "can.h"
#include "spi.h"
#include "usart.h"
#include "adc.h"
#include "tim.h"
#include "wwdg.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "gpio.h"

#include <cstring>
#include <cmath>

#include <CanNode.h>
#include <helpers/API_Header.h>
#include <ic/TMC4671/TMC4671.h>

#include "settings_structs.h"
#include <ComputerInterface.h>
#include <TMC4671Interface.h>
//#include <ComputerMenu.h>

/* USER CODE BEGIN Includes */
#define MC_DIR_ID (hmc_settings->General.ControllerCanId + 4)
#define MC_CMODE_ID (MC_DIR_ID + 1)
#define MC_MAX_VAL_ID (MC_CMODE_ID + 1)
#define MC_ENABLE_ID  (MC_MAX_VAL_ID + 1)
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef *TMC4671_SPI;
CanNode  *mc_node_ptr;
void* hcomp_iface;
MotorControllerValues_t* hmc_settings;
TMC4671Interface* htmc4671;

// global variables for the analog inputs
static volatile uint16_t Throttle_ADCVal;
static volatile uint16_t A_In1_ADCVal;
static volatile uint16_t A_In2_ADCVal;
static volatile uint16_t MotorTemp_ADCVal;
static volatile uint16_t TransistorTemp_ADCVal;

// global CAN watchdog
static volatile uint16_t CAN_watchdog;

// maximum value is 100ms
const uint16_t MAX_CAN_WATCHDOG = 100;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
int thermistorTemperature(uint16_t adcVal);
void mc_nodeRTR(CanMessage *msg);
void mc_nodeHandle(CanMessage *msg);

// initilize global variables and hardware 
void init();

/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

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
  init();

  // intilize CAN variables
  auto base_id = mc_settings.General.ControllerCanId;
  CanNode mc_node(static_cast<CanNodeType>(base_id), mc_nodeRTR);
  // add some filters
  mc_node.addFilter(1004, mc_nodeHandle);
  mc_node.addFilter(MC_CMODE_ID, mc_nodeHandle);
  mc_node.addFilter(MC_MAX_VAL_ID, mc_nodeHandle);
  mc_node.addFilter(MC_ENABLE_ID, mc_nodeHandle);
  mc_node_ptr = &mc_node;

  // setup the two main hardware interfaces
  TMC4671Interface  tmc4671(&mc_settings.tmc4671);
  htmc4671 = &tmc4671;

  ComputerInterface comp_interface(&mc_settings, &tmc4671);

  // initilize pointer for the USB interface
  hcomp_iface = &comp_interface;

  uint32_t time = 0;
  uint32_t prev_time = 0;

  // Use some variables as counters.
  // These variable should be read as milisecond count 50, and milisecond count 100 respectively.
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

      /*
      strcpy(buff1, "\f\r");
      __itoa(Throttle_ADCVal, tmpBuff, 10);
      strcat(buff1, tmpBuff);

      strcat(buff1, "\n\r");
      __itoa(A_In1_ADCVal, tmpBuff, 10);
      strcat(buff1, tmpBuff);
      strcat(buff1, "\n\r");

      __itoa(A_In2_ADCVal, tmpBuff, 10);
      strcat(buff1, tmpBuff);
      strcat(buff1, "\n\r");

      __itoa(thermistorTemperature(MotorTemp_ADCVal), tmpBuff, 10);
      strcat(buff1, tmpBuff);
      strcat(buff1, "\n\r");

      __itoa(thermistorTemperature(TransistorTemp_ADCVal), tmpBuff, 10);
      strcat(buff1, tmpBuff);
      CDC_Transmit_FS(reinterpret_cast<uint8_t*>(buff1), strlen(buff1)+1);
      */
      comp_interface.display_settings();


      HAL_GPIO_TogglePin(Heartbeat_GPIO_Port, Heartbeat_Pin);

      //mc_node_ptr->sendData_uint16(Throttle_ADCVal);
      //reset count
      ms_cnt100 = 0;
    }
  }
}

void init()
{
  // initilize the ADC variables
  Throttle_ADCVal = 0;
  A_In1_ADCVal = 0;
  A_In2_ADCVal = 0;
  MotorTemp_ADCVal = 0;
  TransistorTemp_ADCVal = 0; 

  // initilize the CAN watchdog
  CAN_watchdog = 0;

  // Some default motor settings
  // Trinamic Power Board
  hmc_settings->General.ControllerCanId = 1000;
  hmc_settings->General.ThrottleCanId = 900;
  hmc_settings->General.bool_settings.useAnalog = 1;
  hmc_settings->tmc4671.MotorDir = MotorDirection_t::FORWARD;
  hmc_settings->tmc4671.ControlMode = ControlMode_t::TORQUE;
  hmc_settings->tmc4671.Setpoint = 0;
  hmc_settings->tmc4671.CurrentLimit = 6000;
  hmc_settings->tmc4671.VelocityLimit = 10000;
  hmc_settings->tmc4671.AccelerationLimit = 1000;
  hmc_settings->tmc4671.MotorType = MotorType_t::BLDC_MOTOR;
  hmc_settings->tmc4671.PolePairs_KV = 7;
  hmc_settings->tmc4671.HallMode.HallPolarity = 1;
  hmc_settings->tmc4671.HallMode.HallInterpolate = 1;
  hmc_settings->tmc4671.HallMode.HallDirection = 0;
  hmc_settings->tmc4671.HallMechOffset = 0;
  hmc_settings->tmc4671.HallElecOffset = -8100;
  hmc_settings->tmc4671.FluxP = 256;
  hmc_settings->tmc4671.FluxI = 256;
  hmc_settings->tmc4671.TorqueP = 256;
  hmc_settings->tmc4671.TorqueI = 256;
  hmc_settings->tmc4671.VelocityP = 256;
  hmc_settings->tmc4671.VelocityI = 256;
  hmc_settings->tmc4671.OpenAccel = 0;
  hmc_settings->tmc4671.OpenVel = 0;
  hmc_settings->tmc4671.OpenMaxI = 0;
  hmc_settings->tmc4671.OpenMaxV = 0;

  // *hmc_settings = 
  // {
  //   { // TMC4671 Settings
  //     MotorDirection_t::FORWARD,
  //     ControlMode_t::TORQUE,
  //     0,      // Setpoint

  //     6000,   // current limit
  //     10000,  // velocity limit
  //     1000,    // acceleration limit

  //     MotorType_t::BLDC_MOTOR,
  //     7,      // Pole Pairs

  //     {
  //       0,    // Hall Polarity
  //       1,    // Hall Interpolate
  //       1     // Hall Direction
  //     },
  //     0,      // Hall Mechanical Offset
  //     0,      // Hall Electrical Offset

  //     256,    // flux P
  //     256,    // flux I
  //     256,    // torque P
  //     256,    // torque I
  //     256,    // velocity P
  //     256,    // torque I

  //     0,      // Open Loop Acceleration
  //     0,      // Open Loop Velocity
  //     0,      // Open Loop max Current
  //     0       // Open Loop max Voltage
  //   }
  // };

  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize most configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  MX_ADC1_Init();

  // check if we were reset by the window watchdog
  if(__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
  {
    // set the user pin high
    HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin, GPIO_PIN_SET);
    // clear the reset flags
    __HAL_RCC_CLEAR_RESET_FLAGS();
    Error_Handler();
  }

  // initilize the window watchdog
  MX_WWDG_Init();

  // Calibrate ADCs (the datasheet says this is a good idea)
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  //set the interface the TMC4671 uses (defined in spi.c)
  TMC4671_SPI = &hspi2;

  // initilize USB
  MX_USB_DEVICE_Init();

  // start the two ADC's in interrupt mode
  // start the ADCs: the ADCs run a sequence of conversions with an interrupt after each one.
  // After each conversion the ADC callback function is run, which puts the ADC value into the
  // proper global varibles. Timer6 is used to trigger ADC conversions for a sample rate of ~ 1Khz.
  HAL_ADC_Start_IT(&hadc2);
  HAL_ADC_Start_IT(&hadc1);
  // start the ADC conversion trigger timer
  HAL_TIM_Base_Start(&htim6);
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

/** ADC conversion code
 * This is the callback function from HAL_ADC_Start_IT, it is called for both
 * ADC1 and ADC2. The function checks which ADC initiated the interrupt, then
 * it finds which channel was converted by means of a static iteration variable.
 * (A static variable is defined in the function, but holds its value over multiple
 * function calls.) 
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  static int8_t adc1_conv_count = 0;
  static int8_t adc2_conv_count = 0;

  // check if the sequence is over with
  bool end_of_sequence = __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS);

  // read the ADCValue (this will clear some of the flags)
  uint16_t ADCValue = static_cast<uint16_t>(HAL_ADC_GetValue(hadc));

  if (hadc->Instance == ADC1)
  {
    // which channel to put the ADC value in
    switch(adc1_conv_count)
    {
      default:
      case 0:
        //throtle reading
        MotorTemp_ADCVal = ADCValue;
      break;

      case 1:
        // A_In1 reading
        TransistorTemp_ADCVal = ADCValue;
      break;
    }
    adc1_conv_count++;

    // go back to the beginning
    if (end_of_sequence) 
    {
      adc1_conv_count = 0;
    }
  }
  else if (hadc->Instance == ADC2)
  {
    // which channel to put the ADC value in
    switch(adc2_conv_count)
    {
      default:
      case 0:
        //throtle reading
        Throttle_ADCVal = ADCValue;
      break;

      case 1:
        // A_In1 reading
        A_In1_ADCVal = ADCValue;
      break;

      case 2:
        // A_In2 reading
        A_In2_ADCVal = ADCValue;
      break;
    }
    adc2_conv_count++;

    // go back to the beginning
    if (end_of_sequence) 
    {
      adc2_conv_count = 0;
    }
  }
  
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef* hwwdg) 
{
  UNUSED(hwwdg);

  // set pin high
  HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin, GPIO_PIN_SET);
  // turn of TMC4671 outputs
  htmc4671->disable();
}

void mc_nodeHandle(CanMessage* msg)
{
  // toggle the CAN status pin
  HAL_GPIO_TogglePin(CAN_Status_GPIO_Port, CAN_Status_Pin);
  
  // switch what we do based on the id of the message
  if (msg->id == MC_DIR_ID)
  {
    // switch the direction of the motor
    MotorDirection_t data = MotorDirection_t::FORWARD;
    mc_node_ptr->getData_uint8(msg, (uint8_t*) &data);
    hmc_settings->tmc4671.MotorDir = (data == MotorDirection_t::FORWARD) ?
        MotorDirection_t::FORWARD : MotorDirection_t::REVERSE;
  }
  else if (msg->id == MC_CMODE_ID)
  {
    // switch the direction of the motor
    ControlMode_t data = ControlMode_t::TORQUE;
    mc_node_ptr->getData_uint8(msg, (uint8_t*) &data);

    // put the data into a valid state
    switch (data)
    {
      default:
      data = hmc_settings->tmc4671.ControlMode; 
      break;

      case ControlMode_t::TORQUE:
      case ControlMode_t::VELOCITY:
      case ControlMode_t::OPEN_LOOP:
      break;
    }
    hmc_settings->tmc4671.ControlMode = data;
  }
  else if (msg->id == MC_MAX_VAL_ID)
  {
    uint16_t data[3] = {
      hmc_settings->tmc4671.CurrentLimit, 
      hmc_settings->tmc4671.VelocityLimit,
      hmc_settings->tmc4671.AccelerationLimit
    };

    uint8_t len = 0;
    mc_node_ptr->getDataArr_uint16(msg, data, &len);
    if (len == 3)
    {
      // error check the current setting
      hmc_settings->tmc4671.CurrentLimit = (data[0] > GLOBAL_MAX_CURRENT) ? 
        GLOBAL_MAX_CURRENT : data[0]; 
      hmc_settings->tmc4671.VelocityLimit = data[1];
      hmc_settings->tmc4671.AccelerationLimit = data[2];
    }

  }
  else if (msg->id == MC_ENABLE_ID)
  {
    // reset "watchdog" counter
    CAN_watchdog = 0;
    // re-enable the TMC4671
    htmc4671->enable();
  }
}

void mc_nodeRTR(CanMessage *msg){
// do nothing
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV4;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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
