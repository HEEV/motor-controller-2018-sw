
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
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "gpio.h"

#include <cstring>
#include <cmath>

#include <CanNode.h>
#include <helpers/API_Header.h>
#include <ic/TMC4671/TMC4671.h>
#include "ComputerInterface.h"
#include "TMC4671Interface.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef *TMC4671_SPI;
CanNode  *mc_node_ptr;
void* hcomp_iface;

// global variables for the analog inputs
static volatile uint16_t Throttle_ADCVal;
static volatile uint16_t A_In1_ADCVal;
static volatile uint16_t A_In2_ADCVal;
static volatile uint16_t MotorTemp_ADCVal;
static volatile uint16_t TransistorTemp_ADCVal; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
int thermistorTemperature(uint16_t adcVal);
void mc_nodeRTR(CanMessage *msg);

/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{

  // initilize the ADC variables
  Throttle_ADCVal = 0;
  A_In1_ADCVal = 0;
  A_In2_ADCVal = 0;
  MotorTemp_ADCVal = 0;
  TransistorTemp_ADCVal = 0; 

  // Some default motor settings
  MotorControllerSettings_t mc_settings;
  mc_settings.MotorDir = MotorDirection_t::REVERSE;
  mc_settings.ControlMode = ControlMode_t::TORQUE;
  mc_settings.Setpoint = 0;

  mc_settings.CurrentLimit = 6000;
  mc_settings.VelocityLimit = 10000;
  mc_settings.AccelerationLimit = 500;

  mc_settings.MotorType = MotorType_t::BLDC_MOTOR;
  mc_settings.PolePairs_KV = 7;

  mc_settings.HallMode.HallPolarity = 1;
  mc_settings.HallMode.HallInterpolate = 1;
  mc_settings.HallMode.HallDirection = 0;
  mc_settings.HallElecOffset = -10000;
  mc_settings.HallMechOffset = 0;

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

  // Calibrate ADCs (the datasheet says this is a good idea)
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  CanNode mc_node(static_cast<CanNodeType>(1000), mc_nodeRTR);
  mc_node_ptr = &mc_node;

  //set the interface the TMC4671 uses (defined in spi.c)
  TMC4671_SPI = &hspi2;

  // setup the two main hardware interfaces
  ComputerInterface comp_interface(&mc_settings);
  TMC4671Interface  tmc4671(&mc_settings);

  // initilize pointer for the USB interface
  hcomp_iface = &comp_interface;

  // initilize USB
  MX_USB_DEVICE_Init();

  int32_t target_velocity = 0;
  char buff1[128] = {0};
  char tmpBuff[6];
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
  int8_t ms_cnt50 = 0;
  int8_t ms_cnt100 = 0;

  // start the two ADC's in interrupt mode
  // start the ADCs: the ADCs run a sequence of conversions with an interrupt after each one.
  // After each conversion the ADC callback function is run, which puts the ADC value into the
  // proper global varibles. Timer6 is used to trigger ADC conversions for a sample rate of ~ 1Khz.
  HAL_ADC_Start_IT(&hadc2);
  HAL_ADC_Start_IT(&hadc1);
  // start the ADC conversion trigger timer
  HAL_TIM_Base_Start(&htim6);

  // enable the outputs from the tmc4671
  tmc4671.enable();
  while (1) {
    // Get the time difference (if this stops working for some reason put in a 1ms delay)
    prev_time = time;
    time = HAL_GetTick();
    uint8_t time_diff = static_cast<uint8_t> (time - prev_time);


    //update counters
    ms_cnt50 += time_diff;
    ms_cnt100 += time_diff;

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

      mc_node_ptr->sendData_uint16(Throttle_ADCVal);

      //reset count
      ms_cnt100 = 0;
    }
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
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
    HAL_GPIO_TogglePin(User_LED_GPIO_Port, User_LED_Pin);
    HAL_Delay(100);
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
