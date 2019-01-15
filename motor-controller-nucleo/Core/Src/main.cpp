
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
#include "usb_device.h"
#include "gpio.h"

#include <cstring>

#include <CanNode.h>
#include <helpers/API_Header.h>
#include <ic/TMC4671/TMC4671.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef* TMC4671_SPI;
static const int SPI_TIMEOUT = 50; //timeout in ms
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
// define SPI functions for the trinamic API
extern "C" {
  u8 tmc4671_readwriteByte(u8 motor, u8 data, u8 lastTransfer);
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

u8 tmc4671_readwriteByte(u8 motor, u8 data, u8 lastTransfer)
{
  UNUSED(motor);
  uint8_t data_rx;

  //clear the SS pin
  HAL_GPIO_WritePin(TMC4671_SS_GPIO_Port, TMC4671_SS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(TMC4671_SPI, &data, &data_rx, 1, SPI_TIMEOUT);

  //if the last transfer set the SS pin
  if(lastTransfer){
    HAL_GPIO_WritePin(TMC4671_SS_GPIO_Port, TMC4671_SS_Pin, GPIO_PIN_SET);
  }
  return data_rx;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  char buff1[24] = {0};

  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();

  //set the interface the TMC4671 uses
  TMC4671_SPI = &hspi2;
  int reg0_value = 0;
  int reg1_value = 0;

  tmc4671_switchToMotionMode(TMC_DEFAULT_MOTOR, TMC4671_MOTION_MODE_UQ_UD_EXT);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x0003000E);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_PWM_POLARITIES, 0x00000001);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_PWM_MAXCNT, 0x00000F9F);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_PWM_BBM_H_BBM_L, 0x00001414);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_PWM_SV_CHOP, 0x00000007);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_PHI_E_SELECTION, 0x00000002);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_UQ_UD_EXT, 0x000008A9);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_OPENLOOP_ACCELERATION, 30);
  tmc4671_writeInt(TMC_DEFAULT_MOTOR, TMC4671_OPENLOOP_VELOCITY_TARGET, 30);

  //strcpy(buff1, "hello world\n");
  while (1) {

    // get the current time
    uint32_t time = HAL_GetTick();

    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    tmc4671_writeInt(TMC_DEFAULT_MOTOR, 0x01, 0);
    reg0_value = tmc4671_readInt(TMC_DEFAULT_MOTOR, 0x00);
    reg1_value = tmc4671_readInt(TMC_DEFAULT_MOTOR, 0x01);

    memcpy(buff1, &reg0_value, 4);
    buff1[4] = '\0';
    HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t*>(buff1), strlen(buff1)+1, 10);

    reg1_value = atol(buff1);
    HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t*>(buff1), strlen(buff1)+1, 10);
    
    //++reg_value2;
    //if (reg_value2 > 5) reg_value2 = 0;
      
    HAL_Delay(500);
  }

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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
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
