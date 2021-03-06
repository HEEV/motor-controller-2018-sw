/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stm32f303xe.h>

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Motor_Temp_Pin GPIO_PIN_0
#define Motor_Temp_GPIO_Port GPIOA
#define Transistor_Temp_Pin GPIO_PIN_1
#define Transistor_Temp_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define A_out1_Pin GPIO_PIN_4
#define A_out1_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Throttle_Pin GPIO_PIN_6
#define Throttle_GPIO_Port GPIOA
#define A_in1_Pin GPIO_PIN_7
#define A_in1_GPIO_Port GPIOA
#define A_in2_Pin GPIO_PIN_4
#define A_in2_GPIO_Port GPIOC
#define CAN_Status_Pin GPIO_PIN_0
#define CAN_Status_GPIO_Port GPIOB
#define Heartbeat_Pin GPIO_PIN_1
#define Heartbeat_GPIO_Port GPIOB
#define User_LED_Pin GPIO_PIN_2
#define User_LED_GPIO_Port GPIOB
#define TMC4671_EN_Pin GPIO_PIN_10
#define TMC4671_EN_GPIO_Port GPIOB
#define TMC4671_SS_Pin GPIO_PIN_12
#define TMC4671_SS_GPIO_Port GPIOB
#define TMC4671_SCK_Pin GPIO_PIN_13
#define TMC4671_SCK_GPIO_Port GPIOB
#define TMC4671_MISO_Pin GPIO_PIN_14
#define TMC4671_MISO_GPIO_Port GPIOB
#define TMC4671_MOSI_Pin GPIO_PIN_15
#define TMC4671_MOSI_GPIO_Port GPIOB
#define D_in1_Pin GPIO_PIN_6
#define D_in1_GPIO_Port GPIOC
#define D_in2_Pin GPIO_PIN_7
#define D_in2_GPIO_Port GPIOC
#define D_in3_Pin GPIO_PIN_8
#define D_in3_GPIO_Port GPIOC
#define D_in4_Pin GPIO_PIN_9
#define D_in4_GPIO_Port GPIOC
#define D_out1_Pin GPIO_PIN_8
#define D_out1_GPIO_Port GPIOA
#define D_out2_Pin GPIO_PIN_9
#define D_out2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SPI3_SS_Pin GPIO_PIN_15
#define SPI3_SS_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

// defines for ADC channels ADC2
#define THROTTLE_ADC 3
#define A_IN1_ADC 4
#define A_IN2_ADC 5

// defines for ADC channels ADC1
#define MOTOR_TEMP_ADC 1
#define TRANSISTOR_TEMP_ADC 2
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
