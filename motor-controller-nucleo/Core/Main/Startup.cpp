#include "Startup.h"

#include "main.h"

#include "can.h"
#include "spi.h"
#include "usart.h"
#include "adc.h"
#include "tim.h"
#include "wwdg.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "gpio.h"

// stm32cube auto generated clock config code
void SystemClock_Config(void);


void motor_controller_init()
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
  hmc_settings->General.ThrottleRange = {723, 4096};
  hmc_settings->General.bool_settings.useAnalog = 1;
  hmc_settings->tmc4671.MotorDir = MotorDirection_t::FORWARD;
  hmc_settings->tmc4671.ControlMode = ControlMode_t::TORQUE;
  hmc_settings->tmc4671.Setpoint = 0;
  hmc_settings->tmc4671.CurrentLimit = 6000;
  hmc_settings->tmc4671.VelocityLimit = 10000;
  hmc_settings->tmc4671.AccelerationLimit = 1000;
  hmc_settings->tmc4671.MotorType = MotorType_t::BLDC_MOTOR;
  hmc_settings->tmc4671.PolePairs_KV = 7;
  hmc_settings->tmc4671.HallMode.HallPolarity = 0;
  hmc_settings->tmc4671.HallMode.HallInterpolate = 1;
  hmc_settings->tmc4671.HallMode.HallDirection = 0;
  hmc_settings->tmc4671.HallMechOffset = 0;
  hmc_settings->tmc4671.HallElecOffset = 18000;
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
  MX_CAN_Init();

  // check if we were reset by the window watchdog
  if(__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
  {
    // clear the reset flags
    __HAL_RCC_CLEAR_RESET_FLAGS();
    // blink some lights
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