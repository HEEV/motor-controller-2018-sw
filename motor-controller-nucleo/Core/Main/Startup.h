/**
 * File that has functions for starting up the motor controller
 * 
 * This includes the STM32 clock configuration code and a function
 * that calls the code to start the perhiprerals and load the
 * motor controller settings
 */
#ifndef MC_STARTUP_H
#define MC_STARTUP_H

#include <cstdint>
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

// external setting structs dw
extern SPI_HandleTypeDef *TMC4671_SPI;
extern void* hcomp_iface;
extern MotorControllerValues_t* hmc_settings;
extern TMC4671Interface* htmc4671;

// external CAN variables 
extern volatile uint16_t CAN_watchdog;

// code to initilize the periprieals 
void motor_controller_init();

#endif