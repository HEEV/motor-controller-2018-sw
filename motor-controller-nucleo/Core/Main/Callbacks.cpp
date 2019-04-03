/**
 * Callback code for the Motor controller
 */

#include <cstdint>
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_conf.h"
#include "settings_structs.h"
#include "TMC4671Interface.h"
#include <CanNode.h>

// external ADC global variables
extern volatile std::uint16_t Throttle_ADCVal;
extern volatile std::uint16_t A_In1_ADCVal;
extern volatile std::uint16_t A_In2_ADCVal;
extern volatile std::uint16_t MotorTemp_ADCVal;
extern volatile std::uint16_t TransistorTemp_ADCVal;

// external setting structs dw
extern SPI_HandleTypeDef *TMC4671_SPI;
extern MotorControllerValues_t* hmc_settings;
extern TMC4671Interface* htmc4671;

#define MC_DIR_ID (hmc_settings->General.ControllerCanId + 4)
#define MC_CMODE_ID (MC_DIR_ID + 1)
#define MC_MAX_VAL_ID (MC_CMODE_ID + 1)
#define MC_ENABLE_ID  (MC_MAX_VAL_ID + 1)

// external watchdog counter
extern volatile uint16_t CAN_watchdog;

extern const uint16_t MAX_CAN_WATCHDOG;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  HAL_GPIO_TogglePin(CAN_Status_GPIO_Port, CAN_Status_Pin);
  CAN_RxHeaderTypeDef rx_header;
  uint8_t data[8];

  HAL_CAN_GetRxMessage(hcan, 0, &rx_header, data);

  // fill a CanNode message
  CanMessage msg = {
    static_cast<uint16_t>(rx_header.StdId),
    static_cast<uint8_t>(rx_header.DLC),
    static_cast<uint8_t>(rx_header.FilterMatchIndex),
    (rx_header.RTR == 0),
    {data[0], data[1], data[2], data[3],
     data[4], data[5], data[6], data[7]}
  };

  CanNode::updateMessage(&msg);

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
  auto ADCValue = static_cast<uint16_t>(HAL_ADC_GetValue(hadc));

  if (hadc->Instance == ADC1)
  {
    // which channel to put the ADC value in
    switch(adc1_conv_count)
    {
      // Motor temperature
      default:
      case 0: MotorTemp_ADCVal = ADCValue; break;

      // Transistor temperature
      case 1: TransistorTemp_ADCVal = ADCValue; break;
    }
    adc1_conv_count++;

    // go back to the beginning of the sequence
    if (end_of_sequence) adc1_conv_count = 0;
  }
  else if (hadc->Instance == ADC2)
  {
    // which channel to put the ADC value in
    switch(adc2_conv_count)
    {
      //throtle reading
      default:
      case 0: Throttle_ADCVal = ADCValue; break;

      // A_In1 reading
      case 1: A_In1_ADCVal = ADCValue; break;

      // A_In2 reading
      case 2: A_In2_ADCVal = ADCValue; break;
    }
    adc2_conv_count++;

    // go back to the beginning of the sequence
    if (end_of_sequence) adc2_conv_count = 0;
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