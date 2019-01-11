#include "MotorControllerInterface.h"
#include <stm32f3xx_hal.h>
// from USB_DEVICE/App directory
#include <usbd_cdc_if.h>

// define commonly used types
using int8_t   = std::int8_t;
using uint8_t  = std::uint8_t;
using int16_t  = std::int16_t;
using uint16_t = std::uint16_t;
using int32_t  = std::int32_t;
using uint32_t = std::uint32_t;


MotorControllerInterface::MotorControllerInterface(MotorControllerSettings_t &Settings)
{

}

/** 
 * Send the current value of one of the motor controller settings to the host PC
 * The input to the function is the parameter that should be transmitted
 */
void MotorControllerInterface::transmit_setting(MotorControllerParameter_t param)
{

}

/**
 * Change one of the motor controller settings based on a packet (assumed to have
 * been recieved from the host computer).
 */
void MotorControllerInterface::change_setting(MotorControllerPacket_t setting)
{

}

/**
 * This function is meant to be called from the interrupt service routine for
 * the serial device in stm32f3xx_it.c.
 */
void MotorControllerInterface::recieve_packet(MotorControllerPacket_t &packet)
{

}

void MotorControllerInterface::transmit_packet(const MotorControllerPacket_t)
{

}