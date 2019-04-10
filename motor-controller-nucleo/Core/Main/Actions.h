#ifndef ACTIONS_H
#define ACTIONS_H

#include <cstdint>

std::int16_t get_motor_temperature();
std::int16_t get_transistor_temperature();
std::uint32_t get_analog_setpoint(std::uint16_t value);
void auto_throttle_setup(std::int32_t value);
#endif //ACTIONS_H