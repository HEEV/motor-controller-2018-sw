#ifndef MOTOR_CONTOLLER_SETTINGS_H_
#define MOTOR_CONTOLLER_SETTINGS_H_
#include <cstdint>

using uint8_t  = std::uint8_t;

enum class MotorControllerSetting : uint8_t {
    // open loop settings
    OPEN_LOOP_MODE_ENABLED = 0,     // startup in open loop mode
    OPEN_LOOP_ACCELERATION,         // Acceleration in RPM/s
    OPEN_LOOP_VELOCITY,             // Velocity in RPM
    OPEN_LOOP_MAX_I,                // Current in mili-Amps
    OPEN_LOOP_MAX_V,                // Voltage in Volts
    OPEN_LOOP_TRANSITION_VELOCITY,  // RPM to transition from open loop to closed loop mode
};

class MotorControllerSettingsManager {
    
// define a enum class for communicating between the host computer and the 
// microcontroller
};

#endif // MOTOR_CONTOLLER_SETTINGS_H_