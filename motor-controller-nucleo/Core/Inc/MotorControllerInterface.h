/** MotorControllerInterface
 * \author Samuel Ellicott 
 * 
 * This class and structures is designed as an interface between the host PC
 * and the actual microcontroller doing the control loops. i.e it takes the
 * data from the serial line and converts it into a (global) settings struct
 * used in the motor control loop to send commands to the TMC4671 doing all
 * of the actual work.
 * 
 * The setting struct consists of various parameters that can be changed 
 * (the MotorControllerParameter_t enum). Each of these settings has a name
 * and associated value described in the enum definition.
 * 
 * The settings are changed by reciving a packet MotorControllerPacket_t from
 * the host computer. The rw_address in the packet corresponds to an entry in
 * the MotorControllerParameter_t enum, then the appropriate data is taken from
 * the data union.
 */

#ifndef SERIAL_INTERFACE_H_
#define SERIAL_INTERFACE_H_

#include <cstdint>

using uint8_t  = std::uint8_t;
using int8_t   = std::int8_t;
using uint32_t = std::uint32_t;

enum class MotorControllerParameter_t : uint8_t {
    // open loop settings
    OPEN_LOOP_MODE_ENABLED = 0,     /// startup in open loop mode
    OPEN_LOOP_ACCELERATION,         /// Acceleration in RPM/s
    OPEN_LOOP_VELOCITY,             /// Velocity in RPM
    OPEN_LOOP_MAX_I,                /// Current in mili-Amps
    OPEN_LOOP_MAX_V,                /// Voltage in Volts
    OPEN_LOOP_TRANSITION_VELOCITY,  /// RPM to transition from open loop to closed loop mode
};

struct MotorControllerPacket_t {
    uint8_t rw_address;
    union data {
        uint8_t  u8_arr[4];
        int8_t   i8_arr[4];
        uint8_t  u8;
        int8_t   i8;
        uint16_t u16;
        int16_t  i16;
        uint32_t u32;
        int32_t  i32;
        float    f32;
    };
};

typedef void (*serialIface)(void* iface, uint8_t* buff, uint16_t len);

// This class is basically pure state
struct MotorControllerSettings_t {
    
// define a enum class for communicating between the host computer and the 
// microcontroller
};

class MotorControllerInterface {
public:
    // disallow default initilization and copying
    MotorControllerInterface() = delete;
    MotorControllerInterface(const MotorControllerInterface &cpy) = delete;
    MotorControllerInterface operator=(const MotorControllerInterface &rhs) = delete;

    MotorControllerInterface(MotorControllerSettings_t &Settings);

    /** 
     * Send the current value of one of the motor controller settings to the host PC
     * The input to the function is the parameter that should be transmitted
     */
    void transmit_setting(MotorControllerParameter_t param);

    /**
     * Change one of the motor controller settings based on a packet (assumed to have
     * been recieved from the host computer).
     */
    void change_setting(MotorControllerPacket_t setting);

    /**
     * This function is meant to be called from the interrupt service routine for
     * the serial device in stm32f3xx_it.c.
     */
    void recieve_packet(MotorControllerPacket_t &packet);

private:
    void transmit_packet(const MotorControllerPacket_t);
};

#endif // SERIAL_INTERFACE_H_