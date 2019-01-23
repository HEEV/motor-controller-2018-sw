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

#ifndef PC_INTERFACE
#define PC_INTERFACE UART
#endif

enum class MotorControllerParameter_t : std::uint8_t {
    // general motor settings
    MOTOR_DIRECTION = 0,
    // open loop settings
    OPEN_LOOP_MODE_ENABLED,         /// startup in open loop mode 
    OPEN_LOOP_TRANSITION_VELOCITY,  /// RPM to transition from open loop to closed loop mode
    OPEN_LOOP_ACCELERATION,         /// Acceleration in RPM/s
    OPEN_LOOP_VELOCITY,             /// Velocity in RPM
    OPEN_LOOP_MAX_I,                /// Max current in mili-Amps
    OPEN_LOOP_MAX_V,                /// Max voltage in Volts
};

struct MotorControllerPacket_t {
    uint8_t rw_address;
    union data {
        std::uint8_t  u8_arr[4];
        std::int8_t   i8_arr[4];
        std::uint8_t  u8;
        std::int8_t   i8;
        std::uint16_t u16;
        std::int16_t  i16;
        std::uint32_t u32;
        std::int32_t  i32;
        float         f32;
    } data;
};

// This class is basically pure state
struct MotorControllerSettings_t {
    std::uint8_t    MotorDir;

    std::uint8_t    OpenStartup;
    std::uint16_t   OpenTransistionVel;
    std::uint16_t   OpenAccel;
    std::uint16_t   OpenVel;
    std::uint32_t   OpenMaxI;
    std::uint16_t   OpenMaxV;
};

class MotorControllerInterface {
public:
    // disallow default initilization and copying
    MotorControllerInterface() = delete;
    MotorControllerInterface(const MotorControllerInterface &cpy) = delete;
    MotorControllerInterface operator=(const MotorControllerInterface &rhs) = delete;

    MotorControllerInterface(MotorControllerSettings_t &Settings_);

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
    void transmit_packet(const MotorControllerPacket_t &packet);

    MotorControllerSettings_t* Settings;
};

#endif // SERIAL_INTERFACE_H_