/** ComputerInterface
 * \author Samuel Ellicott 
 * 
 * This class and structures are designed as an interface between the host PC
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

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

// wrapper for ComputerInterface::add_to_buffer()
void computerInterface_update_buffer(void* comp_iface, const uint8_t *buff, uint32_t len);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include <array>

#ifndef PC_INTERFACE
#define PC_INTERFACE UART
#endif

enum class MotorControllerParameter_t : std::uint8_t {
    // general motor settings
    MOTOR_DIRECTION = 0,
    MOTOR_MODE,                     /// Control mode (Velocity, Torque, or Open-loop)
    SETPOINT,                       /// The setpoint for the motor (RPM for velocity and open-loop, mA for Torque)

    // motor current, velocity, and acceleration limits
    CURRENT_LIMIT,                  /// Sets the limit on the maximum current avalible to the motor
    VELOCITY_LIMIT,                 /// Sets the maximum RPM of the motor
    ACCELERATION_LIMIT,             /// Sets the the maximum acceleration of the motor RPM/sec

    // motor setup fields
    MOTOR_TYPE,                     /// what type of motor is connected, 0 for BLDC, 1 for brushed
    POLE_PAIRS_KV,                  /// number of poles in a BLDC motor or the Motor constant for a brushed motor
    
    // hall effect sensor setup
    HALL_POLARITY,                  /// Is rising or falling edge of the hall effect sensor is the correct one
    HALL_INTERPOLATE,               /// After a certain RPM, should the readings be interpolated
    HALL_DIRECTION,                 /// Are the the sensors direction 180 degrees from the motor phases
    HALL_MECH_OFFSET,               /// Hall effect sensors offset from the mechanical angle
    HALL_ELEC_OFFSET,               /// Hall effect sensors offset from the electrial angle 

    // open loop settings
    OPEN_LOOP_MODE_ENABLED,         /// startup in open loop mode 
    OPEN_LOOP_TRANSITION_VELOCITY,  /// RPM to transition from open loop to closed loop mode
    OPEN_LOOP_ACCELERATION,         /// Acceleration in RPM/s
    OPEN_LOOP_MAX_I,                /// Max current in mili-Amps
    OPEN_LOOP_MAX_V,                /// Max voltage in Volts
};

enum class MotorType_t : std::uint8_t {
    BLDC_MOTOR = 3,
    BRUSHED_MOTOR = 1 
};

enum class MotorDirection_t : std::uint8_t {
    FORWARD = 0,
    REVERSE
};

enum class ControlMode_t: std::uint8_t {
    TORQUE = 0,
    VELOCITY,
    OPEN_LOOP
};

struct MotorControllerPacket_t {
    uint8_t rw_address;
    union {
        std::uint8_t  u8_arr[4];
        std::int8_t   i8_arr[4];
        std::uint8_t  u8;
        std::int8_t   i8;
        std::uint16_t u16;
        std::int16_t  i16;
        std::uint32_t u32;
        std::int32_t  i32;
        float         f32;
    };
};

// This class is basically pure state
struct MotorControllerSettings_t {
    MotorDirection_t    MotorDir;
    ControlMode_t       ControlMode;
    std::int32_t        Setpoint;

    std::uint16_t       CurrentLimit;
    std::uint32_t       VelocityLimit;
    std::uint32_t       AccelerationLimit;

    struct {
        std::uint8_t    HallPolarity    : 1;
        std::uint8_t    HallInterpolate : 1;
        std::uint8_t    HallDirection   : 1;
    } HallMode;

    std::int16_t        HallMechOffset;
    std::int16_t        HallElecOffset;

    MotorType_t         MotorType;
    std::uint8_t        PolePairs_KV;

    std::uint8_t        OpenStartup;
    std::uint16_t       OpenTransistionVel;
    std::uint16_t       OpenAccel;
    std::uint16_t       OpenVel;
    std::uint32_t       OpenMaxI;
    std::uint16_t       OpenMaxV;
};

class ComputerInterface {
public:
    ComputerInterface(MotorControllerSettings_t *Settings_);

    /** 
     * Send the current value of one of the motor controller settings to the host PC
     * The input to the function is the parameter that should be transmitted
     */
    void transmit_setting(MotorControllerParameter_t param) const;

    /**
     * Change one of the motor controller settings based on a packet (assumed to have
     * been recieved from the host computer).
     */
    void change_setting(MotorControllerPacket_t &packet);
    /**
     * This function is meant to be called from the interrupt service routine for
     * the serial device in stm32f3xx_it.c or usb_cdc_if.c.
     */
    void recieve_packet(MotorControllerPacket_t &packet);

    //MotorControllerPacket_t parse_command

    void add_to_buffer(const std::uint8_t* buff, uint32_t len);

    void parse_command();

    void display_settings();

    // disallow default initilization and copying
    ComputerInterface() = delete;
    ComputerInterface(const ComputerInterface &cpy) = delete;
    ComputerInterface operator=(const ComputerInterface &rhs) = delete;
private:
    void transmit_packet(const MotorControllerPacket_t &packet) const;

    // uses the rw_address in packet to determine the parameter to be copied. if toSettings is true
    // then the value is copied from packet into settings, otherwise it is copied from settings into packet
    static void copy_setting(MotorControllerPacket_t &packet, MotorControllerSettings_t &settings, bool toSettings);

    MotorControllerSettings_t* Settings;
    std::array<char, 64> command_buff = {0};
    std::uint8_t command_len = 0;
};
#endif // __cplusplus
#endif // SERIAL_INTERFACE_H_