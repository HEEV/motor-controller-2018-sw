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

enum class MotorControllerParameter_t : std::uint8_t {
    // general motor settings
    MOTOR_DIRECTION = 0,
    MOTOR_TYPE,                     /// what type of motor is connected, 0 for BLDC, 1 for brushed
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
    std::uint8_t    MotorType;
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

    void init_tmc4671(uint8_t motor_type = 3, uint16_t pole_pairs = 14);

    MotorControllerSettings_t* Settings;
    
    // default values for the tmc4671 on startup
    const static std::uint32_t tmc4671Registers[128]  = 
        0x00000000, //0x00: (read only)
        0x00000000, //0x01:
        0x00000000, //0x02: (read only)
        0x00000000, //0x03: 
        0x00100010, //0x04: 
        0x20000000, //0x05: 
        0x00000000, //0x06: 
        0x014E014E, //0x07: 
        0x01008001, //0x08: 
        0x01008001, //0x09: 
        0x18000100, //0x0A: 
        0x00000000, //0x0B: 
        0x00044400, //0x0C: 
        0x01000000, //0x0D: 
        0x01000000, //0x0E: 
        0x01000000, //0x0F: 
        0x00000000, //0x10: (reserved)
        0x03020100, //0x11: 
        0x00000000, //0x12: (read only)
        0x00000000, //0x13: (read only)
        0x00000000, //0x14: (read only)
        0x00000000, //0x15: (read only)
        0x00000000, //0x16: (read only)
        0x00000000, //0x17: 
        0x00000F9F, //0x18: 
        0x00000505, //0x19: 
        0x00000007, //0x1A: 
        0x0003000E, //0x1B: 
        0x00000000, //0x1C: 
        0x00000000, //0x1D: 
        0x00000000, //0x1E: 
        0x00000000, //0x1F: 
        0x0000003C, //0x20: 
        0xFFFFFFF6, //0x21: 
        0xFFFFFFFB, //0x22: 
        0x00008BAD, //0x23: 
        0x00000678, //0x24: 
        0x00000000, //0x25: 
        0x00010000, //0x26: 
        0x002641C9, //0x27: 
        0x002641C9, //0x28: 
        0x00000000, //0x29: 
        0x00000000, //0x2A: (read only)
        0x00000000, //0x2B: (reserved)
        0x00000000, //0x2C: 
        0x00010000, //0x2D: 
        0x0025DAC7, //0x2E: 
        0x0025DAC7, //0x2F: 
        0x00000000, //0x30: 
        0x00000000, //0x31: (read only)
        0x00000000, //0x32: (reserved)
        0x00001101, //0x33: 
        0x2AAA0000, //0x34: 
        0x80005555, //0x35: 
        0xD555AAAA, //0x36: 
        0x04000000, //0x37: 
        0x00002AAA, //0x38: 
        0x00000000, //0x39: (read only)
        0x00000000, //0x3A: (read only)
        0x00000000, //0x3B: 
        0x00000000, //0x3C: 
        0x00000000, //0x3D: (read only)
        0x00000000, //0x3E: 
        0x00000000, //0x3F: (read only)
        0x00000001, //0x40: 
        0x00000000, //0x41: (read only)
        0x00000000, //0x42: 
        0x00000000, //0x43: (reserved)
        0x00000000, //0x44: (reserved)
        0x00000000, //0x45: 
        0x00000000, //0x46: (read only)
        0x00000000, //0x47: (read only)
        0x00000000, //0x48: (reserved)
        0x00000000, //0x49: (reserved)
        0x00000000, //0x4A: (reserved)
        0x00000000, //0x4B: (read only)
        0x00000000, //0x4C: (read only)
        0x00000000, //0x4D: 
        0x00000000, //0x4E: 
        0x00000000, //0x4F: (reserved)
        0x00000000, //0x50: 
        0x00000000, //0x51: 
        0x00000005, //0x52: 
        0x00000000, //0x53: (read only)
        0x012C0100, //0x54: 
        0x00000000, //0x55: (reserved)
        0x01000100, //0x56: 
        0x00000000, //0x57: (reserved)
        0x02000100, //0x58: 
        0x00000000, //0x59: (reserved)
        0x00000000, //0x5A: 
        0x00000000, //0x5B: (reserved)
        0x00007FFF, //0x5C: 
        0x00005A81, //0x5D: 
        0x00000FA0, //0x5E: 
        0x00000BB8, //0x5F: 
        0x00001F40, //0x60: 
        0x80000001, //0x61: 
        0x7FFFFFFF, //0x62: 
        0x00000008, //0x63: 
        0x05DC0000, //0x64: 
        0x00000000, //0x65: 
        0x00000000, //0x66: 
        0x00000000, //0x67: 
        0x365A40D8, //0x68: 
        0x00000000, //0x69: (read only)
        0x00000000, //0x6A: (read only)
        0x365A40D8, //0x6B: 
        0x00000000, //0x6C: (read only)
        0x00000000, //0x6D: 
        0x000000D2, //0x6E: 
        0x00000012, //0x6F: 
        0x00000000, //0x70: (reserved)
        0x00000000, //0x71: (reserved)
        0x00000000, //0x72: (reserved)
        0x00000000, //0x73: (reserved)
        0x00000000, //0x74: 
        0xFFFFFFFF, //0x75: 
        0x00000000, //0x76: (read only)
        0x00000000, //0x77: (read only)
        0x00000000, //0x78: 
        0x00009600, //0x79: 
        0x00000000, //0x7A: 
        0x00000000, //0x7B: 
        0xC4788080, //0x7C: 
        0x00000000, //0x7D: 
    };
};

#endif // SERIAL_INTERFACE_H_