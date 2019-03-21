#ifndef SETTINGS_STRUCTS_H
#define SETTINGS_STRUCTS_H

#include <cstdint>

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

struct TMC4671Settings_t {
    MotorDirection_t    MotorDir;
    ControlMode_t       ControlMode;
    std::int32_t        Setpoint;

    std::uint16_t       CurrentLimit;
    std::uint32_t       VelocityLimit;
    std::uint32_t       AccelerationLimit;

    MotorType_t         MotorType;
    std::uint8_t        PolePairs_KV;

    struct {
        std::uint8_t    HallPolarity    : 1;
        std::uint8_t    HallInterpolate : 1;
        std::uint8_t    HallDirection   : 1;
    } HallMode;

    std::int16_t        HallMechOffset;
    std::int16_t        HallElecOffset;

    std::uint16_t       OpenAccel;
    std::uint16_t       OpenVel;
    std::uint32_t       OpenMaxI;
    std::uint16_t       OpenMaxV;
};

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
    OPEN_LOOP_ACCELERATION,         /// Acceleration in RPM/s
    OPEN_LOOP_MAX_I,                /// Max current in mili-Amps
    OPEN_LOOP_MAX_V,                /// Max voltage in Volts

    // settings that don't correspond to actual settings
    HALL_AUTO_SETUP,
    SAVE_SETTINGS,
    NO_ACTION
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
    TMC4671Settings_t tmc4671;
};
#endif //SETTINGS_STRUCTS_H