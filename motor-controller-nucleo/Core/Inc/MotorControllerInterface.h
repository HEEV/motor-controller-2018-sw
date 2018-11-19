#ifndef SERIAL_INTERFACE_H_
#define SERIAL_INTERFACE_H_

#include <cstdint>
#include "MotorControllerSettings.h"

using uint8_t  = std::uint8_t;
using int8_t   = std::int8_t;
using uint32_t = std::uint32_t;

struct MotorControllerPacket {
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
    }
};

class MotorControllerInterface {
public:
    // disallow default initilization and copying
    MotorContrllerInterface() = delete;
    MotorControllerInterface(const MotorContrllerInterface &cpy) = delete;
    MotorContrllerInterface & operator=(const MotorContrllerInterface &rhs) = delete;

    MotorControllerInterface(MotorControllerSettingsManager &Settings);

    void transmit_setting(MotorControllerSetting setting);
    void change_setting(MotorControllerPacket setting);

private:
    static void recieve_packet(MotorControllerPacket &packet);
    static void transmit_packet(const MotorControllerPacket)
};

#endif // SERIAL_INTERFACE_H_