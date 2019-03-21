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
#include "ComputerMenu.h"

#ifndef PC_INTERFACE
#define PC_INTERFACE UART
#endif

// struct and enum forward declaration
struct MotorControllerSettings_t;
struct MotorControllerPacket_t;
class TMC4671Interface;
enum class MotorControllerParameter_t : std::uint8_t;

class ComputerInterface {
public:
    ComputerInterface(MotorControllerSettings_t *Settings_, TMC4671Interface *mc_handle);

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

    void add_to_buffer(const std::uint8_t* buff, uint32_t len);


    void display_settings();

    const char* access_setting_value(char* buff, MotorControllerParameter_t param, bool write, std::int32_t value);

    // disallow default initilization and copying
    ComputerInterface() = delete;
    ComputerInterface(const ComputerInterface &cpy) = delete;
    ComputerInterface operator=(const ComputerInterface &rhs) = delete;
private:
    void println(char* buff, std::uint16_t len);
    void transmit_packet(const MotorControllerPacket_t &packet) const;
    int parse_command();


    // uses the rw_address in packet to determine the parameter to be copied. if toSettings is true
    // then the value is copied from packet into settings, otherwise it is copied from settings into packet
    static void copy_setting(MotorControllerPacket_t &packet, MotorControllerSettings_t &settings, bool toSettings);

    MotorControllerSettings_t* Settings;
    std::array<char, 64> command_buff = {0};
    std::uint8_t command_len = 0;
    ComputerMenu menu;
    TMC4671Interface* htmc4671;
};
#endif // __cplusplus
#endif // SERIAL_INTERFACE_H_