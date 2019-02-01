/** TMC4671Interface 
 * \author Samuel Ellicott 
 * 
 * This class is designed as a "simple" interface to the Trinamic TMC4671 Chip
 * The chip is initilized (or reconfigured) via the 
 * \ref MotorControllerSettings_t struct from ComputerInterface.h. 
 */

#ifndef _TMC4671_INTERFACE_H
#define _TMC4671_INTERFACE_H

#include <cstdint>
#include "ComputerInterface.h"

class TMC4671Interface {
    /**
     * Setup the TMC4671 based on the perameters provided in the Settings 
     * struct. 
     */
    TMC4671Interface(const MotorControllerSettings_t *Settings_);

    /**
     * Reinitilize the TMC4671, the same as the constructor, provided so
     * that a settings change does not necessitate the reinitilization of
     * an object.
     */
    void change_settings(const MotorControllerSettings_t *Settings_);

    /**
     * Selects from Velocity, Torque, or
     * open loop modes of operation.
     */
    void set_control_mode(MotorMode_t mode); 

    /**
     * Sets the motor direction either forward or reverse. 
     */
    void set_direction(MotorDirection_t dir);

    /**
     * Provide a setpoint for any of the three modes of operation
     * Note that all negative values will be interperted as zero,
     * to change direction, use the set_direction function.
     * 
     * Meanings for various modes
     * Velocity mode:   set_point -> RPM
     * Torque mode:     set_point -> Motor Current (mA)  
     * Open loop mode:  ???
     */
    void set_setpoint(std::int32_t set_point);

    // disallow default initilization and copying
    TMC4671Interface() = delete;
    TMC4671Interface(const TMC4671Interface &cpy) = delete;
    TMC4671Interface operator=(const TMC4671Interface &rhs) = delete;
};

#endif // _TMC4671_INTERFACE_H