/** TMC4671Interface 
 * \author Samuel Ellicott 
 * 
 * This class is designed as a "simple" interface to the Trinamic TMC4671 Chip
 * The chip is initilized (or reconfigured) via the 
 * \ref MotorControllerValues_t struct from ComputerInterface.h. 
 */

#ifndef _TMC4671_INTERFACE_H
#define _TMC4671_INTERFACE_H

#include <cstdint>
#include <stm32f3xx_hal.h>

// forward struct/enum declarations
enum class MotorDirection_t : uint8_t;
enum class ControlMode_t : uint8_t;
struct TMC4671Settings_t;

class TMC4671Interface {
public:
    /**
     * Setup the TMC4671 based on the perameters provided in the Settings 
     * struct. 
     */
    TMC4671Interface(TMC4671Settings_t *settings);


    /**
     * Reinitilize the TMC4671, the same as the constructor, provided so
     * that a settings change does not necessitate the reinitilization of
     * an object.
     */
    void change_settings(const TMC4671Settings_t *settings);

    /**
     * Selects from Velocity, Torque, or open loop modes of operation.
     * This immediately sends a command to change modes to the 4671,
     * so you should immediately follow it up with a call to set_setpoint.
     * 
     * This should probably only be called when the setpoint is zero,
     * but it will probably work otherwise.
     */
    void set_control_mode(ControlMode_t mode); 

    /**
     * Sets the motor direction either forward or reverse. 
     * 
     * This immediately reverses the direction, the only limit to the
     * speed is the current limit set by the MotorControllerValues_t struct,
     * should probably only be changed when the setpoint is at or near zero.
     * 
     * Luckily, an electric vehicle shouldn't be changing directions
     * very often :-)
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
    void set_setpoint(std::uint32_t set_point);

    /**
     * Enable the TMC4671 (turns on outputs to the power stage)
     * This function sets the enable pin on the microcontroller which
     * connects to the enable pin on the TMC4671.
     */
    void enable();

    /**
     * Disable the TMC4671 (turns off the outputs to the power stage)
     * This function resets the enable pin on the microcontroller which
     * connects to the enable pin on the TMC4671.
     */
    void disable();


    /**
     * 
     */
    float get_motor_current();

    float get_battery_current();

    float get_battery_voltage();

    std::int32_t get_motor_RPM();

    // disallow default initilization and copying
    TMC4671Interface() = delete;
    TMC4671Interface(const TMC4671Interface &cpy) = delete;
    TMC4671Interface operator=(const TMC4671Interface &rhs) = delete;

private:
    static void hall_effect_init(const TMC4671Settings_t &motor_settings);
    static void adc_init();
    static void pwm_init();

    // constant ADC settings

    // constants for the Trinamic Power board
    // const static uint16_t ADC_PHASE1_SCALE = 105/2;
    // const static uint16_t ADC_PHASE2_SCALE = 99/2;
    // const static int16_t ADC_PHASE1_OFFSET = 33548;
    // const static int16_t ADC_PHASE2_OFFSET = 33587;

    // constants for Isaac's power board
    const static uint16_t ADC_PHASE1_SCALE = 315/2;
    const static uint16_t ADC_PHASE2_SCALE = 315/2;
    const static int16_t ADC_PHASE1_OFFSET = 33494;
    const static int16_t ADC_PHASE2_OFFSET = 33572;

    // Deal with the error in the 4671 datasheet 
    // (0-2v input instead of a 0-5v input)

    // ADC settings for the battery current
    const static uint16_t BATTERY_CURRENT_OFFSET = 52050;
    constexpr static float BATTERY_CURRENT_SCALE  = 1.05;

    const static uint16_t BATTERY_VOLTAGE_OFFSET = 35160;
    const static uint16_t BATTERY_ADC_MAX        = 61848;
    const static uint16_t BATTERY_ADC_MIN        = 35168;
    constexpr static float BATTERY_MAX           = 69.5;


    /// pointer to a settings struct
    TMC4671Settings_t *Settings;

    /// Class state for the direction of the motor
    MotorDirection_t Direction;

    /// Class state for the setpoint for the controller.
    std::int32_t Setpoint;

    /// Class state for the control mode (Velocity, Torque, or Open-loop)
    ControlMode_t ControlMode;

    /// Motor constant for the motor 
    ///(multiplication factor for setting the setpoint in velocity mode)
    std::uint16_t MotorConstant;

    // default values for the tmc4671 on startup
    const static std::uint32_t tmc4671Registers[];
};

#endif // _TMC4671_INTERFACE_H