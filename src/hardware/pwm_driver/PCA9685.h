/** @file
    16-channel, 12-bit PWM driver

    datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf
*/

#pragma once

#include <memory>
#include <Eigen/Dense>
#include <vector>

#include <hardware/common/i2c.h>

namespace unitair {
namespace hardware {


class PCA9685 {

    private:

        std::unique_ptr<I2C> _i2c_interface;
        //device registers
        const uint8_t _I2C_ADDRESS = 0x40;
        const uint8_t _MODE1_REGISTER = 0x00;
        const uint8_t _MODE2_REGISTER = 0x01;
        const uint8_t _FIRST_OUTPUT_REGISTER = 0x06;
        const uint8_t _PRESCALE_REGISTER = 0xFE;

        //device specific constants
        const uint8_t _CLOCK_SPEED_MHZ = 25; // internal oscillator
        const uint8_t _PRESCALE_MIN = 3;
        const uint8_t _PRESCALE_MAX = 255;

    public:

        //constant parameters
        const uint8_t _NUM_CHANNELS = 16;
        int _pwm_frequency;

        //dynamic parameters
        std::vector<std::vector<double>> _output_trims;

        PCA9685(std::string i2c_bus_path);
        ~PCA9685();

        //! Configure device
        /*!
            \param[in] pwm_frequency desired pwm frequency [hz]
        */
        bool configure(int pwm_frequency);

        //! Update pwm outputs
        /*!
            \param[in] actuator_commands vector of desired actuator commands
        */
        bool update_outputs(std::vector<double> actuator_commands);

        //! Update duty cycles
        /*!
            \param[in] duty_cycles vector of desired duty cycles
        */
        bool update_duty_cycles(std::vector<double> duty_cycles);
};


} // namespace hardware
} // namespace unitair
