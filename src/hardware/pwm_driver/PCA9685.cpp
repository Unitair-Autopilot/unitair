/** @file
    16-channel, 12-bit PWM driver

    datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf
*/

#include <iostream>
#include <unistd.h>
#include <cmath>

#include "tools/tools.hpp"

#include "hardware/pwm_driver/PCA9685.h"

namespace unitair {
namespace hardware {


PCA9685::PCA9685(std::string i2c_bus_path){

    _i2c_interface = std::make_unique<I2C>(i2c_bus_path, _I2C_ADDRESS);
}


PCA9685::~PCA9685(){
}


bool PCA9685::configure(int pwm_frequency){

    uint8_t tx_buffer[1];

    //put device to sleep (to allow certain config register writes)
    tx_buffer[0] = 0x11;
    if(not _i2c_interface->write_data(_MODE1_REGISTER, tx_buffer, 1)){
        std::cerr << "PWM: Failed to set device to sleep mode" << std::endl;
        return false;
    }

    //set prescale value to set pwm frequency
    uint8_t prescale = (_CLOCK_SPEED_MHZ*1e6)/(4096*pwm_frequency) - 1;
    tx_buffer[0] = std::max(_PRESCALE_MIN, std::min(prescale, _PRESCALE_MAX));
    if(not _i2c_interface->write_data(_PRESCALE_REGISTER, tx_buffer, 1)){
        std::cerr << "PWM: Failed to set PRESCALE reigster" << std::endl;
        return false;
    }
    _pwm_frequency = pwm_frequency;

    //wake device up
    tx_buffer[0] = 0x01;
    if(not _i2c_interface->write_data(_MODE1_REGISTER, tx_buffer, 1)){
        std::cerr << "PWM: Failed to set device out of sleep mode" << std::endl;
        return false;
    }

    sleep(1); //wait for oscillator to come up

    //enable auto increment on register reads and clear restart bit
    tx_buffer[0] = 0xA0;
    if(not _i2c_interface->write_data(_MODE1_REGISTER, tx_buffer, 1)){
        std::cerr << "PWM: Failed to set device out of sleep mode" << std::endl;
        return false;
    }

    //enable totem pole push/pull output
    tx_buffer[0] = 0x04;
    if(not _i2c_interface->write_data(_MODE2_REGISTER, tx_buffer, 1)){
        std::cerr << "PWM: Failed to set MODE2 register" << std::endl;
        return false;
    }

    return true;
}


bool PCA9685::update_outputs(std::vector<double> actuator_commands){

    if(actuator_commands.size() > _NUM_CHANNELS){
        std::cerr << "PWM: Input vector is a larger than number of channels on device, not updating pwm outputs" << std::endl;
        return false;
    }

    std::vector<double> duty_cycles(actuator_commands.size());
    uint8_t channel_index = 0;
    for(auto& actuator_command: actuator_commands){

        if((channel_index >= _output_trims.size()) or (_output_trims[channel_index].size() != 3)){
            duty_cycles[channel_index] = 0.0;
        }
        else{
            // Piecewise-linear mapping from commanded effort to desired pwm duty-cycle
            double slope;
            if(actuator_command >= 0){
                slope = _output_trims[channel_index][2] - _output_trims[channel_index][1];
            }
            else{
                slope = _output_trims[channel_index][1] - _output_trims[channel_index][0];
            }
            duty_cycles[channel_index] = (slope * actuator_command) + _output_trims[channel_index][1];
        }

        duty_cycles[channel_index] = tools::clip(duty_cycles[channel_index], 0.0, 1.0);

        channel_index++;
    }

    return update_duty_cycles(duty_cycles);
}


bool PCA9685::update_duty_cycles(std::vector<double> duty_cycles){

    if(duty_cycles.size() > _NUM_CHANNELS){
        std::cerr << "PWM: Input vector is a larger than number of channels on device, not updating duty cycle" << std::endl;
        return false;
    }

    uint8_t tx_buffer[4 * duty_cycles.size()];
    uint8_t i = 0;
    for(auto& duty_cycle: duty_cycles){
        //LEDX_ON registers always zero for now (could be useful for phase shifting if needed)
        tx_buffer[i] = 0x00;
        tx_buffer[i+1] = 0x00;

        if(duty_cycle > 1.0 or duty_cycle < 0.0){
            std::cerr << "PWM: Invalid duty cycle specified, specified " <<
                                duty_cycle <<
                                "setting value to 0" << std::endl;
            //LEDX_OFF registers
            tx_buffer[i+2] = 0x00;
            tx_buffer[i+3] = 0x00;
        }
        else{
            uint16_t on_count = 4095 * duty_cycle;
            //LEDX_OFF registers
            tx_buffer[i+2] = (on_count) & 0xFF;
            tx_buffer[i+3] = (on_count >> 8) & 0xFF;
        }
        i += 4;
    }

    if(not _i2c_interface->write_data(_FIRST_OUTPUT_REGISTER, tx_buffer, 4 * duty_cycles.size())){
        std::cerr << "PWM: Could not write to output registers" << std::endl;
        return false;
    }

    return true;
}


} // namespace hardware
} // namespace unitair
