/** @file
    3-axis gyroscope

    datasheet: https://www.nxp.com/docs/en/data-sheet/FXAS21002.pdf
*/

#include <iostream>
#include <cmath>

#include <hardware/gyroscope/FXAS21002.h>

namespace unitair {
namespace hardware {


FXAS21002::FXAS21002(std::string i2c_bus_path){

    _i2c_interface = std::make_unique<I2C>(i2c_bus_path, _I2C_ADDRESS);
}


FXAS21002::~FXAS21002(){

}


bool FXAS21002::configure(int gyro_sensitivity){

    //check device id
    uint8_t rx_buffer[1];
    if(not _i2c_interface->read_data(_WHO_AM_I_REGISTER, rx_buffer, 1)){
        std::cerr << "GYRO: Could not read device ID register" << std::endl;
        return false;
    }
    else if(rx_buffer[0] != _DEVICE_ID){
        std::cerr << "GYRO: Device ID does not match expected. Received: " <<
                                                                +rx_buffer[0] <<
                                                                " Expected: " <<
                                                                +_DEVICE_ID << std::endl;
        return false;
    }

    uint8_t tx_buffer[1];
    //set device in standby mode (required to write to certain registers)
    tx_buffer[0] = 0x00;
    if(not _i2c_interface->write_data(_CTRL1_REGISTER, tx_buffer, 1)){
        std::cerr << "GYRO: Failed to set device into standby mode" << std::endl;
        return false;
    }
    //set gyro full scale range
    switch(gyro_sensitivity){
        case(250):
            tx_buffer[0] = 0x03;
            _GYRO_SENSITIVITY = _GYRO_SENSITIVITY_250DPS;
            break;
        case(500):
            tx_buffer[0] = 0x02;
            _GYRO_SENSITIVITY = _GYRO_SENSITIVITY_500DPS;
            break;
        case(1000):
            tx_buffer[0] = 0x01;
            _GYRO_SENSITIVITY = _GYRO_SENSITIVITY_1000DPS;
            break;
        case(2000):
            tx_buffer[0] = 0x00;
            _GYRO_SENSITIVITY = _GYRO_SENSITIVITY_2000DPS;
            break;
        default:
            std::cerr << "GYRO: Invalid gyro sensitivity specified" << std::endl;
            return false;
    }
    tx_buffer[0] = 0x02;
    if(not _i2c_interface->write_data(_CTRL0_REGISTER, tx_buffer, 1)){
        std::cerr << "GYRO: Failed to write to CTRL_0 register" << std::endl;
        return false;
    }
    //set device to "active" mode
    tx_buffer[0] = 0x0E;
    if(not _i2c_interface->write_data(_CTRL1_REGISTER, tx_buffer, 1)){
        std::cerr << "GYRO: Failed to set device into active mode" << std::endl;
        return false;
    }

    return true;
}


FXAS21002::return_data_t FXAS21002::read_data(){

    return_data_t data;

    uint8_t rx_buffer[6];
    uint16_t data_buffer;

    //see datasheet for device register descriptions/layout
    if(_i2c_interface->read_data(_FIRST_DATA_REGISTER, rx_buffer, 6)){
        data_buffer = rx_buffer[0];
        data.gyro.x() = (int16_t)(data_buffer << 8) | rx_buffer[1];
        data_buffer = rx_buffer[2];
        data.gyro.y() = (int16_t)(data_buffer << 8) | rx_buffer[3];
        data_buffer = rx_buffer[4];
        data.gyro.z() = (int16_t)(data_buffer << 8) | rx_buffer[5];
    }
    else{
        std::cerr << "GYRO: Data register read failed" << std::endl;
    }

    //convert to SI units
    data.gyro.x() *= _GYRO_SENSITIVITY * (M_PI/180.0);
    data.gyro.y() *= _GYRO_SENSITIVITY * (M_PI/180.0);
    data.gyro.z() *= _GYRO_SENSITIVITY * (M_PI/180.0);

    return data;
}


} // namespace hardware
} // namespace unitair
