/** @file
    6-axis sensor with integrated linear accelerometer and magnetometer

    datasheet: https://www.nxp.com/docs/en/data-sheet/FXOS8700CQ.pdf
*/

#include <iostream>

#include <hardware/accelerometer_magnetometer/FXOS8700.h>

namespace unitair {
namespace hardware {


FXOS8700::FXOS8700(std::string i2c_bus_path){

    _i2c_interface = std::make_unique<I2C>(i2c_bus_path, _I2C_ADDRESS);
}


FXOS8700::~FXOS8700(){

}


bool FXOS8700::configure(int accel_sensitivity){

    //check device id
    uint8_t rx_buffer[1];
    if(not _i2c_interface->read_data(_WHO_AM_I_REGISTER, rx_buffer, 1)){
        std::cerr << "ACCEL-MAG: Could not read device ID register" << std::endl;
        return false;
    }
    else if(rx_buffer[0] != _DEVICE_ID){
        std::cerr << "ACCEL-MAG: Device ID does not match expected. Received: " <<
                                                                +rx_buffer[0] <<
                                                                " Expected: " <<
                                                                +_DEVICE_ID << std::endl;
        return false;
    }

    uint8_t tx_buffer[1];
    //set device in standby mode (required to write to certain registers)
    tx_buffer[0] = 0x00;
    if(not _i2c_interface->write_data(_CTRL1_REGISTER, tx_buffer, 1)){
        std::cerr << "ACCEL-MAG: Failed to set device into standby mode" << std::endl;
        return false;
    }
    //set accelerometer full scale range
    switch(accel_sensitivity){
        case(2):
            tx_buffer[0] = 0x00;
            _ACCEL_MG_LSB = _ACCEL_G_LSB_2G;
            break;
        case(4):
            tx_buffer[0] = 0x01;
            _ACCEL_MG_LSB = _ACCEL_G_LSB_4G;
            break;
        case(8):
            tx_buffer[0] = 0x02;
            _ACCEL_MG_LSB = _ACCEL_G_LSB_8G;
            break;
        default:
            std::cerr << "ACCEL-MAG: Invalid accel sensitivity specified" << std::endl;
            return false;
    }
    if(not _i2c_interface->write_data(_XYZ_DATA_CFG_REGISTER, tx_buffer, 1)){
        std::cerr << "ACCEL-MAG: Failed to write to XYZ_DATA_CFG register" << std::endl;
        return false;
    }
    //set high resolution mode
    tx_buffer[0] = {0x02};
    if(not _i2c_interface->write_data(_CTRL2_REGISTER, tx_buffer, 1)){
        std::cerr << "ACCEL-MAG: Failed to write to CTRL_2 register" << std::endl;
        return false;
    }
    //set "active" mode, "low noise" enable, 100Hz output in "hybrid" mode
    tx_buffer[0] = {0x15};
    if(not _i2c_interface->write_data(_CTRL1_REGISTER, tx_buffer, 1)){
        std::cerr << "ACCEL-MAG: Failed to write to CTRL_1 register" << std::endl;
        return false;
    }
    //set magnetometer oversampling ratio to 16 (may need to be adjusted)
    tx_buffer[0] = {0x1F};
    if(not _i2c_interface->write_data(_MAG_CTRL1_REGISTER, tx_buffer, 1)){
        std::cerr << "ACCEL-MAG: Failed to write to MAG_CTRL_1 register" << std::endl;
        return false;
    }
    //set device to automatically jump from accel data registers to mag data registers on burst read
    tx_buffer[0] = {0x20};
    if(not _i2c_interface->write_data(_MAG_CTRL2_REGISTER, tx_buffer, 1)){
        std::cerr << "ACCEL-MAG: Failed to write to MAG_CTRL_2 register" << std::endl;
        return false;
    }

    return true;
}


FXOS8700::return_data_t FXOS8700::read_data(){

    return_data_t data;

    uint8_t rx_buffer[12];
    uint16_t data_buffer;

    //see datasheet for device register descriptions/layout
    if(_i2c_interface->read_data(_FIRST_DATA_REGISTER, rx_buffer, 12)){
        //accel
        data_buffer = rx_buffer[0];
        data.accel.x() = (int16_t)((data_buffer << 8) | rx_buffer[1]) >> 2;
        data_buffer = rx_buffer[2];
        data.accel.y() = (int16_t)((data_buffer << 8) | rx_buffer[3]) >> 2;
        data_buffer = rx_buffer[4];
        data.accel.z() = (int16_t)((data_buffer << 8) | rx_buffer[5]) >> 2;
        //mag
        data_buffer = rx_buffer[6];
        data.mag.x() = (int16_t)(data_buffer << 8) | (rx_buffer[7]);
        data_buffer = rx_buffer[8];
        data.mag.y() = (int16_t)(data_buffer << 8) | (rx_buffer[9]);
        data_buffer = rx_buffer[10];
        data.mag.z() = (int16_t)(data_buffer << 8) | (rx_buffer[11]);
    }
    else{
        std::cerr << "ACCEL-MAG: Data register read failed" << std::endl;
    }

    //convert to SI units
    data.accel.x() *= _ACCEL_MG_LSB * _GRAVITY_ON_EARTH;
    data.accel.y() *= _ACCEL_MG_LSB * _GRAVITY_ON_EARTH;
    data.accel.z() *= _ACCEL_MG_LSB * _GRAVITY_ON_EARTH;
    data.mag.x() *= _MAG_UT_LSB * 1e-6;
    data.mag.y() *= _MAG_UT_LSB * 1e-6;
    data.mag.z() *= _MAG_UT_LSB * 1e-6;

    return data;
}


} // namespace hardware
} // namespace unitair
