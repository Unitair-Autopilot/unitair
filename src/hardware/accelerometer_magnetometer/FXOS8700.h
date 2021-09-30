/** @file
    6-axis sensor with integrated linear accelerometer and magnetometer

    datasheet: https://www.nxp.com/docs/en/data-sheet/FXOS8700CQ.pdf
*/

#pragma once

#include <memory>
#include <Eigen/Dense>

#include <hardware/common/i2c.h>

namespace unitair {
namespace hardware {


class FXOS8700 {

    private:

        std::unique_ptr<I2C> _i2c_interface;
        //device registers
        const uint8_t _I2C_ADDRESS = 0x1F;
        const uint8_t _DEVICE_ID = 0xC7;
        const uint8_t _FIRST_DATA_REGISTER = 0x01;
        const uint8_t _WHO_AM_I_REGISTER = 0x0D;
        const uint8_t _XYZ_DATA_CFG_REGISTER = 0x0E;
        const uint8_t _CTRL1_REGISTER = 0x2A;
        const uint8_t _CTRL2_REGISTER = 0x2B;
        const uint8_t _MAG_CTRL1_REGISTER = 0x5B;
        const uint8_t _MAG_CTRL2_REGISTER = 0x5C;

        // device specific conversions
        const double _ACCEL_G_LSB_2G = 0.000244; //g per LSB at +/- 2g
        const double _ACCEL_G_LSB_4G = 0.000488; //g per LSB at +/- 4g
        const double _ACCEL_G_LSB_8G = 0.000976; //g per LSB at +/- 8g
        double _ACCEL_MG_LSB; //determined at run time by device configuration
        const double _MAG_UT_LSB = 0.1; //micro tesla (uT) per LSB

        const double _GRAVITY_ON_EARTH = 9.80665; //[m/s^2]

    public:

        FXOS8700(std::string i2c_bus_path);
        ~FXOS8700();

        //! Configure device
        /*!
            \param[in] accel_sensitivity desired accelerometer sensitivity [+/- g]. Valid options = 2, 4, 8
            \return True = configure successful, False = configure failed
        */
        bool configure(int accel_sensitivity);

        struct return_data_t{
            Eigen::Vector3d accel;
            Eigen::Vector3d mag;
        };
        //! Read data from device
        /*!
            \return return_data_t
        */
        return_data_t read_data();
};


} // namespace hardware
} // namespace unitair
