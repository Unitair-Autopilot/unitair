/** @file
    3-axis gyroscope

    datasheet: https://www.nxp.com/docs/en/data-sheet/FXAS21002.pdf
*/

#pragma once

#include <memory>
#include <Eigen/Dense>

#include <hardware/common/i2c.h>

namespace unitair {
namespace hardware {


class FXAS21002 {

    private:

        std::unique_ptr<I2C> _i2c_interface;
        //device registers
        const uint8_t _I2C_ADDRESS = 0x21;
        const uint8_t _DEVICE_ID = 0xD7;
        const uint8_t _FIRST_DATA_REGISTER = 0x01;
        const uint8_t _WHO_AM_I_REGISTER = 0x0C;
        const uint8_t _CTRL0_REGISTER = 0x0D;
        const uint8_t _CTRL1_REGISTER = 0x13;

        //device specific constant
        const double _GYRO_SENSITIVITY_250DPS  = 0.0078125;
        const double _GYRO_SENSITIVITY_500DPS  = 0.015625;
        const double _GYRO_SENSITIVITY_1000DPS = 0.03125;
        const double _GYRO_SENSITIVITY_2000DPS = 0.0625;
        double _GYRO_SENSITIVITY; //determined at run time by device configuration

    public:

        FXAS21002(std::string i2c_bus_path);
        ~FXAS21002();

        //! Configure device
        /*!
            \param[in] gyro_sensitivity desired gyroscope sensitivity [+/- deg/s] valid options = 250, 500, 1000, 2000
            \return True = configure successful, False = configure failed
        */
        bool configure(int gyro_sensitivity);

        struct return_data_t{
            Eigen::Vector3d gyro;
        };
        //! Read data from device
        /*!
            \return return_data_t
        */
        return_data_t read_data();
};


} // namespace hardware
} // namespace unitair
