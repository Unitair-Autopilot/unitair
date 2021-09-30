/** @file
    Userspace control of I2C bus through the /dev interface
*/

#pragma once

namespace unitair {
namespace hardware {


class I2C {

    private:

        int _bus_fd;
        uint16_t _device_address;

        static constexpr uint8_t MAX_NUM_BYTES_WRITE = 128;
        uint8_t _write_buffer[MAX_NUM_BYTES_WRITE];


    public:


        //! Configure and open I2C device
        /*!
            \param[in] i2c_bus_path filepath to i2c adapater
            \param[in] device_address i2c device address
        */
        I2C(std::string i2c_bus_path, uint16_t device_address);


        //! Close I2C device
        ~I2C();


        //! Read device register
        /*!
            \param[in] register_address register address to read data from
            \param[in] data_buffer pointer to data buffer
            \param[in] num_bytes number of bytes to read from device
            \return True = read successful, False = read failed
        */
        bool read_data(uint8_t register_address, uint8_t* data_buffer, uint16_t num_bytes = 1);


        //! Write device register
        /*!
            \param[in] register_address register address to write data to
            \param[in] data_buffer pointer to data buffer
            \param[in] num_bytes number of bytes to write to device
            \return True = write successful, False = write failed
        */
        bool write_data(uint8_t register_address, uint8_t* data_buffer, uint16_t num_bytes = 1);

};


} // namespace hardware
} // namespace unitair
