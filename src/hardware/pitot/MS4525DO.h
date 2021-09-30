/** @file
    Airspeed sensor with 14 bit digital pressure and 11 bit temperature reading capabilities

    datasheet: https://www.te.com/usa-en/product-CAT-BLPS0002.datasheet.pdf
*/

#pragma once

#include <memory>
#include <string>

#include "hardware/common/i2c.h"

namespace unitair{
namespace hardware{

class MS4525DO {
    public:
        MS4525DO(const std::string& i2c_bus_path);
        MS4525DO(const MS4525DO&) = delete;
        MS4525DO& operator=(const MS4525DO&) = delete;
        MS4525DO(MS4525DO&&) = default;
        MS4525DO& operator=(MS4525DO&&) = default;
        ~MS4525DO() = default;

        /// Returning data structure
        struct Measurement {
            bool is_valid = false;
            double differential_pressure = 0.0;  // Pascals
            double temperature = 0.0;  // degrees Celsius
        };

        /// Read data from device
        Measurement read_measurement();

    private:
        const uint8_t _MEASUREMENT_REGISTER;
        const uint8_t _I2C_ADDRESS_1;
        std::unique_ptr<I2C> _i2c_interface;
};

} // namespace hardware
} // namespace unitair
