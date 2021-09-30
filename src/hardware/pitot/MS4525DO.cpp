/** @file
    Airspeed sensor with 14 bit digital pressure and 11 bit temperature reading capabilities

    datasheet: https://www.te.com/usa-en/product-CAT-BLPS0002.datasheet.pdf
    px4 code: https://github.com/ArduPilot/archived-PX4Firmware/blob/master/src/drivers/meas_airspeed/meas_airspeed.cpp
*/

#include "hardware/pitot/MS4525DO.h"

#include <algorithm>
#include <array>
#include <iostream>

namespace unitair {
namespace hardware {


MS4525DO::MS4525DO(const std::string& i2c_bus_path) :
    //arbitrary measure register chosen as it is not specified by datasheet
    _MEASUREMENT_REGISTER(0x00),
    //i2c address depends on the interface type selected from the ordering information
    _I2C_ADDRESS_1(0x28 /*0x36*/ /*0x46*/) {

    _i2c_interface = std::make_unique<I2C>(i2c_bus_path, _I2C_ADDRESS_1);
}


MS4525DO::Measurement MS4525DO::read_measurement(){

    // Query sensor a first time
    auto first_buffer = std::array<uint8_t, 4>{};
    if(!_i2c_interface->read_data(_MEASUREMENT_REGISTER, first_buffer.data(), first_buffer.size())) {
        std::cerr << "PITOT: First data register read failed" << std::endl;
        return {};
    }

    // Query sensor a second time
    auto second_buffer = std::array<uint8_t, 4>{};
    if(!_i2c_interface->read_data(_MEASUREMENT_REGISTER, second_buffer.data(), second_buffer.size())){
        std::cerr << "PITOT: Second data register read failed" << std::endl;
        return {};
    }

    // Check if incoming data is valid or stale
    const auto first_status = static_cast<uint8_t>(first_buffer[0] & 0xC0) >> 6;
    const auto second_status = static_cast<uint8_t>(second_buffer[0] & 0xC0) >> 6;
    const auto match_iter = std::mismatch(first_buffer.cbegin() + 1, first_buffer.cend(), second_buffer.cbegin() + 1);
    const bool buffers_are_equal = (match_iter.first == first_buffer.cend());

    //status definitions and status bits
    enum MeasurementStatus : uint8_t {
        GOOD_PACKET = 0, RESERVED_PACKED = 1, STALE_PACKET = 2, FAULTY_PACKET = 3
    };

    // Calculate differential pressure and temperature if data is vaild
    const bool data_is_valid = (first_status == MeasurementStatus::GOOD_PACKET)
                                && (second_status == MeasurementStatus::STALE_PACKET)
                                && buffers_are_equal;
    if(!data_is_valid){
        std::cerr << "PITOT: Data is not valid: (" << first_status << ", " << second_status << ")" << std::endl;
        return {};
    }

    auto return_value = Measurement{};
    return_value.is_valid = true;

    // calculate temperature
    int16_t dT_raw = static_cast<int16_t>(first_buffer[2] << 8) | first_buffer[3];
    dT_raw = (0xFFE0 & dT_raw) >> 5;
    return_value.temperature = ((200.0f * dT_raw) / 2047.0) - 50.0;

    // Calculate differential pressure. As its centered around 8000 and can go positive or negative.
    // This equation is an inversion of the equation in the pressure transfer function
    // figure on page 4 of the datasheet. We negate the result so that positive differential
    // pressures are generated when the bottom port is used as the static port on the pitot
    // and top port is used as the dynamic port
    static constexpr auto P_min = -1.0;
    static constexpr auto P_max = 1.0;
    static constexpr auto PSI_to_Pa = 6894.757;
    const int16_t dp_raw = static_cast<int16_t>((0x3F & first_buffer[0]) << 8) | first_buffer[1];
    const double diff_press_PSI = -((dp_raw - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
    const double diff_press_pa_raw = diff_press_PSI * PSI_to_Pa;
    return_value.differential_pressure = diff_press_pa_raw;

    return return_value;
}


} // namespace hardware
} // namespace unitair
