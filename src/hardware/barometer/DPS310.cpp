/** @file
    Barometer driver implementation
*/

#include "hardware/barometer/DPS310.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

#include <array>
#include <exception>
#include <system_error>
#include <type_traits>

namespace unitair {
namespace hardware {

namespace {
/*!
 * \brief Reads a byte from the I2C bus.
 */
boost::leaf::result<std::uint8_t> read_byte(int& file_descriptor, const std::uint8_t reg)
{
    using Error = DPS310::Error;
    const auto read_result = i2c_smbus_read_byte_data(file_descriptor, (unsigned char)(reg));
    if (read_result < 0) {
        return boost::leaf::new_error(Error::READ_FAILED);
    }
    return static_cast<std::uint8_t>(read_result);
}

/*!
 * \brief Writes a byte to the I2C bus.
 */
boost::leaf::result<void> write_byte(int& file_descriptor, const std::uint8_t reg, const std::uint8_t value)
{
    using Error = DPS310::Error;
    const auto write_result = i2c_smbus_write_byte_data(
      file_descriptor, (unsigned char)(reg), (unsigned char)(value));
    if (write_result < 0) {
        return boost::leaf::new_error(Error::WRITE_FAILED);
    }
    return {};
}

/*!
 * \brief Reads a byte from the I2C bus and then checks equality with an expected value.
 */
boost::leaf::result<void> validate_byte(int& file_descriptor, const std::uint8_t reg, const std::uint8_t value)
{
    using Error = DPS310::Error;
    BOOST_LEAF_AUTO(validate_result, read_byte(file_descriptor, reg));
    if (validate_result != value) {
        return boost::leaf::new_error(Error::VALIDATE_FAILED);
    }
    return {};
}

/*!
 * \brief Reads N bytes from the I2C bus.
 */
template<std::size_t N>
boost::leaf::result<std::array<std::uint8_t, N>> read_bytes(int& file_descriptor, const std::uint8_t reg)
{
    using Error = DPS310::Error;
    std::array<std::uint8_t, N> bytes;
    const auto read_result = i2c_smbus_read_i2c_block_data(
      file_descriptor, (unsigned char)(reg), N, reinterpret_cast<unsigned char*>(&bytes[0]));
    if (read_result < 0 || read_result != N) {
        return boost::leaf::new_error(Error::READ_FAILED);
    }
    return bytes;
}

/*!
 * \brief Sign extends an integral type from N bits to the full width of the underlying type T
 */
template<typename T, std::size_t N, std::enable_if_t<std::is_integral<T>::value, bool> = true>
constexpr inline T sign_extend(T x)
{
    // clear top part:
    constexpr auto n = T{ (1U << N) - 1 };
    x &= n;

    // sign-extend:
    constexpr auto m = T{ 1U << (N - 1) };
    return (x ^ m) - m;
}
}  // anonymous namespace

DPS310::DPS310(const std::string& device_name, const bool address_toggle)
{
    // open I2C device
    _fd = open(device_name.c_str(), O_RDWR);
    if (_fd < 0) {
        throw std::system_error(errno, std::system_category(), "Could not open I2C device.");
    }
    const auto address = address_toggle ? 0x76 : 0x77;
    if (ioctl(_fd, I2C_SLAVE, address) < 0) {
        close(_fd);  // must close the file descriptor prior to throwing
        throw std::system_error(errno, std::system_category(), "Could not configure I2C device.");
    }
}

DPS310::DPS310(const DPS310& rhs)
{
    _fd = dup(rhs._fd);
}

DPS310::DPS310(DPS310&& rhs)
{
    _fd = std::exchange(rhs._fd, -1);
}

DPS310& DPS310::operator=(DPS310 copy)
{
    copy.swap(*this);
    return *this;
}

void DPS310::swap(DPS310& rhs) noexcept
{
    std::swap(_fd, rhs._fd);
}

DPS310::~DPS310()
{
    close(_fd);
}

boost::leaf::result<std::tuple<std::vector<std::int32_t>, std::vector<std::int32_t>>> DPS310::readAllValuesFromFifo()
{
    using byte = std::uint8_t;
    using int32 = std::int32_t;

    static constexpr auto PSR_B2 = std::uint8_t{ 0x00 };
    static constexpr auto MAX_FIFO_SIZE = 32u;

    // reserve space on the heap for the maximum number of expected pressure values
    std::vector<int32> pressure;
    pressure.reserve(MAX_FIFO_SIZE);

    // reserve space on the heap for the maximum number of expected temperature values
    std::vector<int32> temperature;
    temperature.reserve(MAX_FIFO_SIZE);

    // read FIFO iteratively until we hit the break keyword ('0x00800000')
    static constexpr auto FIFO_IS_EMPTY_VALUE = 0x008000000;
    auto count = 0u;
    do {
        BOOST_LEAF_AUTO(byte_array, read_bytes<3>(_fd, PSR_B2));
        const auto value = (int32(byte_array[2]) << 16) | (int32(byte_array[1]) << 8) |
                           int32(byte_array[0]);
        if (value == FIFO_IS_EMPTY_VALUE) {
            break;  // FIFO is empty
        }
        // check lsb to determine if pressure or temperature measurement
        if ((byte_array[0] & byte{ 1 }) == byte{ 0 }) {
            temperature.push_back(value);
        }
        else {
            pressure.push_back(value);
        }
        ++count;
    } while (count < 2 * MAX_FIFO_SIZE);
    return std::make_tuple(pressure, temperature);
}

boost::leaf::result<DPS310::CalibrationCoefficients> DPS310::readCalibrationCoefficients()
{
    using byte = std::uint8_t;
    using int16 = std::int16_t;
    using int32 = std::int32_t;

    // read calibration coefficients (as bytes) from the device
    BOOST_LEAF_AUTO(c0_msb, read_byte(_fd, byte{ 0x10 }));
    BOOST_LEAF_AUTO(c0_c1, read_byte(_fd, byte{ 0x11 }));
    BOOST_LEAF_AUTO(c1_lsb, read_byte(_fd, byte{ 0x12 }));
    BOOST_LEAF_AUTO(c00_msb, read_byte(_fd, byte{ 0x13 }));
    BOOST_LEAF_AUTO(c00_lsb, read_byte(_fd, byte{ 0x14 }));
    BOOST_LEAF_AUTO(c00_c10, read_byte(_fd, byte{ 0x15 }));
    BOOST_LEAF_AUTO(c10_msb, read_byte(_fd, byte{ 0x16 }));
    BOOST_LEAF_AUTO(c10_lsb, read_byte(_fd, byte{ 0x17 }));
    BOOST_LEAF_AUTO(c01_msb, read_byte(_fd, byte{ 0x18 }));
    BOOST_LEAF_AUTO(c01_lsb, read_byte(_fd, byte{ 0x19 }));
    BOOST_LEAF_AUTO(c11_msb, read_byte(_fd, byte{ 0x1A }));
    BOOST_LEAF_AUTO(c11_lsb, read_byte(_fd, byte{ 0x1B }));
    BOOST_LEAF_AUTO(c20_msb, read_byte(_fd, byte{ 0x1C }));
    BOOST_LEAF_AUTO(c20_lsb, read_byte(_fd, byte{ 0x1D }));
    BOOST_LEAF_AUTO(c21_msb, read_byte(_fd, byte{ 0x1E }));
    BOOST_LEAF_AUTO(c21_lsb, read_byte(_fd, byte{ 0x1F }));
    BOOST_LEAF_AUTO(c30_msb, read_byte(_fd, byte{ 0x20 }));
    BOOST_LEAF_AUTO(c30_lsb, read_byte(_fd, byte{ 0x21 }));

    // convert calibration bytes to integer values; store in a struct and return it
    CalibrationCoefficients output;
    output.c0 = int16(c0_msb) << 4;                   // msb
    output.c0 |= int16(c0_c1 >> 4) & 0x000F;          // lsb
    output.c0 = sign_extend<decltype(output.c0), 12>(output.c0);  // sign-extend from 12-bit

    output.c1 = (int16(c0_c1) & 0x000F) << 8;         // msb
    output.c1 |= int16(c1_lsb);                       // lsb
    output.c1 = sign_extend<decltype(output.c1), 12>(output.c1);  // sign-extend from 12-bit

    output.c00 = int32(c00_msb) << 12;  // msb
    output.c00 |= int32(c00_lsb) << 4;
    output.c00 |= int32(c00_c10 >> 4);                   // lsb
    output.c00 = sign_extend<decltype(output.c00), 20>(output.c00);  // sign-extend from 20-bit

    output.c10 = int32(byte{ 0b0000'1111 } & c00_c10) << 16;  // msb
    output.c10 |= int32(c10_msb) << 8;
    output.c10 |= int32(c10_lsb);                        // lsb
    output.c10 = sign_extend<decltype(output.c10), 20>(output.c10);  // sign-extend from 20-bit

    output.c01 = (int16(c01_msb) << 8) | int16(c01_lsb);
    output.c01 = sign_extend<decltype(output.c01), 16>(output.c01);  // sign-extend from 16-bit

    output.c11 = (int16(c11_msb) << 8) | int16(c11_lsb);
    output.c11 = sign_extend<decltype(output.c11), 16>(output.c11);  // sign-extend from 16-bit

    output.c20 = (int16(c20_msb) << 8) | int16(c20_lsb);
    output.c20 = sign_extend<decltype(output.c20), 16>(output.c20);  // sign-extend from 16-bit

    output.c21 = (static_cast<int16>(c21_msb) << 8) | static_cast<int16>(c21_lsb);
    output.c21 = sign_extend<decltype(output.c21), 16>(output.c21);  // sign-extend from 16-bit

    output.c30 = (static_cast<int16>(c30_msb) << 8) | static_cast<int16>(c30_lsb);
    output.c30 = sign_extend<decltype(output.c30), 16>(output.c30);  // sign-extend from 16-bit
    return output;
}

boost::leaf::result<bool> DPS310::areCoefficientsFromExternalTemperatureSensor()
{
    using byte = std::uint8_t;
    static constexpr auto TMP_COEF_SRCE = byte{ 0x28 };
    BOOST_LEAF_AUTO(read_value, read_byte(_fd, TMP_COEF_SRCE));
    return (read_value & byte{ 0b1000'0000 }) == byte{ 0b1000'0000 };
}

boost::leaf::result<std::int32_t> DPS310::readPressure()
{
    using int32 = std::int32_t;

    static constexpr auto PRS_B2 = std::uint8_t{ 0x00 };
    BOOST_LEAF_AUTO(read_result, read_bytes<3>(_fd, PRS_B2));
    auto pressure = int32(read_result[0]) << 16;
    pressure |= int32(read_result[1]) << 8;
    pressure |= int32(read_result[2]);
    return sign_extend<decltype(pressure), 24>(pressure);  // sign-extend as 24-bit
}

boost::leaf::result<std::int32_t> DPS310::readTemperature()
{
    using int32 = std::int32_t;

    static constexpr auto TMP_B2 = std::uint8_t{ 0x03 };
    BOOST_LEAF_AUTO(read_result, read_bytes<3>(_fd, TMP_B2));
    auto temperature = int32(read_result[0]) << 16;
    temperature |= int32(read_result[1]) << 8;
    temperature |= int32(read_result[2]);
    return sign_extend<decltype(temperature), 24>(temperature);  // sign-extend as 24-bit
}

boost::leaf::result<void> DPS310::configurePressureMeasurement(const OutputRate rate,
                                                               const OversamplingFactor factor,
                                                               const bool validate)
{
    using byte = std::uint8_t;
    static constexpr auto PRS_CFG = byte{ 0x06 };
    BOOST_LEAF_AUTO(read_value, read_byte(_fd, PRS_CFG));
    auto command = read_value & byte{ 0b1000'0000 };
    switch (rate) {
        // clang-format off
        default: [[fallthrough]];
        case OutputRate::_1HZ:
            break;  // clang-format on
        case OutputRate::_2HZ:
            command |= byte{ 0b001 } << 4;
            break;
        case OutputRate::_4HZ:
            command |= byte{ 0b010 } << 4;
            break;
        case OutputRate::_8HZ:
            command |= byte{ 0b011 } << 4;
            break;
        case OutputRate::_16HZ:
            command |= byte{ 0b100 } << 4;
            break;
        case OutputRate::_32HZ:
            command |= byte{ 0b101 } << 4;
            break;
        case OutputRate::_64HZ:
            command |= byte{ 0b110 } << 4;
            break;
        case OutputRate::_128HZ:
            command |= byte{ 0b111 } << 4;
            break;
    }
    switch (factor) {
        // clang-format off
        default: [[fallthrough]];
        case OversamplingFactor::_1X:
            break;  // clang-format on
        case OversamplingFactor::_2X:
            command |= byte{ 0b0001 };
            break;
        case OversamplingFactor::_4X:
            command |= byte{ 0b0010 };
            break;
        case OversamplingFactor::_8X:
            command |= byte{ 0b0011 };
            break;
        case OversamplingFactor::_16X:
            command |= byte{ 0b0100 };
            break;
        case OversamplingFactor::_32X:
            command |= byte{ 0b0101 };
            break;
        case OversamplingFactor::_64X:
            command |= byte{ 0b0110 };
            break;
        case OversamplingFactor::_128X:
            command |= byte{ 0b0111 };
            break;
    }
    BOOST_LEAF_CHECK(write_byte(_fd, PRS_CFG, command));
    if (validate) {
        BOOST_LEAF_CHECK(validate_byte(_fd, PRS_CFG, command));
    }

    // enable bit-shift based on oversampling factor
    static constexpr auto CFG_REG = byte{ 0x09 };
    BOOST_LEAF_AUTO(value, read_byte(_fd, CFG_REG));
    const auto bitshift_enabled = (value & byte{ 0b0000'0100 }) == byte{ 0b0000'0100 };
    const auto bitshift_should_be_enabled =
      (factor == OversamplingFactor::_16X) || (factor == OversamplingFactor::_32X) ||
      (factor == OversamplingFactor::_64X) || (factor == OversamplingFactor::_128X);
    if (bitshift_enabled && !bitshift_should_be_enabled) {
        // disable bitshift
        command = value & byte{ 0b1111'1011 };
        BOOST_LEAF_ASSERT(write_byte(_fd, CFG_REG, command));
        if (validate) {
            BOOST_LEAF_ASSERT(validate_byte(_fd, CFG_REG, command));
        }
    }
    else if (!bitshift_enabled && bitshift_should_be_enabled) {
        // enable bitshift
        command = value | byte{ 0b0000'0100 };
        BOOST_LEAF_ASSERT(write_byte(_fd, CFG_REG, command));
        if (validate) {
            BOOST_LEAF_ASSERT(validate_byte(_fd, CFG_REG, command));
        }
    }
    return {};
}

boost::leaf::result<void> DPS310::configureTemperatureMeasurement(OutputRate rate,
                                                                  OversamplingFactor factor,
                                                                  bool validate)
{
    using byte = std::uint8_t;
    static constexpr auto TMP_CFG = byte{ 0x07 };
    BOOST_LEAF_AUTO(read_value, read_byte(_fd, TMP_CFG));
    auto command = read_value & byte{ 0b1000'0000 };
    switch (rate) {
        // clang-format off
        default: [[fallthrough]];
        case OutputRate::_1HZ:
            break;  // clang-format on
        case OutputRate::_2HZ:
            command |= byte{ 0b0001 } << 4;
            break;
        case OutputRate::_4HZ:
            command |= byte{ 0b0010 } << 4;
            break;
        case OutputRate::_8HZ:
            command |= byte{ 0b0011 } << 4;
            break;
        case OutputRate::_16HZ:
            command |= byte{ 0b0100 } << 4;
            break;
        case OutputRate::_32HZ:
            command |= byte{ 0b0101 } << 4;
            break;
        case OutputRate::_64HZ:
            command |= byte{ 0b0110 } << 4;
            break;
        case OutputRate::_128HZ:
            command |= byte{ 0b0111 } << 4;
            break;
    }
    switch (factor) {
        // clang-format off
        default: [[fallthrough]];
        case OversamplingFactor::_1X:
            break;  // clang-format on
        case OversamplingFactor::_2X:
            command |= byte{ 0b0001 };
            break;
        case OversamplingFactor::_4X:
            command |= byte{ 0b0010 };
            break;
        case OversamplingFactor::_8X:
            command |= byte{ 0b0011 };
            break;
        case OversamplingFactor::_16X:
            command |= byte{ 0b0100 };
            break;
        case OversamplingFactor::_32X:
            command |= byte{ 0b0101 };
            break;
        case OversamplingFactor::_64X:
            command |= byte{ 0b0110 };
            break;
        case OversamplingFactor::_128X:
            command |= byte{ 0b0111 };
            break;
    }
    BOOST_LEAF_CHECK(write_byte(_fd, TMP_CFG, command));
    if (validate) {
        BOOST_LEAF_CHECK(validate_byte(_fd, TMP_CFG, command));
    }

    // enable bit-shift based on oversampling factor
    static constexpr auto CFG_REG = byte{ 0x09 };
    BOOST_LEAF_AUTO(value, read_byte(_fd, CFG_REG));
    const auto bitshift_enabled = (value & byte{ 0b0000'1000 }) == byte{ 0b0000'1000 };
    const auto bitshift_should_be_enabled =
      (factor == OversamplingFactor::_16X) || (factor == OversamplingFactor::_32X) ||
      (factor == OversamplingFactor::_64X) || (factor == OversamplingFactor::_128X);
    if (bitshift_enabled && !bitshift_should_be_enabled) {
        // disable bitshift
        command = value & byte{ 0b1111'0111 };
        BOOST_LEAF_ASSERT(write_byte(_fd, CFG_REG, command));
        if (validate) {
            BOOST_LEAF_ASSERT(validate_byte(_fd, CFG_REG, command));
        }
    }
    else if (!bitshift_enabled && bitshift_should_be_enabled) {
        // enable bitshift
        command = value | byte{ 0b0000'1000 };
        BOOST_LEAF_ASSERT(write_byte(_fd, CFG_REG, command));
        if (validate) {
            BOOST_LEAF_ASSERT(validate_byte(_fd, CFG_REG, command));
        }
    }
    return {};
}

boost::leaf::result<void> DPS310::setMeasurementMode(const MeasurementMode mode, const bool validate)
{
    using byte = std::uint8_t;
    static constexpr auto MEAS_CFG = byte{ 0x08 };
    BOOST_LEAF_AUTO(read_value, read_byte(_fd, MEAS_CFG));
    auto command = read_value & byte{ 0b1111'1000 };
    switch (mode) {
        // clang-format off
        default: [[fallthrough]];
        case MeasurementMode::IDLE:
            break;  // clang-format on
        case MeasurementMode::SINGLE_PRESSURE:
            command |= byte{ 0b0000'0001 };
            break;
        case MeasurementMode::SINGLE_TEMPERATURE:
            command |= byte{ 0b0000'0010 };
            break;
        case MeasurementMode::CONTINUOUS_PRESSURE:
            command |= byte{ 0b0000'0101 };
            break;
        case MeasurementMode::CONTINUOUS_TEMPERATURE:
            command |= byte{ 0b0000'0110 };
            break;
        case MeasurementMode::CONTINUOUS_TEMPERATURE_AND_PRESSURE:
            command |= byte{ 0b0000'0111 };
            break;
    }
    BOOST_LEAF_CHECK(write_byte(_fd, MEAS_CFG, command));
    if (validate) {
        BOOST_LEAF_CHECK(validate_byte(_fd, MEAS_CFG, command));
    }
    return {};
}

boost::leaf::result<std::tuple<bool, bool>> DPS310::queryMeasurementStatus()
{
    using byte = std::uint8_t;
    static constexpr auto MEAS_CFG = byte{ 0x08 };
    BOOST_LEAF_AUTO(read_value, read_byte(_fd, MEAS_CFG));
    const auto is_temperature_ready = (read_value & byte{ 0b0010'0000 }) == byte{ 0b0010'0000 };
    const auto is_pressure_ready = (read_value & byte{ 0b0001'0000 }) == byte{ 0b0001'0000 };
    return std::make_tuple(is_temperature_ready, is_pressure_ready);
}

boost::leaf::result<void> DPS310::enableFifo(const bool enable, const bool validate)
{
    using byte = std::uint8_t;
    static constexpr auto CFG_REG = byte{ 0x09 };
    BOOST_LEAF_AUTO(read_value, read_byte(_fd, CFG_REG));
    auto command = read_value & byte{ 0b1111'1101 };
    if (enable) {
        command |= byte{ 0b0000'0010 };
    }
    BOOST_LEAF_CHECK(write_byte(_fd, CFG_REG, command));
    if (validate) {
        BOOST_LEAF_CHECK(validate_byte(_fd, CFG_REG, command));
    }
    return {};
}

boost::leaf::result<std::tuple<bool, bool>> DPS310::queryFifoStatus()
{
    using byte = std::uint8_t;
    static constexpr auto FIFO_STS = byte{ 0x0B };
    BOOST_LEAF_AUTO(read_value, read_byte(_fd, FIFO_STS));
    const auto is_fifo_full = (read_value & byte{ 0b0000'0010 }) == byte{ 0b0000'0010 };
    const auto is_fifo_empty = (read_value & byte{ 0b0000'0001 }) == byte{ 0b0000'0001 };
    return std::make_tuple(is_fifo_full, is_fifo_empty);
}

boost::leaf::result<void> DPS310::flushFifo()
{
    using byte = std::uint8_t;
    static constexpr auto RESET = byte{ 0x0C };
    const auto command = byte{ 0b1000'0000 };
    BOOST_LEAF_CHECK(write_byte(_fd, RESET, command));
    return {};
}

boost::leaf::result<std::tuple<bool, bool>> DPS310::isSensorReady()
{
    using byte = std::uint8_t;
    static constexpr auto MEAS_CFG = byte{ 0x08 };
    BOOST_LEAF_AUTO(read_value, read_byte(_fd, MEAS_CFG));
    const auto are_coeffs_ready = (read_value & byte{ 0b1000'0000 }) == byte{ 0b1000'0000 };
    const auto is_sensor_ready = (read_value & byte{ 0b0100'0000 }) == byte{ 0b0100'0000 };
    return std::make_tuple(are_coeffs_ready, is_sensor_ready);
}

boost::leaf::result<void> DPS310::enableExternalTemperatureSensor(const bool enable, const bool validate)
{
    using byte = std::uint8_t;
    static constexpr auto TMP_CFG = byte{ 0x07 };
    BOOST_LEAF_AUTO(read_value, read_byte(_fd, TMP_CFG));
    auto command = read_value;
    if (enable) {
        command |= byte{ 0b1000'0000 };
    }
    else {
        command &= byte{ 0b0111'1111 };
    }
    BOOST_LEAF_CHECK(write_byte(_fd, TMP_CFG, command));
    if (validate) {
        BOOST_LEAF_CHECK(validate_byte(_fd, TMP_CFG, command));
    }
    return {};
}

boost::leaf::result<void> DPS310::performSoftReset()
{
    using byte = std::uint8_t;
    static constexpr auto RESET = byte{ 0x0C };
    BOOST_LEAF_AUTO(read_value, read_byte(_fd, RESET));
    const auto command = (read_value & byte{ 0b1111'0000 }) | byte{ 0b0000'1001 };
    BOOST_LEAF_CHECK(write_byte(_fd, RESET, command));
    return {};
}

} // namespace hardware
} // namespace unitair
