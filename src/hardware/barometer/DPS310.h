/** @file
    Barometer driver
*/

#pragma once

#include <string>
#include <tuple>
#include <vector>

#include "tools/conversions.hpp"
#include "tools/boost/leaf.hpp"

namespace unitair {
namespace hardware {

class DPS310 {
    public:
        /*!
         * \brief Constructs the DPS310 device using I2C.
         *
         * Constructs the DPS310 on the requested I2C device. Throws if the device cannot be opened or configured.
         *
         * Make sure the user running the executable has the correct permissions to access the I2C device; this is
         * usually solved by adding the user to the "i2c" group.
         * \param[in] device_name The filename of the device to open (e.g., "/dev/i2c-0")
         * \param[in] address_toggle A boolean that toggles between I2C addresses (true == 0x76; false == 0x77)
         */
        DPS310(const std::string& device_name = "/dev/i2c-0", bool address_toggle = false);
        DPS310(const DPS310&);
        DPS310(DPS310&&);
        DPS310& operator=(DPS310);
        void swap(DPS310&) noexcept;
        friend void swap(DPS310& lhs, DPS310& rhs) noexcept { lhs.swap(rhs); }
        ~DPS310();

        /*!
         * \brief An error type commonly returned from the methods in this class.
         */
        enum class Error { READ_FAILED, WRITE_FAILED, VALIDATE_FAILED };

        /*!
         * \brief Read all values from FIFO
         */
        boost::leaf::result<std::tuple<std::vector<std::int32_t>, std::vector<std::int32_t>>> readAllValuesFromFifo();

        /*!
         * \brief Calibration coefficients
         *
         * All calibration coefficients are two's complement signed numbers with appropriate sign extension.
         */
        struct CalibrationCoefficients {
                std::int32_t c0{ 0 };   // 12-bit
                std::int32_t c1{ 0 };   // 12-bit
                std::int32_t c00{ 0 };  // 20-bit
                std::int32_t c10{ 0 };  // 20-bit
                std::int32_t c01{ 0 };  // 16-bit
                std::int32_t c11{ 0 };  // 16-bit
                std::int32_t c20{ 0 };  // 16-bit
                std::int32_t c21{ 0 };  // 16-bit
                std::int32_t c30{ 0 };  // 16-bit
        };

        /*!
         * \brief Read calibration coefficients
         *
         * All coefficients are two's complement signed numbers with appropriate sign extension.
         */
        boost::leaf::result<CalibrationCoefficients> readCalibrationCoefficients();

        /*!
         * \brief Queries whether the calibration coefficients are from external temperature sensor
         *
         * There are two temperature sensors on the device - one is internal to the ASIC and the other is external to the
         * pressure sensor. The source of the temperature measurement must match up to the calibration coefficients.
         * You may set the source of the temperature measurement using the "enableExternalTemperatureSensor" method.
         * \return True if from external; false if from internal
         */
        boost::leaf::result<bool> areCoefficientsFromExternalTemperatureSensor();

        /*!
         * \brief Read pressure register
         *
         * Please see the manual (Sec. 4.9.1) to determine the metric pressure value from calibration coefficients.
         *
         * \return Unitless pressure value as two's complement signed value with 24-bit resolution
         */
        boost::leaf::result<std::int32_t> readPressure();

        /*!
         * \brief Read temperature register
         *
         * Please see the manual (Sec. 4.9.2) to determine the metric temperature value from calibration coefficients.
         *
         * \return Unitless temperature value as two's complement signed value with 24-bit resolution
         */
        boost::leaf::result<std::int32_t> readTemperature();

        /*!
         * \brief Indicates output rate
         */
        enum class OutputRate { _1HZ, _2HZ, _4HZ, _8HZ, _16HZ, _32HZ, _64HZ, _128HZ };

        /*!
         * \brief Indicates oversampling/precision factor
         */
        enum class OversamplingFactor { _1X, _2X, _4X, _8X, _16X, _32X, _64X, _128X };

        /*!
         * \brief Sets pressure measurement rate and precision factor
         *
         * Please see the manual (Table 16) for information on oversampling factor vs. precision.
         *
         * Increasing the oversampling factor will increase the measurement time; consequently, the total
         * measurement time must be less than the desired output rate.
         *
         * For example, for an oversampling factor of 64x, the total measurement time will require 104.4 ms (see Table 16).
         * Thus, the requested output rate must be less than 10 Hz. It is important to note that this is only valid
         * for the case where temperature is not being measured.
         *
         * In general:
         *     (temperature rate * temperature measurement time) + (pressure rate * pressure measurement time) < 1.
         *
         * According to the manual, a "bit shift" must be enabled when oversampling is 16x or greater. This function
         * automatically enables/disables this.
         */
        boost::leaf::result<void> configurePressureMeasurement(OutputRate rate = OutputRate::_1HZ,
                                                               OversamplingFactor factor = OversamplingFactor::_1X,
                                                               bool validate = true);

        /*!
         * \brief Sets temperature measurement rate and precision factor
         *
         * Please see the manual (Table 16) for information on oversampling factor vs. precision.
         *
         * Increasing the oversampling factor will increase the measurement time; consequently, the total
         * measurement time must be less than the desired output rate.
         *
         * For example, for an oversampling factor of 64x, the total measurement time will require 104.4 ms (see Table 16).
         * Thus, the requested output rate must be less than 10 Hz. It is important to note that this is only valid
         * for the case where pressure is not being measured.
         *
         * In general:
         *     (temperature rate * temperature measurement time) + (pressure rate * pressure measurement time) < 1.
         *
         * According to the manual, a "bit shift" must be enabled when oversampling is 16x or greater. This function
         * automatically enables/disables this.
         */
        boost::leaf::result<void> configureTemperatureMeasurement(OutputRate rate = OutputRate::_1HZ,
                                                                  OversamplingFactor factor = OversamplingFactor::_1X,
                                                                  bool validate = true);

        /*!
         * \brief Indicates measurement mode.
         */
        enum class MeasurementMode {
                IDLE,
                SINGLE_PRESSURE,
                SINGLE_TEMPERATURE,
                CONTINUOUS_PRESSURE,
                CONTINUOUS_TEMPERATURE,
                CONTINUOUS_TEMPERATURE_AND_PRESSURE
        };

        /*!
         * \brief Set measurement mode.
         */
        boost::leaf::result<void> setMeasurementMode(MeasurementMode mode = MeasurementMode::IDLE, bool validate = true);

        /*!
         * \brief Query measurement status.
         *
         * Indicates whether new temperature and pressure measurements are ready.
         * \return Tuple of bools indicating (temperature ready, pressure ready)
         */
        boost::leaf::result<std::tuple<bool, bool>> queryMeasurementStatus();

        /*!
         * \brief Enables the FIFO.
         */
        boost::leaf::result<void> enableFifo(bool enable = true, bool validate = true);

        /*!
         * \brief Checks FIFO status
         * \return FIFO is full (1st retval) and FIFO is empty (2nd retval).
         */
        boost::leaf::result<std::tuple<bool, bool>> queryFifoStatus();

        /*!
         * \brief Flush the FIFO.
         */
        boost::leaf::result<void> flushFifo();

        /*!
         * \brief Query if the sensor is ready
         * \return Coefficients are ready (1st retval) and sensor is ready (2nd retval)
         */
        boost::leaf::result<std::tuple<bool, bool>> isSensorReady();

        /*!
         * \brief Use external temperature sensor
         *
         * There are two temperature sensors on the device - one is internal to the ASIC and the other is external
         * and based on the MEMS pressure sensor. Note that there are two sets of calibration coefficients; you must query
         * the "coefficient source" register to verify the coefficients match up to the correct sensor you're using.
         *
         */
        boost::leaf::result<void> enableExternalTemperatureSensor(bool enable = true, bool validate = true);

        /*!
         * \brief Performs a soft reset of the device.
         */
        boost::leaf::result<void> performSoftReset();

    private:
        int _fd{ -1 };  //!< file descriptor
};

} // namespace hardware
} // namespace unitair
