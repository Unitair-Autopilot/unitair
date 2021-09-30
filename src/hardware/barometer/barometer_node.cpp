/** @file
    ROS node for the DPS310 barometer
*/

#include <cmath>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <tuple>

#include "hardware/barometer/DPS310.h"

#include <ros/ros.h>
#include <unitair/Barometer.h>

namespace unitair {
namespace hardware {

namespace {
enum class NodeError {
    UNDEFINED_OUTPUT_RATE,
    INVALID_OUTPUT_RATE,
    UNDEFINED_OVERSAMPLING_FACTOR,
    INVALID_OVERSAMPLING_FACTOR,
    UNDEFINED_POLLING_PERIOD,
    INVALID_POLLING_PERIOD,
    UNDEFINED_I2C_DEVICE_NAME,
    INVALID_I2C_DEVICE_NUMBER,
    UNDEFINED_I2C_DEVICE_ADDRESS_TOGGLE,
    FAILED_TO_READ_CALIBRATION_COEFFICIENTS
};

bool is_valid(DPS310::OutputRate rate, DPS310::OversamplingFactor factor)
{
    using OutputRate = DPS310::OutputRate;
    using OversamplingFactor = DPS310::OversamplingFactor;
    switch (factor) {
        case OversamplingFactor::_128X:
            // measurement time (both temperature and pressure) = 2 * 206.8 ms
            // theoretical maximum rate = 2.4 Hz
            return ((rate == OutputRate::_1HZ) || (rate == OutputRate::_2HZ));
        case OversamplingFactor::_64X:
            // measurement time (both temperature and pressure) = 2 * 104.4 ms
            // theoretical maximum rate = 4.7 Hz
            return ((rate == OutputRate::_1HZ) || (rate == OutputRate::_2HZ) || (rate == OutputRate::_4HZ));
        case OversamplingFactor::_32X:
            // measurement time (both temperature and pressure) = 2 * 53.2 ms
            // theoretical maximum rate = 9.3 Hz
            return ((rate == OutputRate::_1HZ) || (rate == OutputRate::_2HZ) || (rate == OutputRate::_4HZ) ||
                    (rate == OutputRate::_8HZ));
        case OversamplingFactor::_16X:
            // measurement time (both temperature and pressure) = 2 * 27.6 ms
            // theoretical maximum rate = 18.1 Hz
            return ((rate == OutputRate::_1HZ) || (rate == OutputRate::_2HZ) || (rate == OutputRate::_4HZ) ||
                    (rate == OutputRate::_8HZ) || (rate == OutputRate::_16HZ));
        case OversamplingFactor::_8X:
            // measurement time (both temperature and pressure) = 2 * 14.8 ms
            // theoretical maximum rate = 33.7 Hz
            return ((rate == OutputRate::_1HZ) || (rate == OutputRate::_2HZ) || (rate == OutputRate::_4HZ) ||
                    (rate == OutputRate::_8HZ) || (rate == OutputRate::_16HZ) || (rate == OutputRate::_32HZ));
        case OversamplingFactor::_4X:
            // measurement time (both temperature and pressure) = 2 * 8.4 ms
            // theoretical maximum rate = 59.5 Hz
            return ((rate == OutputRate::_1HZ) || (rate == OutputRate::_2HZ) || (rate == OutputRate::_4HZ) ||
                    (rate == OutputRate::_8HZ) || (rate == OutputRate::_16HZ) || (rate == OutputRate::_32HZ));
        case OversamplingFactor::_2X:
            // measurement time (both temperature and pressure) = 2 * 5.2 ms
            // theoretical maximum rate = 96.1 Hz
            return ((rate == OutputRate::_1HZ) || (rate == OutputRate::_2HZ) || (rate == OutputRate::_4HZ) ||
                    (rate == OutputRate::_8HZ) || (rate == OutputRate::_16HZ) || (rate == OutputRate::_32HZ) ||
                    (rate == OutputRate::_64HZ));
        case OversamplingFactor::_1X:
            // measurement time (both temperature and pressure) = 2 * 3.6 ms
            // theoretical maximum rate = 138.8 Hz
            return true;
        default:
            return false;
    }
}

boost::leaf::result<DPS310::OutputRate> convert_output_rate(const int rate)
{
    using OutputRate = DPS310::OutputRate;
    switch (rate) {
        case 1:
            return OutputRate::_1HZ;
        case 2:
            return OutputRate::_2HZ;
        case 4:
            return OutputRate::_4HZ;
        case 8:
            return OutputRate::_8HZ;
        case 16:
            return OutputRate::_16HZ;
        case 32:
            return OutputRate::_32HZ;
        case 64:
            return OutputRate::_64HZ;
        case 128:
            return OutputRate::_128HZ;
        default:
            return boost::leaf::new_error(NodeError::INVALID_OUTPUT_RATE);
    }
    return boost::leaf::new_error(NodeError::INVALID_OUTPUT_RATE);
}

boost::leaf::result<DPS310::OutputRate> get_output_rate(ros::NodeHandle& nh)
{
    int output_rate_hz = 1;
    if (!nh.getParam("hardware/barometer/sample_rate", output_rate_hz)) {
        return boost::leaf::new_error(NodeError::UNDEFINED_OUTPUT_RATE);
    }
    return convert_output_rate(output_rate_hz);
}

boost::leaf::result<DPS310::OversamplingFactor> convert_oversampling_factor(const int factor)
{
    using OversamplingFactor = DPS310::OversamplingFactor;
    switch (factor) {
        case 1:
            return OversamplingFactor::_1X;
        case 2:
            return OversamplingFactor::_2X;
        case 4:
            return OversamplingFactor::_4X;
        case 8:
            return OversamplingFactor::_8X;
        case 16:
            return OversamplingFactor::_16X;
        case 32:
            return OversamplingFactor::_32X;
        case 64:
            return OversamplingFactor::_64X;
        case 128:
            return OversamplingFactor::_128X;
        default:
            return boost::leaf::new_error(NodeError::INVALID_OVERSAMPLING_FACTOR);
    }
    return boost::leaf::new_error(NodeError::INVALID_OVERSAMPLING_FACTOR);
}

boost::leaf::result<DPS310::OversamplingFactor> get_oversampling_factor(ros::NodeHandle& nh)
{
    int oversampling_factor = 32;
    nh.getParam("oversampling_factor", oversampling_factor);
    return convert_oversampling_factor(oversampling_factor);
}

boost::leaf::result<double> get_polling_period(ros::NodeHandle& nh)
{
    double polling_period_sec = 0.0625;
    nh.getParam("polling_period_sec", polling_period_sec);
    if (polling_period_sec <= 0.0) {
        return boost::leaf::new_error(NodeError::INVALID_POLLING_PERIOD);
    }
    return polling_period_sec;
}

boost::leaf::result<std::string> get_i2c_device_name(ros::NodeHandle& nh)
{
    std::string i2c_device_name = "";
    if (!nh.getParam("hardware/barometer/i2c_device", i2c_device_name)) {
        return boost::leaf::new_error(NodeError::UNDEFINED_I2C_DEVICE_NAME);
    }
    return i2c_device_name;
}

boost::leaf::result<bool> get_i2c_device_address(ros::NodeHandle& nh)
{
    bool i2c_device_address_toggle = false;
    nh.getParam("i2c_device_address_toggle", i2c_device_address_toggle);
    return i2c_device_address_toggle;
}

bool is_sensor_ready(DPS310& sensor,
                     const std::chrono::milliseconds max_time_to_wait = std::chrono::milliseconds{ 5000 })
{
    using std::chrono::duration_cast;
    using clock = std::chrono::system_clock;
    using milliseconds = std::chrono::milliseconds;
    static constexpr auto SLEEP_DURATION = std::chrono::milliseconds(50);  // TODO: make configurable
    static constexpr auto MAX_ATTEMPTS = 1000u;                            // TODO: make configurable

    // repeatedly query the sensor and wait until it's ready
    const auto start_time = clock::now();
    auto count = std::size_t{ 0 };
    do {
        const auto result = sensor.isSensorReady();
        if (!result || !std::get<0>(result.value()) || !std::get<1>(result.value())) {
            // sensor is not ready; check for timeout
            if (duration_cast<milliseconds>(clock::now() - start_time) >= max_time_to_wait) {
                return false;  // failure: timeout occurred
            }
        }
        else {
            return true;  // success: sensor is ready
        }
        std::this_thread::sleep_for(SLEEP_DURATION);
        count++;
    } while (count < MAX_ATTEMPTS);
    return false;  // failure: too many attempts
}

bool reset_sensor(DPS310& sensor,
                  const std::chrono::milliseconds max_time_to_wait = std::chrono::milliseconds{ 3000 })
{
    using std::chrono::duration_cast;
    using clock = std::chrono::system_clock;
    using milliseconds = std::chrono::milliseconds;
    static constexpr auto SLEEP_DURATION = std::chrono::milliseconds(50);  // TODO: make configurable
    static constexpr auto MAX_ATTEMPTS = 1000u;                            // TODO: make configurable

    // repeatedly attempt to reset the sensor
    const auto start_time = clock::now();
    auto count = std::size_t{ 0 };
    do {
        const auto result = sensor.performSoftReset();
        if (!result) {
            // failed to reset sensor
            if (duration_cast<milliseconds>(clock::now() - start_time) >= max_time_to_wait) {
                return false;  // failure: timeout occurred
            }
        }
        else {
            return true;  // success: request was successful
        }
        std::this_thread::sleep_for(SLEEP_DURATION);
        count++;
    } while (count < MAX_ATTEMPTS);
    return false;  // failure: too many attempts
}

bool set_measurement_mode(DPS310& sensor,
                          const DPS310::MeasurementMode mode,
                          const std::chrono::milliseconds max_time_to_wait = std::chrono::milliseconds{ 3000 })
{
    using std::chrono::duration_cast;
    using clock = std::chrono::system_clock;
    using milliseconds = std::chrono::milliseconds;
    static constexpr auto SLEEP_DURATION = std::chrono::milliseconds(50);  // TODO: make configurable
    static constexpr auto MAX_ATTEMPTS = 1000u;                            // TODO: make configurable

    // repeatedly attempt to set the measurement mode
    const auto start_time = clock::now();
    auto count = std::size_t{ 0 };
    do {
        const auto result = sensor.setMeasurementMode(mode);
        if (!result) {
            // failed to set measurement mode
            if (duration_cast<milliseconds>(clock::now() - start_time) >= max_time_to_wait) {
                return false;  // failure: timeout occurred
            }
        }
        else {
            return true;  // success: request was successful
        }
        std::this_thread::sleep_for(SLEEP_DURATION);
        count++;
    } while (count < MAX_ATTEMPTS);
    return false;  // failure: too many attempts
}

bool configure_temperature_measurement(DPS310& sensor,
                                       const DPS310::OutputRate rate,
                                       const DPS310::OversamplingFactor factor,
                                       const std::chrono::milliseconds max_time_to_wait = std::chrono::milliseconds{
                                         3000 })
{
    using std::chrono::duration_cast;
    using clock = std::chrono::system_clock;
    using milliseconds = std::chrono::milliseconds;
    static constexpr auto SLEEP_DURATION = std::chrono::milliseconds(50);  // TODO: make configurable
    static constexpr auto MAX_ATTEMPTS = 1000u;                            // TODO: make configurable

    // repeatedly attempt to configure temperature measurement
    const auto start_time = clock::now();
    auto count = std::size_t{ 0 };
    do {
        const auto result = sensor.configureTemperatureMeasurement(rate, factor);
        if (!result) {
            // failed to configure temperature measurement
            if (duration_cast<milliseconds>(clock::now() - start_time) >= max_time_to_wait) {
                return false;  // failure: timeout occurred
            }
        }
        else {
            return true;  // success: request was successful
        }
        std::this_thread::sleep_for(SLEEP_DURATION);
        count++;
    } while (count < MAX_ATTEMPTS);
    return false;  // failure: too many attempts
}

bool configure_pressure_measurement(DPS310& sensor,
                                    const DPS310::OutputRate rate,
                                    const DPS310::OversamplingFactor factor,
                                    const std::chrono::milliseconds max_time_to_wait = std::chrono::milliseconds{
                                      3000 })
{
    using std::chrono::duration_cast;
    using clock = std::chrono::system_clock;
    using milliseconds = std::chrono::milliseconds;
    static constexpr auto SLEEP_DURATION = std::chrono::milliseconds(50);  // TODO: make configurable
    static constexpr auto MAX_ATTEMPTS = 1000u;                            // TODO: make configurable

    // repeatedly attempt to configure pressure measurement
    const auto start_time = clock::now();
    auto count = std::size_t{ 0 };
    do {
        const auto result = sensor.configurePressureMeasurement(rate, factor);
        if (!result) {
            // failed to configure pressure measurement
            if (duration_cast<milliseconds>(clock::now() - start_time) >= max_time_to_wait) {
                return false;  // failure: timeout occurred
            }
        }
        else {
            return true;  // success: request was successful
        }
        std::this_thread::sleep_for(SLEEP_DURATION);
        count++;
    } while (count < MAX_ATTEMPTS);
    return false;  // failure: too many attempts
}

boost::leaf::result<DPS310::CalibrationCoefficients> read_calibration_coefficients(
  DPS310& sensor,
  const std::chrono::milliseconds max_time_to_wait = std::chrono::milliseconds{ 3000 })
{
    using std::chrono::duration_cast;
    using clock = std::chrono::system_clock;
    using milliseconds = std::chrono::milliseconds;
    static constexpr auto SLEEP_DURATION = std::chrono::milliseconds(50);  // TODO: make configurable
    static constexpr auto MAX_ATTEMPTS = 1000u;                            // TODO: make configurable

    // repeatedly attempt to read calibration coefficients
    const auto start_time = clock::now();
    auto count = std::size_t{ 0 };
    do {
        auto result = sensor.readCalibrationCoefficients();
        if (!result) {
            // failed to read calibration coefficients
            if (duration_cast<milliseconds>(clock::now() - start_time) >= max_time_to_wait) {
                return boost::leaf::new_error(
                  NodeError::FAILED_TO_READ_CALIBRATION_COEFFICIENTS);  // failure: timeout occurred
            }
        }
        else {
            return result;  // success: request was successful
        }
        std::this_thread::sleep_for(SLEEP_DURATION);
        count++;
    } while (count < MAX_ATTEMPTS);
    return boost::leaf::new_error(NodeError::FAILED_TO_READ_CALIBRATION_COEFFICIENTS);  // failure: too many attempts
}

bool enable_external_temperature_sensor(DPS310& sensor,
                                        const std::chrono::milliseconds max_time_to_wait = std::chrono::milliseconds{
                                          3000 })
{
    using std::chrono::duration_cast;
    using clock = std::chrono::system_clock;
    using milliseconds = std::chrono::milliseconds;
    static constexpr auto SLEEP_DURATION = std::chrono::milliseconds(50);  // TODO: make configurable
    static constexpr auto MAX_ATTEMPTS = 1000u;                            // TODO: make configurable

    // repeatedly attempt to use external temperature sensor
    const auto start_time = clock::now();
    auto count = std::size_t{ 0 };
    do {
        auto result = sensor.enableExternalTemperatureSensor();
        if (!result) {
            // failed to read calibration coefficients
            if (duration_cast<milliseconds>(clock::now() - start_time) >= max_time_to_wait) {
                return false;  // failure: timeout occurred
            }
        }
        else {
            return true;  // success: request was successful
        }
        std::this_thread::sleep_for(SLEEP_DURATION);
        count++;
    } while (count < MAX_ATTEMPTS);
    return false;  // failure: too many attempts
}

bool validate_coefficients_are_from_external_sensor(
  DPS310& sensor,
  const std::chrono::milliseconds max_time_to_wait = std::chrono::milliseconds{ 3000 })
{
    using std::chrono::duration_cast;
    using clock = std::chrono::system_clock;
    using milliseconds = std::chrono::milliseconds;
    static constexpr auto SLEEP_DURATION = std::chrono::milliseconds(50);  // TODO: make configurable
    static constexpr auto MAX_ATTEMPTS = 1000u;                            // TODO: make configurable

    // repeatedly attempt validate coefficients are from external temperature sensor
    const auto start_time = clock::now();
    auto count = std::size_t{ 0 };
    do {
        auto result = sensor.areCoefficientsFromExternalTemperatureSensor();
        if (!result || !result.value()) {
            // failed to read calibration coefficients or coefficients are not from external sensor
            if (duration_cast<milliseconds>(clock::now() - start_time) >= max_time_to_wait) {
                return false;  // failure: timeout occurred
            }
        }
        else {
            return true;  // success: validation was successful
        }
        std::this_thread::sleep_for(SLEEP_DURATION);
        count++;
    } while (count < MAX_ATTEMPTS);
    return false;  // failure: too many attempts
}

std::tuple<double, double> convert_to_metric(const std::int32_t pressure_value,
                                             const std::int32_t temperature_value,
                                             const DPS310::OversamplingFactor factor,
                                             const DPS310::CalibrationCoefficients calibration)
{
    using OversamplingFactor = DPS310::OversamplingFactor;
    auto scaled_temperature = double{ NAN };
    auto scaled_pressure = double{ NAN };
    switch (factor) {
        case OversamplingFactor::_1X:
            scaled_pressure = static_cast<double>(pressure_value) / 524288;
            scaled_temperature = static_cast<double>(temperature_value) / 524288;
            break;
        case OversamplingFactor::_2X:
            scaled_pressure = static_cast<double>(pressure_value) / 1572864;
            scaled_temperature = static_cast<double>(temperature_value) / 1572864;
            break;
        case OversamplingFactor::_4X:
            scaled_pressure = static_cast<double>(pressure_value) / 3670016;
            scaled_temperature = static_cast<double>(temperature_value) / 3670016;
            break;
        case OversamplingFactor::_8X:
            scaled_pressure = static_cast<double>(pressure_value) / 7864320;
            scaled_temperature = static_cast<double>(temperature_value) / 7864320;
            break;
        case OversamplingFactor::_16X:
            scaled_pressure = static_cast<double>(pressure_value) / 253952;
            scaled_temperature = static_cast<double>(temperature_value) / 253952;
            break;
        case OversamplingFactor::_32X:
            scaled_pressure = static_cast<double>(pressure_value) / 516096;
            scaled_temperature = static_cast<double>(temperature_value) / 516096;
            break;
        case OversamplingFactor::_64X:
            scaled_pressure = static_cast<double>(pressure_value) / 1040384;
            scaled_temperature = static_cast<double>(temperature_value) / 1040384;
            break;
        case OversamplingFactor::_128X:
            scaled_pressure = static_cast<double>(pressure_value) / 2088960;
            scaled_temperature = static_cast<double>(temperature_value) / 2088960;
            break;
        default:
            break;
    }
    return std::make_tuple(
      calibration.c00 +
        scaled_pressure * (calibration.c10 + scaled_pressure * (calibration.c20 + scaled_pressure * calibration.c30)) +
        scaled_temperature * calibration.c01 +
        scaled_temperature * scaled_pressure * (calibration.c11 + scaled_pressure * calibration.c21),
      calibration.c0 * 0.5 + calibration.c1 * scaled_temperature);
}

}  // anonymous namespace

} // namespace hardware
} // namespace unitair

using namespace unitair;
using namespace hardware;

int main(int argc, char* argv[])
{
    // initialize ROS
    ros::init(argc, argv, "barometer");
    ros::NodeHandle nh;

    // register publishers
    static constexpr auto PUBLISHER_QUEUE_SIZE = 10u;
    auto pressure_publisher = nh.advertise<unitair::Barometer>("hardware/barometer", PUBLISHER_QUEUE_SIZE);
    // auto temperature_publisher = nh.advertise<sensor_msgs::Temperature>("temperature", PUBLISHER_QUEUE_SIZE);

    // get parameters from ROS parameter server

    const auto output_rate = get_output_rate(nh);
    if (!output_rate) {
        ROS_ERROR("BAROMETER: parameter 'sample_rate' is invalid or undefined");
        return -1;
    }

    const auto oversampling_factor = get_oversampling_factor(nh);
    if (!oversampling_factor) {
        ROS_ERROR("BAROMETER: parameter 'oversampling_factor' is invalid or undefined");
        return -1;
    }

    if (!is_valid(output_rate.value(), oversampling_factor.value())) {
        ROS_ERROR("Desired output rate and oversampling factor are incompatible");
        return -1;
    }

    const auto polling_period_sec = get_polling_period(nh);
    if (!polling_period_sec) {
        ROS_ERROR("BAROMETER: parameter 'polling_period_sec' is invalid or undefined");
        return -1;
    }

    const auto i2c_device_name = get_i2c_device_name(nh);
    if (!i2c_device_name) {
        ROS_ERROR("BAROMETER: parameter 'i2c_device' is invalid or undefined");
        return -1;
    }

    const auto i2c_device_address_toggle = get_i2c_device_address(nh);
    if (!i2c_device_address_toggle) {
        ROS_ERROR("BAROMETER: parameter 'i2c_device_address_toggle' is invalid or undefined");
        return -1;
    }

    // construct the sensor object
    std::shared_ptr<DPS310> sensor{ nullptr };
    try {
        sensor = std::make_shared<DPS310>(i2c_device_name.value(), i2c_device_address_toggle.value());
    }
    catch (const std::exception&) {
        ROS_ERROR("Could not open I2C device.");
        return -1;
    }
    ROS_ASSERT(sensor != nullptr);

    // perform a soft reset of the sensor to reset all parameters to default;
    // ROS_INFO_STREAM(std::string{ ros::this_node::getName() + ": resetting sensor..." }.c_str());
    if (!reset_sensor(*sensor)) {
        ROS_ERROR("Failed to reset the sensor. Check connection to sensor.");
        return -1;
    }

    // check that the sensor is ready after the soft reset
    // ROS_INFO_STREAM(std::string{ ros::this_node::getName() + ": waiting for sensor to be ready..." }.c_str());
    if (!is_sensor_ready(*sensor)) {
        ROS_ERROR("Timed out waiting for sensor to be ready. Check connection to sensor.");
        return -1;
    }

    // enable the use of the external temperature (instead of internal)
    // ROS_INFO_STREAM(std::string{ ros::this_node::getName() + ": enabling external temperature sensor..." }.c_str());
    if (!enable_external_temperature_sensor(*sensor)) {
        ROS_ERROR("Failed to enable external temperature sensor.");
        return -1;
    }

    // validate that coefficients are from external sensor
    // ROS_INFO_STREAM(std::string{ ros::this_node::getName() + ": validating use of external temperature sensor..." }.c_str());
    if (!validate_coefficients_are_from_external_sensor(*sensor)) {
        ROS_ERROR("Could not validate that calibration coefficients are from external temperature sensor.");
        return -1;
    }

    // read the sensor's calibration coefficients
    // ROS_INFO_STREAM(std::string{ ros::this_node::getName() + ": reading calibration coefficients..." }.c_str());
    auto calibration_coefficients = read_calibration_coefficients(*sensor);
    if (!calibration_coefficients) {
        ROS_ERROR("Failed to read calibration coefficients.");
        return -1;
    }

    // configure sensor as requested by parameters
    // ROS_INFO_STREAM(std::string{ ros::this_node::getName() + ": configuring sensor..." }.c_str());
    if (!set_measurement_mode(*sensor, DPS310::MeasurementMode::CONTINUOUS_TEMPERATURE_AND_PRESSURE)) {
        ROS_ERROR("Failed to set measurement mode.");
        return -1;
    }
    if (!configure_pressure_measurement(*sensor, output_rate.value(), oversampling_factor.value())) {
        ROS_ERROR("Failed to configure pressure measurement.");
        return -1;
    }
    if (!configure_temperature_measurement(*sensor, output_rate.value(), oversampling_factor.value())) {
        ROS_ERROR("Failed to configure temperature measurement.");
        return -1;
    }

    // at this point the sensor is configured and ready to go
    // ROS_INFO_STREAM(std::string{ ros::this_node::getName() + ": starting main loop..." }.c_str());
    ros::Rate polling_rate{ 1.0 / polling_period_sec.value() };
    while (ros::ok()) {
        // set the polling rate
        polling_rate.sleep();

        // check if there are new pressure and temperature values
        auto status_result = sensor->queryMeasurementStatus();
        if (!status_result) {
            continue;
        }
        const auto& ready = status_result.value();
        if (!std::get<0>(ready) || !std::get<1>(ready)) {
            continue;
        }
        // read the pressure and temperature values from the device; convert to metric values
        auto temperature = sensor->readTemperature();
        auto pressure = sensor->readPressure();
        if (!temperature || !pressure) {
            continue;
        }
        const double pressure_in_pascals = std::get<0>(convert_to_metric(
          pressure.value(), temperature.value(), oversampling_factor.value(), calibration_coefficients.value()));

        // publish pressure message
        unitair::Barometer pressure_msg;
        pressure_msg.stamp = ros::Time::now();
        pressure_msg.pressure = pressure_in_pascals;
        pressure_publisher.publish(pressure_msg);

        // // publish temperature message
        // sensor_msgs::Temperature temperature_msg;
        // temperature_msg.header.stamp = ros::Time::now();
        // temperature_msg.temperature = temperature_in_deg_celsius;
        // temperature_msg.variance = 0.0;
        // temperature_publisher.publish(temperature_msg);
    }
    // ROS_INFO_STREAM(std::string{ ros::this_node::getName() + ": shutting down..." }.c_str());
    return 0;
}
