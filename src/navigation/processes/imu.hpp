/** @file
Declaration of the IMU class.
*/
#pragma once
#include "navigation/estimator.hpp"

namespace unitair {
namespace navigation {
namespace processes {

//////////////////////////////////////////////////

/// Process model with accel/angvel driven by an inertial measurement unit,
/// biases/atmosphere on Brownian random walks, and inference via linearization (EKF)
class IMU : public Process, public inference_methods::EKF {
    /// @name Given
    //@{
    // Accelerometer parameters
    RealMat<3,3> const accelerometer_rotmat; ///< Orientation of accelerometer-coordinates relative to body-coordinates as a rotation matrix
    RealMat<3,3> const accelerometer_noise_covar; ///< Covariance of the accelerometer white noise
    RealMat<3,3> const accelerometer_drift_covar; ///< Covariance of the accelerometer random walk (continuous-time)
    // Gyroscope parameters
    RealMat<3,3> const gyroscope_rotmat; ///< Orientation of gyroscope-coordinates relative to body-coordinates as a rotation matrix
    RealMat<3,3> const gyroscope_noise_covar; ///< Covariance of the gyroscope white noise
    RealMat<3,3> const gyroscope_drift_covar; ///< Covariance of the gyroscope random walk (continuous-time)
    // Environment parameters
    RealMat<3,3> const wind_drift_covar; ///< Covariance of the wind random walk (continuous-time)
    Real const pressure_drift_stdev; ///< Standard-deviation of the atmospheric pressure random walk (continuous-time)
    Real const temperature; ///< Absolute thermal temperature of the local atmosphere (assumed uniform and constant)
    //@}

    /// @name Derived
    //@{
    Real const specific_energy; ///< Memoized product of the specific gas constant and temperature
    //@}

public:
    /// @name Core
    //@{
    /// Constructs using a configuration YAML that recursively specifies all the "Given" attributes
    IMU(YAML::Node const& cfg);
    /// Input is a 6-vector of the accelerometer measurement stacked above the gyroscope measurement
    bool evolve(State& state, RealVecN const& input, Real timestep) const override;
    //@}

    /// @name Access
    //@{
    inline RealMat<3,3> const& get_accelerometer_rotmat()      const {return accelerometer_rotmat;}
    inline RealMat<3,3> const& get_accelerometer_noise_covar() const {return accelerometer_noise_covar;}
    inline RealMat<3,3> const& get_accelerometer_drift_covar() const {return accelerometer_drift_covar;}
    inline RealMat<3,3> const& get_gyroscope_rotmat()          const {return gyroscope_rotmat;}
    inline RealMat<3,3> const& get_gyroscope_noise_covar()     const {return gyroscope_noise_covar;}
    inline RealMat<3,3> const& get_gyroscope_drift_covar()     const {return gyroscope_drift_covar;}
    inline RealMat<3,3> const& get_wind_drift_covar()          const {return wind_drift_covar;}
    inline         Real const& get_pressure_drift_stdev()      const {return pressure_drift_stdev;}
    inline         Real const& get_temperature()               const {return temperature;}
    //@}
};

//////////////////////////////////////////////////

} // namespace processes
} // namespace navigation
} // namespace unitair
