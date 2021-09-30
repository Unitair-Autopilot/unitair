/** @file
Declaration of the Magnetometer class.
*/
#pragma once
#include "navigation/estimator.hpp"

namespace unitair {
namespace navigation {
namespace informants {

//////////////////////////////////////////////////

/// Model for a three-axis magnetic field sensor, with inference via linearization
/// (note: only valid within a few kilometers of origin due to assumption of constant reference field)
class Magnetometer : public Informant, public inference_methods::EKF {
    /// @name Given
    //@{
    // Origin parameters
    RealMat<3,3> const rotmat; ///< Orientation of sensor-coordinates relative to body-coordinates as a rotation matrix
    // Offset parameters
    RealMat<3,3> const soft_distort; ///< Soft-iron distortion matrix in sensor-coordinates
    RealVec<3>   const hard_distort; ///< Hard-iron distortion vector in sensor-coordinates
    // Noise parameters
    RealMat<3,3> const noise_covar; ///< Covariance of the measurement's additive white noise
    // Environment parameters
    RealVec<3> field; ///< Reference external magnetic field in world-coordinates
    //@}

    /// @name Derived
    //@{
    RealMat<3,3> const soft_distort_rotmat_inv; ///< The product of soft_distort and the inverse of rotmat
    //@}

public:
    /// @name Core
    //@{
    /// Constructs using a configuration YAML that recursively specifies all the "Given" attributes
    Magnetometer(YAML::Node const& cfg);
    /// Info is the measurement 3-vector
    bool fuse(State& state, RealVecN const& info) const override;
    //@}

    /// @name Access
    //@{
    inline RealMat<3,3> const& get_rotmat()       const {return rotmat;}
    inline RealMat<3,3> const& get_soft_distort() const {return soft_distort;}
    inline RealVec<3>   const& get_hard_distort() const {return hard_distort;}
    inline RealMat<3,3> const& get_noise_covar()  const {return noise_covar;}
    inline RealVec<3>   const& get_field()        const {return field;}
    inline void set_field(RealVec<3> const& new_field) {field = new_field;}
    //@}
};

//////////////////////////////////////////////////

} // namespace informants
} // namespace navigation
} // namespace unitair
