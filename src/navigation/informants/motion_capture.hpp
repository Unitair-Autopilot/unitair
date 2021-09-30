/** @file
Declaration of the MotionCapture class.
*/
#pragma once
#include "navigation/estimator.hpp"

namespace unitair {
namespace navigation {
namespace informants {

//////////////////////////////////////////////////

/// Sensor model for an offboard motion-capture system that tracks
/// the pose of the body, with inference via linearization
class MotionCapture : public Informant, public inference_methods::EKF {
    /// @name Given
    //@{
    // Origin parameters
    RealVec<3> const pos_origin; ///< Position in world-coordinates of the measurement pose-origin
          Quat const ori_origin; ///< Orientation of the measurement pose-origin relative to the world-coordinates
    // Offset parameters
    RealVec<3> const pos_offset; ///< Position in body-coordinates of the tracked body-fixed point
          Quat const ori_offset; ///< Orientation of the mocap's assigned coordinates relative to the body-coordinates
    // Noise parameters
    RealVec<6> const noise_stdevs; ///< Standard-deviations of the measurement's additive white noise (translation then rotation) (crude loose-coupling assumption)
    //@}

    /// @name Derived
    //@{
    RealMat<3,3> const pos_offset_crossmat_tr; ///< Cross-product matrix form of pos_offset, negated
    //@}

public:
    /// @name Core
    //@{
    /// Constructs using a configuration YAML that recursively specifies all the "Given" attributes
    MotionCapture(YAML::Node const& cfg);
    /// Info is a pose represented by the position 3-vector concatenated with the [w, x, y, z] orientation quaternion coefficients
    bool fuse(State& state, RealVecN const& info) const override;
    //@}

    /// @name Access
    //@{
    inline RealVec<3> const&   get_pos_origin() const {return pos_origin;}
    inline       Quat const&   get_ori_origin() const {return ori_origin;}
    inline RealVec<3> const&   get_pos_offset() const {return pos_offset;}
    inline       Quat const&   get_ori_offset() const {return ori_offset;}
    inline RealVec<6> const& get_noise_stdevs() const {return noise_stdevs;}
    //@}
};

//////////////////////////////////////////////////

} // namespace informants
} // namespace navigation
} // namespace unitair
