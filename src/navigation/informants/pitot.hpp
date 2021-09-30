/** @file
Declaration of the Pitot class.
*/
#pragma once
#include "navigation/estimator.hpp"

namespace unitair {
namespace navigation {
namespace informants {

//////////////////////////////////////////////////

/// Model for a Pitot-style (differential pressure) airspeed sensor, with inference via linearization
class Pitot : public Informant, public inference_methods::EKF {
    /// @name Given
    //@{
    RealVec<3> const dir; ///< Mounting direction in body-coordinates
    Real const noise_stdev; ///< Standard-deviation of the measurement's additive white noise
    //@}

public:
    /// @name Core
    //@{
    /// Constructs using a configuration YAML that recursively specifies all the "Given" attributes
    Pitot(YAML::Node const& cfg);
    /// Info is the measured airspeed scalar, which must be non-negative
    bool fuse(State& state, RealVecN const& info) const override;
    //@}

    /// @name Access
    //@{
    inline RealVec<3> const& get_dir()         const {return dir;}
    inline Real       const& get_noise_stdev() const {return noise_stdev;}
    //@}
};

//////////////////////////////////////////////////

} // namespace informants
} // namespace navigation
} // namespace unitair
