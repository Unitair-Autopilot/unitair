/** @file
Declaration of the Barometer class.
*/
#pragma once
#include "navigation/estimator.hpp"

namespace unitair {
namespace navigation {
namespace informants {

//////////////////////////////////////////////////

/// Model for a static air pressure sensor, with inference via linearization
class Barometer : public Informant, public inference_methods::EKF {
    /// @name Given
    //@{
    Real const noise_stdev; ///< Standard-deviation of the measurement's additive white noise
    //@}

public:
    /// @name Core
    //@{
    /// Constructs using a configuration YAML that recursively specifies all the "Given" attributes
    Barometer(YAML::Node const& cfg);
    /// Info is the measured pressure scalar
    bool fuse(State& state, RealVecN const& info) const override;
    //@}

    /// @name Access
    //@{
    inline Real const& get_noise_stdev() const {return noise_stdev;}
    //@}
};

//////////////////////////////////////////////////

} // namespace informants
} // namespace navigation
} // namespace unitair
