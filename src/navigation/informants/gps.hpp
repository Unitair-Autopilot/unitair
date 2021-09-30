/** @file
Declaration of the GPS class.
*/
#pragma once
#include "navigation/estimator.hpp"

namespace unitair {
namespace navigation {
namespace informants {

//////////////////////////////////////////////////

/// Sensor model for a loosely-coupled GPS receiver that treats ENU
/// as world-coordinates, with inference via linearization
class GPS : public Informant, public inference_methods::EKF {
    /// @name Given
    //@{
    // Origin parameters
    Real const home_lat_deg; ///< Latitude of the world-origin in degrees
    Real const home_lon_deg; ///< Longitude of the world-origin in degrees
    Real const home_alt_m;   ///< Altitude of the world-origin in meters
    // Offset parameters
    RealVec<3> const pos_offset; ///< Position in body-coordinates of the GPS antenna
    // Noise parameters
    RealVec<3> const noise_stdevs; ///< Standard-deviations of the measurement's additive white noise in world-coordinates (crude loose-coupling assumption)
    //@}

    /// @name Derived
    //@{
    tools::Geoid const geoid; ///< Provides conversion from LLA to ENU
    RealMat<3,3> const pos_offset_crossmat_tr; ///< Cross-product matrix form of pos_offset, negated
    //@}

public:
    /// @name Core
    //@{
    /// Constructs using a configuration YAML that recursively specifies all the "Given" attributes
    GPS(YAML::Node const& cfg);
    /// Info is the measured latitude, longitude, and altitude as a 3-vector in that order, with angles in degrees
    bool fuse(State& state, RealVecN const& info) const override;
    //@}

    /// @name Access
    //@{
    inline         Real const& get_home_lat_deg() const {return home_lat_deg;}
    inline         Real const& get_home_lon_deg() const {return home_lon_deg;}
    inline         Real const&   get_home_alt_m() const {return home_alt_m;}
    inline   RealVec<3> const&   get_pos_offset() const {return pos_offset;}
    inline   RealVec<3> const& get_noise_stdevs() const {return noise_stdevs;}
    inline tools::Geoid const&        get_geoid() const {return geoid;}
    //@}
};

//////////////////////////////////////////////////

} // namespace informants
} // namespace navigation
} // namespace unitair
