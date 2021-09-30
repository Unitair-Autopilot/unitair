/** @file
Geodetic coordinate transformation and field calculation tools.
*/
#pragma once
#include "tools/general.hpp"
#include "tools/arithmetic.hpp"
#include "tools/geometry.hpp"

namespace unitair {
namespace tools {

//////////////////////////////////////////////////

/// Container for a latitude-longitude-altitude coordinate
struct LLA {
    /// @name Values
    //@{
    Real lat; ///< Latitude, [rad]
    Real lon; ///< Longitude, [rad]
    Real alt; ///< Altitude
    //@}
    /// Constructs uninitialized
    LLA() =default;
    /// Constructs with the given values
    LLA(Real lat, Real lon, Real alt) : lat(lat), lon(lon), alt(alt) {}
    /// Pretty-prints the values, with angles in degrees
    inline void print() const {std::cout << "lat: " << RAD2DEG*lat << " | lon: " << RAD2DEG*lon << " | alt: " << alt << std::endl;}
};

//////////////////////////////////////////////////

/// Provides transformations between coordinates for an oblate-spheroid model of the globe
/// (note: all latitude and longitude angles must be in radians)
class Geoid {
    /// @name Given
    //@{
    LLA home_lla; ///< Latitude, longitude, and altitude of the local origin (with angles in radians)
    Real const a; ///< Semi-major axis
    Real const f; ///< Flattening factor
    //@}

    /// @name Derived
    //@{
    Real const b; ///< Semi-minor axis
    Real const e2; ///< Eccentricity squared
    Real const ep2; ///< Second-eccentricity squared
    Quat const enu_ned; ///< Orientation of east-north-up relative to north-east-down coordinates
    Quat const ned_enu; ///< Inverse of enu_ned
    RealVec<3> home_ecef; ///< Position of the local origin in earth-centered-earth-fixed coordinates
    Quat enu_ecef; ///< Orientation of east-north-up relative to earth-centered-earth-fixed coordinates
    Quat ned_ecef; ///< Orientation of north-east-down relative to earth-centered-earth-fixed coordinates
    Quat ecef_enu; ///< Inverse of enu_ecef
    Quat ecef_ned; ///< Inverse of ned_ecef
    //@}

public:
    /// @name Core
    //@{
    /// Constructs with the given values (defaults are the WGS84 standard)
    Geoid(LLA const& init_home_lla, Real a=6378137.0, Real f=1.0/298.257223563) :
        a(a),
        f(f),
        b(a*(1.0-f)),
        e2(f*(2.0-f)),
        ep2(e2/(1.0-e2)),
        enu_ned(0.0, sqrt(2.0)/2.0, sqrt(2.0)/2.0, 0.0),
        ned_enu(enu_ned.inverse()) {
        reset_home_lla(init_home_lla);
    }
    /// Sets the local origin and updates all derived quantities accordingly
    void reset_home_lla(LLA const& other_home_lla) {
        home_lla = other_home_lla;
        home_ecef = ecef_from_lla(home_lla);
        enu_ecef = enu_at_lla(home_lla);
        ned_ecef = ned_at_lla(home_lla);
        ecef_enu = enu_ecef.inverse();
        ecef_ned = ned_ecef.inverse();
    }
    //@}

    /// @name Utilities
    //@{
    /////////////////////////

    /// @name ECEF <-> LLA
    //@{
    /// Converts an LLA to a position in ECEF coordinates
    RealVec<3> ecef_from_lla(LLA const& lla) const {
        Real const c_lat = cos(lla.lat);
        Real const s_lat = sin(lla.lat);
        Real const c_lon = cos(lla.lon);
        Real const s_lon = sin(lla.lon);
        Real const n = a/sqrt(1.0-e2*pow(s_lat,2));
        return RealVec<3>((n+lla.alt)*c_lat*c_lon, (n+lla.alt)*c_lat*s_lon, (n*b*b/(a*a)+lla.alt)*s_lat);
    }
    /// Converts a position in ECEF coordinates to an LLA
    LLA lla_from_ecef(RealVec<3> const& pos) const {
        LLA lla;
        Real const r = sqrt(pos(0)*pos(0) + pos(1)*pos(1));
        Real const t = atan2(a*pos(2), b*r);
        lla.lat = atan2(pos(2)+ep2*b*pow(sin(t),3), r-e2*a*pow(cos(t),3));
        lla.lon = fmod(atan2(pos(1), pos(0)), 2.0*PI);
        if(fabs(pos(0))<1.0 && fabs(pos(1))<1.0) {lla.alt = fabs(pos(2)) - b;}
        else {lla.alt = r/cos(lla.lat) - a/sqrt(1.0-e2*pow(sin(lla.lat),2));}
        return lla;
    }
    //@}

    /// @name ECEF <-> ENU
    //@{
    /// Converts a position in ENU coordinates to ECEF coordinates
    RealVec<3> ecef_from_enu(RealVec<3> const& pos) const {return home_ecef + enu_ecef*pos;}
    /// Converts a position in ECEF coordinates to ENU coordinates
    RealVec<3> enu_from_ecef(RealVec<3> const& pos) const {return ecef_enu*(pos - home_ecef);}
    //@}

    /// @name LLA <-> ENU
    //@{
    /// Converts a position in ENU coordinates to an LLA
    LLA lla_from_enu(RealVec<3> const& pos) const {return lla_from_ecef(ecef_from_enu(pos));}
    /// Converts an LLA to a position in ENU coordinates
    RealVec<3> enu_from_lla(LLA const& lla) const {return enu_from_ecef(ecef_from_lla(lla));}
    //@}

    /// @name NED <-> ENU
    //@{
    /// Converts a vector in ENU coordinates to NED coordinates
    RealVec<3> ned_from_enu(RealVec<3> const& vec) const {return enu_ned*vec;}
    /// Converts a vector in NED coordinates to ENU coordinates
    RealVec<3> enu_from_ned(RealVec<3> const& vec) const {return ned_enu*vec;}
    //@}

    /// @name ECEF <-> NED
    //@{
    /// Converts a position in NED coordinates to ECEF coordinates
    RealVec<3> ecef_from_ned(RealVec<3> const& pos) const {return home_ecef + ned_ecef*pos;}
    /// Converts a position in ECEF coordinates to NED coordinates
    RealVec<3> ned_from_ecef(RealVec<3> const& pos) const {return ecef_ned*(pos - home_ecef);}
    //@}

    /// @name LLA <-> NED
    //@{
    /// Converts a position in NED coordinates to an LLA
    LLA lla_from_ned(RealVec<3> const& pos) const {return lla_from_ecef(ecef_from_ned(pos));}
    /// Converts an LLA to a position in NED coordinates
    RealVec<3> ned_from_lla(LLA const& lla) const {return ned_from_ecef(ecef_from_lla(lla));}
    //@}

    /// @name LLA -> Local Orientation
    //@{
    /// Computes the orientation of ENU relative to ECEF at the given LLA
    Quat enu_at_lla(LLA const& lla) const {
        Real const hyaw = 0.5*(PI/2.0+lla.lon);
        Real const hroll = 0.5*(PI/2.0-lla.lat);
        Real const cy = cos(hyaw);
        Real const sy = sin(hyaw);
        Real const cr = cos(hroll);
        Real const sr = sin(hroll);
        return Quat(cy*cr, cy*sr, sy*sr, sy*cr);
    }
    /// Computes the orientation of NED relative to ECEF at the given LLA
    Quat ned_at_lla(LLA const& lla) const {return enu_at_lla(lla) * ned_enu;}
    //@}

    /////////////////////////
    //@}

    /// @name Access
    //@{
    inline LLA        const& get_home_lla()  const {return home_lla;}
    inline RealVec<3> const& get_home_ecef() const {return home_ecef;}
    inline Quat       const& get_enu_ecef()  const {return enu_ecef;}
    inline Quat       const& get_ned_ecef()  const {return ned_ecef;}
    inline Quat       const& get_ecef_enu()  const {return ecef_enu;}
    inline Quat       const& get_ecef_ned()  const {return ecef_ned;}
    //@}
};

//////////////////////////////////////////////////

} // namespace tools
} // namespace unitair
