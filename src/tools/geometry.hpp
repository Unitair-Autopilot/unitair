/** @file
Geometry / Lie algebra tools.
All quaternions are assumed to have unit-norm.
*/
#pragma once
#include "tools/general.hpp"
#include "tools/arithmetic.hpp"

namespace unitair {
namespace tools {

//////////////////////////////////////////////////

/// Maps an angle (SO2 minimal representation) onto [-pi, pi] or [-180, 180]
inline Real amod(Real angle, bool is_degrees=false) {
    Real const half_circle = is_degrees ? copysign(180.0, angle) : copysign(PI, angle);
    return fmod(angle + half_circle, 2.0*half_circle) - half_circle;
}

//////////////////////////////////////////////////

/// Converts an angle*axis vector (in radians) to a unit-quaternion
inline Quat qexp(RealVec<3> const& r) {
    Real const half_angle = r.norm() / 2.0;
    Quat q; q.w() = cos(half_angle); q.vec() = (sinc(half_angle)/2.0)*r;
    return q;
}

/// Converts a unit-quaternion to an angle*axis vector (in radians)
inline RealVec<3> qlog(Quat const& q) {
    return (copysign(2.0, q.w()) / sinc(acos(std::min(1.0, fabs(q.w()))))) * q.vec();
}

//////////////////////////////////////////////////

/// Constructs the cross-product matrix operator form of the 3-vector r
inline RealMat<3,3> crossmat(RealVec<3> const& r) {
    RealMat<3,3> C;
    C <<  0.0, -r(2),  r(1),
         r(2),   0.0, -r(0),
        -r(1),  r(0),   0.0;
    return C;
}

/// Constructs the 3-vector representation of a 3-by-3 skew-symmetric matrix C
inline RealVec<3> crossvec(RealMat<3,3> const& C) {
    return RealVec<3>(C(2,1), C(0,2), C(1,0));
}

//////////////////////////////////////////////////

/// Euler angles in k-j'-i'' (aerospace, equivalent to i-j-k) convention
class Euler {
    /// @name Given / Derived
    //@{
    Real _roll; ///< Angle of simple rotation about i'' (last)
    Real _pitch; ///< Angle of simple rotation about j' (middle)
    Real _yaw; ///< Angle of simple rotation about k (first)
    bool _is_degrees; ///< Whether or not the angles should be interpreted as degrees or radians
    //@}

public:
    /// @name Core
    //@{
    /// Constructs with the given angles and unit
    Euler(Real roll, Real pitch, Real yaw, bool is_degrees=false) :
        _roll(roll),
        _pitch(pitch),
        _yaw(yaw),
        _is_degrees(is_degrees) {
    }
    /// Constructs based on the given unit-quaternion and unit
    Euler(Quat const& q, bool use_degrees=false) :
        _is_degrees(false) {
        _roll = atan2(2.0*(q.w()*q.x() + q.y()*q.z()), 1.0 - 2.0*(q.x()*q.x() + q.y()*q.y()));
        Real const sp = 2.0*(q.w()*q.y() - q.z()*q.x());
        _pitch = (fabs(sp) >= 1.0-EPS) ? copysign(PI/2.0, sp) : asin(sp);
        _yaw = atan2(2.0*(q.w()*q.z() + q.x()*q.y()), 1.0 - 2.0*(q.y()*q.y() + q.z()*q.z()));
        if (use_degrees) {rad2deg();}
    }
    /// Modifies in-place the unit from radians to degrees (idempotent)
    Euler& rad2deg() {if (not _is_degrees) {_roll *= RAD2DEG; _pitch *= RAD2DEG; _yaw *= RAD2DEG; _is_degrees = true;} return *this;}
    /// Modifies in-place the unit from degrees to radians (idempotent)
    Euler& deg2rad() {if (_is_degrees) {_roll *= DEG2RAD; _pitch *= DEG2RAD; _yaw *= DEG2RAD; _is_degrees = false;} return *this;}
    /// Modifies in-place the angles to their equivalent values on [-PI, PI] or [-180, 180]
    Euler& amod() {_roll = tools::amod(_roll, _is_degrees); _pitch = tools::amod(_pitch, _is_degrees); _yaw = tools::amod(_yaw, _is_degrees); return *this;}
    //@}

    /// @name Utilities
    //@{
    /// Computes the quaternion equivalent to these angles
    Quat quat() const {
        Real hr, hp, hy;
        if (_is_degrees) {hr = 0.5*DEG2RAD*_roll; hp = 0.5*DEG2RAD*_pitch; hy = 0.5*DEG2RAD*_yaw;}
        else {hr = 0.5*_roll; hp = 0.5*_pitch; hy = 0.5*_yaw;}
        Real const cr = cos(hr) , sr = sin(hr);
        Real const cp = cos(hp) , sp = sin(hp);
        Real const cy = cos(hy) , sy = sin(hy);
        Quat q(cy*cr*cp + sy*sr*sp, cy*sr*cp - sy*cr*sp, cy*cr*sp + sy*sr*cp, sy*cr*cp - cy*sr*sp);
        if (q.w() < 0.0) {q.coeffs() = -q.coeffs();}
        return q;
    }
    /// Pretty-prints the angles
    inline void print() const {std::cout << "roll: " << _roll << " | pitch: " << _pitch << " | yaw: " << _yaw << std::endl;}
    //@}

    /// @name Access
    //@{
    inline Real       roll() const {return _roll;}
    inline Real      pitch() const {return _pitch;}
    inline Real        yaw() const {return _yaw;}
    inline bool is_degrees() const {return _is_degrees;}
    //@}
};

//////////////////////////////////////////////////

} // namespace tools
} // namespace unitair
