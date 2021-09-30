/** @file
Declaration of the Allocator and supporting classes.
*/
#pragma once
#include "tools/tools.hpp"

namespace unitair {
namespace control {

//////////////////////////////////////////////////

/// Model of a wrench-generating commandable device
class Actuator {
    /// @name Given
    //@{
    RealVec<3> center; ///< Position in body-coordinates of where the wrench is applied (DEFAULT: [0,0,0])
    RealVec<3> linvec; ///< Basis-vector in body-coordinates of the force part of the wrench (DEFAULT: [0,0,0])
    RealVec<3> angvec; ///< Basis-vector in body-coordinates of the torque part of the wrench (DEFAULT: [0,0,0])
    Real coeff1; ///< First-order positive coefficient for force from effort (DEFAULT: 0)
    Real coeff2; ///< Second-order positive coefficient for force from effort (DEFAULT: 0)
    Real mineff; ///< Minimum allowed effort (DEFAULT: -1)
    Real maxeff; ///< Maximum allowed effort (DEFAULT: +1)
    //@}

    /// @name Derived
    //@{
    RealVec<6> basis; ///< Basis-vector for the entire wrench (scalar * [linvec, angvec + center X linvec])
    //@}

public:
    /// @name Core
    //@{
    /// Constructs using a configuration YAML that recursively specifies all the "Given" attributes
    Actuator(YAML::Node const& cfg, RealVec<3> const& com=RealVec<3>::Zero());
    //@}

    /// @name Utilities
    //@{
    Real scalar_at_effort(Real eff) const; ///< Computes the wrench scalar corresponding to the given effort value
    Real effort_at_scalar(Real scl) const; ///< Computes the bounded effort value corresponding to the given wrench scalar
    //@}

    /// @name Access
    //@{
    inline Real get_mineff() const {return mineff;}
    inline Real get_maxeff() const {return maxeff;}
    inline RealVec<6> const& get_basis() const {return basis;}
    //@}
};

//////////////////////////////////////////////////

/// Solves for an optimal allocation of actuator efforts to achieve a desired wrench
class Allocator {
    /// @name Given
    //@{
    RealVec<3> com; ///< Position in body-coordinates of the center-of-mass
    Seq<Actuator> actuators; ///< List of components that generate wrenches
    //@}

    /// @name Derived
    //@{
    /// Decomposition of the complete actuator wrench basis for solving linear least-squares
    Eigen::CompleteOrthogonalDecomposition<RealMat<6, Eigen::Dynamic>> decomp;
    //@}

public:
    /// @name Core
    //@{
    /// Constructs using a configuration YAML that recursively specifies all the "Given" attributes
    Allocator(YAML::Node const& cfg);
    //@}

    /// @name Utilities
    //@{
    /// Computes the actuator efforts that best achieve the specified wrench in a least-squares sense
    RealVecN allocate(RealVec<6> const& wrench) const;
    //@}

    /// @name Access
    //@{
    inline Seq<Actuator> const& get_actuators() const {return actuators;}
    inline Eigen::CompleteOrthogonalDecomposition<RealMat<6, Eigen::Dynamic>> const& get_decomp() const {return decomp;}
    //@}
};

//////////////////////////////////////////////////

} // namespace control
} // namespace unitair
