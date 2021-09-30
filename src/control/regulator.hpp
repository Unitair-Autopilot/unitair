/** @file
Declaration of the Regulator.
*/
#pragma once
#include "tools/tools.hpp"

namespace unitair {
namespace control {

//////////////////////////////////////////////////

/// Computes torques that reject attitude tracking error via adaptive feedback
class Regulator {
    /// @name Given
    //@{
    // Physical parameters
    RealMat<3,3> inertia_matrix; ///< Inertia tensor expressed in body-coordinates about the center-of-mass
    RealVec<3> aero_axis; ///< Natural direction of forward flight in body-coordinates (DEFAULT: [0,0,0] implies no aerospring effects)
    Real min_airvel; ///< Minimum air-relative velocity along aero_axis below which there is insufficient attitude control authority (DEFAULT: -infinity)
    // Tuning knobs
    RealVec<3> ori_feedback_gains;    ///< Sensitivities to orientation error in body-coordinates
    RealVec<3> angvel_feedback_gains; ///< Sensitivities to angular velocity error in body-coordinates
    RealVec<3> aerospring_adaptive_gains;  ///< Learning-rates for the restorative aerodynamic torque model in body-coordinates (DEFAULT: [0,0,0])
    RealVec<3> aerodamper_adaptive_gains;  ///< Learning-rates for the dissipative aerodynamic torque model in body-coordinates
    RealVec<3> disturbance_adaptive_gains; ///< Learning-rates for the exogenous or actuator-induced disturbance torque in body-coordinates, i.e. traditional integral gain
    RealVec<3> aerospring_adaptive_means;  ///< Expectation for the restorative aerodynamic torque model coefficients in body-coordinates (DEFAULT: [0,0,0])
    RealVec<3> aerodamper_adaptive_means;  ///< Expectation for the dissipative aerodynamic torque model coefficients in body-coordinates (DEFAULT: [0,0,0])
    RealVec<3> disturbance_adaptive_means; ///< Expectation for the exogenous or actuator-induced disturbance torque in body-coordinates (DEFAULT: [0,0,0])
    Real amnesia; ///< Rate of mean-reversion for all adaptations, i.e. integral leakiness (DEFAULT: 0)
    Real filter; ///< Time-constant of the first-order IIR low-pass filter on the tracking error signal (DEFAULT: 0 implies no filter)
    //@}

    /// @name Derived
    //@{
    RealMat<3,3> inertia_inverse; ///< Memoized inverse of the inertia matrix
    RealVec<3> aerospring_coeffs; ///< Estimated parameters of the "torsion spring" effect that tends to align airspeed forward
    RealVec<3> aerodamper_coeffs; ///< Estimated parameters of rotational damping about each body axis
    RealVec<3> disturbance; ///< Estimate of the disturbance torque, i.e. traditional integrator state
    RealVec<3> angvel_error; ///< Current value of the angular velocity tracking error signal
    //@}

public:
    /// @name Core
    //@{
    /// Constructs using a configuration YAML that recursively specifies all the "Given" attributes
    Regulator(YAML::Node const& cfg);
    /// Computes a torque that drives the given attitude state towards the target, and adapts to the error
    RealVec<3> regulate(Quat       const& ori,
                        Quat       const& ori_target,
                        RealVec<3> const& angvel,
                        RealVec<3> const& angvel_target,
                        RealVec<3> const& angaccel_target,
                        RealVec<3> const& airvel,
                        Real timestep);
    //@}

    /// @name Access
    //@{
    inline RealMat<3,3> const& get_inertia_matrix()    const {return inertia_matrix;}
    inline RealVec<3>   const& get_aero_axis()         const {return aero_axis;}
    inline Real                get_min_airvel()        const {return min_airvel;}
    inline RealVec<3>   const& get_aerospring_coeffs() const {return aerospring_coeffs;}
    inline RealVec<3>   const& get_aerodamper_coeffs() const {return aerodamper_coeffs;}
    inline RealVec<3>   const& get_disturbance()       const {return disturbance;}
    inline RealVec<3>   const& get_angvel_error()      const {return angvel_error;}
    //@}
};

//////////////////////////////////////////////////

} // namespace control
} // namespace unitair
