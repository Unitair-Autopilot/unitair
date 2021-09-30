/** @file
Implementation of the Regulator.
*/
#include "control/regulator.hpp"

namespace unitair {
namespace control {

//////////////////////////////////////////////////

Regulator::Regulator(YAML::Node const& cfg) :
    inertia_matrix(tools::realmat_from_yaml(tools::yget("inertia_matrix", cfg), 3, 3)),
    aero_axis(tools::realvec_from_yaml(tools::yget("aero_axis", cfg, Seq<Real>({0,0,0})), 3).normalized()),
    min_airvel(tools::yget("min_airvel", cfg, -tools::YINF).as<Real>()),
    ori_feedback_gains(tools::realvec_from_yaml(tools::yget("ori_feedback_gains", cfg), 3)),
    angvel_feedback_gains(tools::realvec_from_yaml(tools::yget("angvel_feedback_gains", cfg), 3)),
    aerospring_adaptive_gains(tools::realvec_from_yaml(tools::yget("aerospring_adaptive_gains", cfg, Seq<Real>({0,0,0})), 3)),
    aerodamper_adaptive_gains(tools::realvec_from_yaml(tools::yget("aerodamper_adaptive_gains", cfg), 3)),
    disturbance_adaptive_gains(tools::realvec_from_yaml(tools::yget("disturbance_adaptive_gains", cfg), 3)),
    aerospring_adaptive_means(tools::realvec_from_yaml(tools::yget("aerospring_adaptive_means", cfg, Seq<Real>({0,0,0})), 3)),
    aerodamper_adaptive_means(tools::realvec_from_yaml(tools::yget("aerodamper_adaptive_means", cfg, Seq<Real>({0,0,0})), 3)),
    disturbance_adaptive_means(tools::realvec_from_yaml(tools::yget("disturbance_adaptive_means", cfg, Seq<Real>({0,0,0})), 3)),
    amnesia(tools::yget("amnesia", cfg, 0).as<Real>()),
    filter(tools::yget("filter", cfg, 0).as<Real>()),
    inertia_inverse(inertia_matrix.inverse()),
    aerospring_coeffs(aerospring_adaptive_means),
    aerodamper_coeffs(aerodamper_adaptive_means),
    disturbance(disturbance_adaptive_means),
    angvel_error(0, 0, 0) {
    // Validate inertia
    if (inertia_matrix.diagonal().minCoeff() <= 0) {throw std::runtime_error("Regulator inertia_matrix is not positive-definite.");} // weak check
    if (not inertia_inverse.allFinite()) {throw std::runtime_error("Regulator inertia_matrix is not invertible.");}
    // Validate gains
    if (ori_feedback_gains.minCoeff() < 0) {throw std::runtime_error("Regulator ori_feedback_gains cannot be negative.");}
    if (angvel_feedback_gains.minCoeff() < 0) {throw std::runtime_error("Regulator angvel_feedback_gains cannot be negative.");}
    if (aerospring_adaptive_gains.minCoeff() < 0) {throw std::runtime_error("Regulator aerospring_adaptive_gains cannot be negative.");}
    if (aerodamper_adaptive_gains.minCoeff() < 0) {throw std::runtime_error("Regulator aerodamper_adaptive_gains cannot be negative.");}
    if (disturbance_adaptive_gains.minCoeff() < 0) {throw std::runtime_error("Regulator disturbance_adaptive_gains cannot be negative.");}
    if (amnesia < 0) {throw std::runtime_error("Regulator amnesia cannot be negative.");}
    if (filter < 0) {throw std::runtime_error("Regulator filter cannot be negative.");}
}

/////////////////////////

RealVec<3> Regulator::regulate(Quat       const& ori,
                               Quat       const& ori_target,
                               RealVec<3> const& angvel,
                               RealVec<3> const& angvel_target,
                               RealVec<3> const& angaccel_target,
                               RealVec<3> const& airvel,
                               Real timestep) {
    // Tracking errors
    RealVec<3> const ori_error = tools::qlog(ori.conjugate() * ori_target); // so3 of body_from_world * world_from_targetbody
    RealVec<3> const angvel_error_raw = (angvel_target - angvel) + ori_feedback_gains.cwiseProduct(ori_error); // emulates an outer-loop on orientation
    if (filter > timestep) {angvel_error += timestep * (angvel_error_raw - angvel_error) / filter;}
    else {angvel_error = angvel_error_raw;}
    // Feedforward-model basis vectors
    RealVec<3> const precession_basis = -angvel.cross(inertia_matrix*angvel);
    RealVec<3> const aerospring_basis = aero_axis.cross(airvel);
    RealVec<3> const aerodamper_basis = -angvel;
    // Lyapunov-gradient adaptation
    if (airvel.dot(aero_axis) >= min_airvel) {
        RealVec<3> const dual = inertia_inverse * angvel_error;
        aerospring_coeffs -= timestep * aerospring_adaptive_gains.cwiseProduct(aerospring_basis.cwiseProduct(dual));
        aerodamper_coeffs -= timestep * aerodamper_adaptive_gains.cwiseProduct(aerodamper_basis.cwiseProduct(dual));
        disturbance       -= timestep * disturbance_adaptive_gains.cwiseProduct(dual);
    }
    // Adaptation mean-reversion
    Real const leak = tools::clip(amnesia * timestep, 0.0, 1.0);
    aerospring_coeffs += leak * (aerospring_adaptive_means - aerospring_coeffs);
    aerodamper_coeffs += leak * (aerodamper_adaptive_means - aerodamper_coeffs);
    disturbance       += leak * (disturbance_adaptive_means - disturbance);
    // Prevent nonphysical solutions
    aerodamper_coeffs = aerodamper_coeffs.cwiseMax(0.0);
    if (not angvel_error.allFinite()) {angvel_error.setZero();}
    if (not aerospring_coeffs.allFinite()) {aerospring_coeffs.setZero();}
    if (not aerodamper_coeffs.allFinite()) {aerodamper_coeffs.setZero();}
    if (not disturbance.allFinite()) {disturbance.setZero();}
    // Approximate feedback-linearizing control policy for rotational mechanics
    RealVec<3> const model = precession_basis + aerospring_coeffs.cwiseProduct(aerospring_basis) + aerodamper_coeffs.cwiseProduct(aerodamper_basis) + disturbance;
    return inertia_matrix*(angaccel_target + angvel_feedback_gains.cwiseProduct(angvel_error)) - model;
}

//////////////////////////////////////////////////

} // namespace control
} // namespace unitair
