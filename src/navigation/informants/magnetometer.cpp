/** @file
Implementation of the Magnetometer class.
*/
#include "navigation/informants/magnetometer.hpp"

namespace unitair {
namespace navigation {
namespace informants {

//////////////////////////////////////////////////

Magnetometer::Magnetometer(YAML::Node const& cfg) :
    Informant(cfg, 3),
    rotmat(tools::realmat_from_yaml(tools::yget("rotmat", cfg), 3, 3)),
    soft_distort(tools::realmat_from_yaml(tools::yget("soft_distort", cfg), 3, 3)),
    hard_distort(tools::realvec_from_yaml(tools::yget("hard_distort", cfg), 3)),
    noise_covar(tools::realmat_from_yaml(tools::yget("noise_covar", cfg), 3, 3)),
    field(tools::realvec_from_yaml(tools::yget("field", cfg), 3)),
    soft_distort_rotmat_inv(soft_distort * rotmat.inverse()) {
    // Prepare the Jacobian by leaving placeholders for variable entries
    J.resize(3, State::DIM);
    J.set_block_holder<3,3>(0, State::ORI_X);
    J.initialize(tools::EPS);
    // Prepare the observation noise covariance matrix
    C.resize(3, 3);
    C.set_block<3,3>(noise_covar, 0, 0);
    C.initialize(tools::EPS);
}

//////////////////////////////////////////////////

bool Magnetometer::fuse(State& state, RealVecN const& info) const {
    // Verify info
    if (info.rows() != info_dim) {return false;}
    // Compute the reference field in body-coordinates
    RealVec<3> const reoriented = state.ori.conjugate() * field;
    // Compute the expected measurement
    RealVec<3> const expected = (soft_distort_rotmat_inv * reoriented) + hard_distort;
    // Update the linearization
    J.set_block<3,3>(soft_distort_rotmat_inv*tools::crossmat(reoriented), 0, State::ORI_X);
    // Correct the state based on the discrepancy
    return EKF::correct_state(state, info - expected);
}

//////////////////////////////////////////////////

} // namespace informants
} // namespace navigation
} // namespace unitair
