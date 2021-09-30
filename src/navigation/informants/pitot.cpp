/** @file
Implementation of the Pitot class.
*/
#include "navigation/informants/pitot.hpp"

namespace unitair {
namespace navigation {
namespace informants {

//////////////////////////////////////////////////

Pitot::Pitot(YAML::Node const& cfg) :
    Informant(cfg, 1),
    dir(tools::realvec_from_yaml(tools::yget("dir", cfg), 3).normalized()),
    noise_stdev(tools::yget("noise_stdev", cfg).as<Real>()) {
    // Prepare the Jacobian by evaluating constant entries and leaving placeholders for variable entries
    J.resize(1, State::DIM);
    ////
    // dobs/dvel = dir
    J.set_block<1,3>(dir, 0, State::VEL_X);
    // dobs/dori = ?
    J.set_block_holder<1,3>(0, State::ORI_X);
    // dobs/dwind = ?
    J.set_block_holder<1,3>(0, State::WIND_X);
    ////
    J.initialize(tools::EPS);
    // Prepare the observation noise covariance matrix
    C.resize(1, 1);
    C.set_value(pow(noise_stdev,2), 0, 0);
    C.initialize(tools::EPS);
}

//////////////////////////////////////////////////

bool Pitot::fuse(State& state, RealVecN const& info) const {
    // Verify info
    if ((info.rows() != info_dim) or (info(0) < -tools::EPS)) {return false;}
    // Compute the expected measurement
    RealVec<3> const wind_body = state.ori.conjugate() * state.wind;
    RealVec<1> const expected(dir.dot(state.vel - wind_body));
    // Update the linearization
    J.set_block<1,3>(wind_body.cross(dir), 0, State::ORI_X); // dobs/dori
    J.set_block<1,3>(-(state.ori*dir), 0, State::WIND_X); // dobs/dwind
    // Correct the state based on the discrepancy
    return EKF::correct_state(state, info - expected);
}

//////////////////////////////////////////////////

} // namespace informants
} // namespace navigation
} // namespace unitair
