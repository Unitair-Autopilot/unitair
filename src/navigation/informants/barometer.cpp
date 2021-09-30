/** @file
Implementation of the Barometer class.
*/
#include "navigation/informants/barometer.hpp"

namespace unitair {
namespace navigation {
namespace informants {

//////////////////////////////////////////////////

Barometer::Barometer(YAML::Node const& cfg) :
    Informant(cfg, 1),
    noise_stdev(tools::yget("noise_stdev", cfg).as<Real>()) {
    // Prepare the Jacobian
    J.resize(1, State::DIM);
    J.set_value(1.0, 0, State::PRESSURE);
    J.initialize(tools::EPS);
    // Prepare the observation noise covariance matrix
    C.resize(1, 1);
    C.set_value(pow(noise_stdev,2), 0, 0);
    C.initialize(tools::EPS);
}

//////////////////////////////////////////////////

bool Barometer::fuse(State& state, RealVecN const& info) const {
    // Verify info
    if (info.rows() != info_dim) {return false;}
    // Compute the expected measurement
    RealVec<1> const expected(state.pressure);
    // Correct the state based on the discrepancy
    return EKF::correct_state(state, info - expected);
}

//////////////////////////////////////////////////

} // namespace informants
} // namespace navigation
} // namespace unitair
