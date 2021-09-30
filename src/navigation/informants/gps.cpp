/** @file
Implementation of the GPS class.
*/
#include "navigation/informants/gps.hpp"

namespace unitair {
namespace navigation {
namespace informants {

//////////////////////////////////////////////////

GPS::GPS(YAML::Node const& cfg) :
    Informant(cfg, 3),
    home_lat_deg(tools::yget("home_lat_deg", cfg).as<Real>()),
    home_lon_deg(tools::yget("home_lon_deg", cfg).as<Real>()),
    home_alt_m(tools::yget("home_alt_m", cfg).as<Real>()),
    pos_offset(tools::realvec_from_yaml(tools::yget("pos_offset", cfg), 3)),
    noise_stdevs(tools::realvec_from_yaml(tools::yget("noise_stdevs", cfg), 3)),
    geoid(tools::LLA(tools::DEG2RAD*home_lat_deg, tools::DEG2RAD*home_lon_deg, home_alt_m)),
    pos_offset_crossmat_tr(tools::crossmat(pos_offset).transpose()) {
    // Prepare the Jacobian by evaluating constant entries and leaving placeholders for variable entries
    J.resize(3, State::DIM);
    ////
    // dobs/dpos = I
    J.set_diagonal<3>(1.0, 0, State::POS_X);
    // dobs/dori = ?
    J.set_block_holder<3,3>(0, State::ORI_X);
    ////
    J.initialize(tools::EPS);
    // Prepare the observation noise covariance matrix
    C.resize(3, 3);
    C.set_diagonal<3>(noise_stdevs.cwiseAbs2(), 0, 0);
    C.initialize(tools::EPS);
}

//////////////////////////////////////////////////

bool GPS::fuse(State& state, RealVecN const& info) const {
    // Verify info
    if (info.rows() != info_dim) {return false;}
    // Express the measurement in world-coordinates
    tools::LLA const measured_lla = tools::LLA(tools::DEG2RAD*info(0), tools::DEG2RAD*info(1), info(2));
    RealVec<3> const measured_pos = geoid.enu_from_lla(measured_lla);
    // Compute the expected measurement
    RealVec<3> const expected_pos = state.pos + state.ori*pos_offset;
    // Update the linearization
    J.set_block<3,3>(state.ori.toRotationMatrix()*pos_offset_crossmat_tr, 0, State::ORI_X); // dobs/dori
    // Correct the state based on the discrepancy
    return EKF::correct_state(state, measured_pos - expected_pos);
}

//////////////////////////////////////////////////

} // namespace informants
} // namespace navigation
} // namespace unitair
