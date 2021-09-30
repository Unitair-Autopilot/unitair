/** @file
Implementation of the MotionCapture class.
*/
#include "navigation/informants/motion_capture.hpp"

namespace unitair {
namespace navigation {
namespace informants {

//////////////////////////////////////////////////

MotionCapture::MotionCapture(YAML::Node const& cfg) :
    Informant(cfg, 7),
    pos_origin(tools::realvec_from_yaml(tools::yget("pos_origin", cfg), 3)),
    ori_origin(tools::quat_from_yaml(tools::yget("ori_origin", cfg))),
    pos_offset(tools::realvec_from_yaml(tools::yget("pos_offset", cfg), 3)),
    ori_offset(tools::quat_from_yaml(tools::yget("ori_offset", cfg))),
    noise_stdevs(tools::realvec_from_yaml(tools::yget("noise_stdevs", cfg), 6)),
    pos_offset_crossmat_tr(tools::crossmat(pos_offset).transpose()) {
    // Prepare the Jacobian by evaluating constant entries and leaving placeholders for variable entries
    J.resize(6, State::DIM);
    ////
    // dobs_pos/dpos = I
    J.set_diagonal<3>(1.0, 0, State::POS_X);
    // dobs_pos/dori = ?
    J.set_block_holder<3,3>(0, State::ORI_X);
    // dobs_ori/dori = ori_offset^-1
    J.set_block<3,3>(ori_offset.toRotationMatrix().transpose(), 3, State::ORI_X);
    ////
    J.initialize(tools::EPS);
    // Prepare the observation noise covariance matrix
    C.resize(6, 6);
    C.set_diagonal<6>(noise_stdevs.cwiseAbs2(), 0, 0);
    C.initialize(tools::EPS);
}

//////////////////////////////////////////////////

bool MotionCapture::fuse(State& state, RealVecN const& info) const {
    // Verify info
    if (info.rows() != info_dim) {return false;}
    // Express the measurement in world-coordinates
    RealVec<3> const measured_pos = pos_origin + ori_origin*info.head<3>();
    Quat const measured_ori = ori_origin * Quat(info(3), info(4), info(5), info(6));
    // Compute the expected measurement
    RealVec<3> const expected_pos = state.pos + state.ori*pos_offset;
    Quat const expected_ori = state.ori * ori_offset;
    // Compute the discrepancy vector
    RealVec<6> discrepancy;
    discrepancy << measured_pos - expected_pos,
                   tools::qlog(expected_ori.conjugate() * measured_ori);
    // Update the linearization
    J.set_block<3,3>(state.ori.toRotationMatrix()*pos_offset_crossmat_tr, 0, State::ORI_X); // dobs_pos/dori
    // Correct the state
    return EKF::correct_state(state, discrepancy);
}

//////////////////////////////////////////////////

} // namespace informants
} // namespace navigation
} // namespace unitair
