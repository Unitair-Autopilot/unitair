/** @file
Implementation of the IMU class.
*/
#include "navigation/processes/imu.hpp"

namespace unitair {
namespace navigation {
namespace processes {

//////////////////////////////////////////////////

IMU::IMU(YAML::Node const& cfg) :
    Process(cfg, 6),
    accelerometer_rotmat(tools::realmat_from_yaml(tools::yget("accelerometer_rotmat", cfg), 3, 3)),
    accelerometer_noise_covar(tools::realmat_from_yaml(tools::yget("accelerometer_noise_covar", cfg), 3, 3)),
    accelerometer_drift_covar(tools::realmat_from_yaml(tools::yget("accelerometer_drift_covar", cfg), 3, 3)),
    gyroscope_rotmat(tools::realmat_from_yaml(tools::yget("gyroscope_rotmat", cfg), 3, 3)),
    gyroscope_noise_covar(tools::realmat_from_yaml(tools::yget("gyroscope_noise_covar", cfg), 3, 3)),
    gyroscope_drift_covar(tools::realmat_from_yaml(tools::yget("gyroscope_drift_covar", cfg), 3, 3)),
    wind_drift_covar(tools::realmat_from_yaml(tools::yget("wind_drift_covar", cfg), 3, 3)),
    pressure_drift_stdev(tools::yget("pressure_drift_stdev", cfg).as<Real>()),
    temperature(tools::yget("temperature", cfg).as<Real>()),
    specific_energy(287.052874*temperature) {
    // Prepare the Jacobian by evaluating constant entries and leaving placeholders for variable entries
    J.resize(State::DIM, State::DIM);
    ////
    // dpos'/dpos = I
    J.set_diagonal<3>(1.0, State::POS_X, State::POS_X);
    // dpos'/dvel = ?
    J.set_block_holder<3,3>(State::POS_X, State::VEL_X);
    // dpos'/daccel = ?
    J.set_block_holder<3,3>(State::POS_X, State::ACCEL_X);
    // dpos'/dori = ?
    J.set_block_holder<3,3>(State::POS_X, State::ORI_X);
    // dvel'/dvel = I
    J.set_diagonal<3>(1.0, State::VEL_X, State::VEL_X);
    // dvel'/daccel = ?*I
    J.set_diagonal_holder<3>(State::VEL_X, State::ACCEL_X);
    // daccel'/dvel = ? = -?^T
    J.set_skewsym_holder(State::ACCEL_X, State::VEL_X);
    // daccel'/dori = ? = -?^T
    J.set_skewsym_holder(State::ACCEL_X, State::ORI_X);
    // daccel'/dangvel = ? = -?^T
    J.set_skewsym_holder(State::ACCEL_X, State::ANGVEL_X);
    // daccel'/dbias_accel = -accelerometer_rotmat
    J.set_block<3,3>(-accelerometer_rotmat, State::ACCEL_X, State::BIAS_ACCEL_X);
    // daccel'/dgravity = ?
    J.set_block_holder<3,3>(State::ACCEL_X, State::GRAVITY_X);
    // dori'/dori = ?
    J.set_block_holder<3,3>(State::ORI_X, State::ORI_X);
    // dori'/dangvel = ?*I
    J.set_diagonal_holder<3>(State::ORI_X, State::ANGVEL_X);
    // dangvel'/dbias_angvel = -gyroscope_rotmat
    J.set_block<3,3>(-gyroscope_rotmat, State::ANGVEL_X, State::BIAS_ANGVEL_X);
    // dbias_accel'/dbias_accel = I
    J.set_diagonal<3>(1.0, State::BIAS_ACCEL_X, State::BIAS_ACCEL_X);
    // dbias_angvel'/dbias_angvel = I
    J.set_diagonal<3>(1.0, State::BIAS_ANGVEL_X, State::BIAS_ANGVEL_X);
    // dgravity'/dgravity = I
    J.set_diagonal<3>(1.0, State::GRAVITY_X, State::GRAVITY_X);
    // dwind'/dwind = I
    J.set_diagonal<3>(1.0, State::WIND_X, State::WIND_X);
    // dpressure'/dvel = ?
    J.set_block_holder<1,3>(State::PRESSURE, State::VEL_X);
    // dpressure'/daccel = ?
    J.set_block_holder<1,3>(State::PRESSURE, State::ACCEL_X);
    // dpressure'/dori = ?
    J.set_block_holder<1,3>(State::PRESSURE, State::ORI_X);
    // dpressure'/dgravity = ?
    J.set_block_holder<1,3>(State::PRESSURE, State::GRAVITY_X);
    // dpressure'/dpressure = ?
    J.set_value_holder(State::PRESSURE, State::PRESSURE);
    ////
    J.initialize(tools::EPS);
    // Prepare the (continuous-time) process noise covariance matrix
    C.resize(State::DIM, State::DIM);
    ////
    // Reorient accelerometer_noise_covar to body-coordinates
    RealMat<3,3> const accelerometer_noise_covar_reoriented = accelerometer_rotmat * accelerometer_noise_covar * accelerometer_rotmat.transpose();
    C.set_block<3,3>(accelerometer_noise_covar_reoriented, State::ACCEL_X, State::ACCEL_X);
    // Reorient gyroscope_noise_covar to body-coordinates
    RealMat<3,3> const gyroscope_noise_covar_reoriented = gyroscope_rotmat * gyroscope_noise_covar * gyroscope_rotmat.transpose();
    C.set_block<3,3>(gyroscope_noise_covar_reoriented, State::ANGVEL_X, State::ANGVEL_X);
    // Accelerometer bias random walk is in accelerometer coordinates, as given
    C.set_block<3,3>(accelerometer_drift_covar, State::BIAS_ACCEL_X, State::BIAS_ACCEL_X);
    // Gyroscope bias random walk is in gyroscope coordinates, as given
    C.set_block<3,3>(gyroscope_drift_covar, State::BIAS_ANGVEL_X, State::BIAS_ANGVEL_X);
    // Wind random walk is in world-coordinates, as given
    C.set_block<3,3>(wind_drift_covar, State::WIND_X, State::WIND_X);
    // Pressure random walk, as given
    C.set_value(pow(pressure_drift_stdev,2), State::PRESSURE, State::PRESSURE);
    // To prevent numerical instability for static states, add a negligible random walk
    C.set_diagonal<3>(1e-18, State::GRAVITY_X, State::GRAVITY_X);
    ////
    C.initialize(tools::EPS);
}

//////////////////////////////////////////////////

bool IMU::evolve(State& state, RealVecN const& input, Real dt) const {
    // Verify input
    if (input.rows() != input_dim) {return false;}
    auto const& accelerometer = input.head<3>();
    auto const& gyroscope = input.tail<3>();
    State next;
    // Memoize
    Real const hdt2 = 0.5*dt*dt;
    RealVec<3> const dp = dt*state.vel + hdt2*state.accel;
    RealMat<3,3> const R = state.ori.toRotationMatrix();
    RealVec<3> const Rdp = R*dp;
    RealVec<3> const gR = R.transpose()*state.gravity;
    Quat const dq = tools::qexp(dt*state.angvel);
    Real const rho = state.pressure / specific_energy;
    Real const gRdp = gR.dot(dp);
    // Compute evolved state mean
    next.pos = state.pos + Rdp;
    next.vel = state.vel + dt*state.accel;
    next.accel = accelerometer_rotmat*(accelerometer - state.bias_accel) + gR - state.angvel.cross(state.vel); // - earth_motion_effects - lever_arm_effects (negligible)
    next.ori = (state.ori*dq).normalized();
    next.angvel = gyroscope_rotmat*(gyroscope - state.bias_angvel); // - earth_motion_effects (negligible)
    next.bias_accel = state.bias_accel;
    next.bias_angvel = state.bias_angvel;
    next.gravity = state.gravity;
    next.wind = state.wind;
    next.pressure = state.pressure + rho*gRdp;
    // Update linearization
    ////
    // dpos'/dvel = dt*ori
    J.set_block<3,3>(dt*R, State::POS_X, State::VEL_X);
    // dpos'/daccel = (0.5*dt^2)*ori
    J.set_block<3,3>(hdt2*R, State::POS_X, State::ACCEL_X);
    // dpos'/dori = -ori*crossmat(dt*vel + (0.5*dt^2)*accel)
    J.set_block<3,3>(R*tools::crossmat(dp).transpose(), State::POS_X, State::ORI_X);
    // dvel'/daccel = dt*I
    J.set_diagonal<3>(dt, State::VEL_X, State::ACCEL_X);
    // daccel'/dvel = -crossmat(angvel)
    J.set_skewsym(-state.angvel, State::ACCEL_X, State::VEL_X);
    // daccel'/dori = crossmat((ori^-1)*gravity)
    J.set_skewsym(gR, State::ACCEL_X, State::ORI_X);
    // daccel'/dangvel = crossmat(vel)
    J.set_skewsym(state.vel, State::ACCEL_X, State::ANGVEL_X);
    // daccel'/dgravity = ori^-1
    J.set_block<3,3>(R.transpose(), State::ACCEL_X, State::GRAVITY_X);
    // dori'/dori = exp(dt*angvel)^T
    J.set_block<3,3>(dq.toRotationMatrix().transpose(), State::ORI_X, State::ORI_X);
    // dori'/dangvel = dt*I
    J.set_diagonal<3>(dt, State::ORI_X, State::ANGVEL_X);
    // dpressure'/dvel = dt*rho*(ori^-1)*gravity
    J.set_block<1,3>(dt*rho*gR, State::PRESSURE, State::VEL_X);
    // dpressure'/daccel = (0.5*dt^2)*rho*(ori^-1)*gravity
    J.set_block<1,3>(hdt2*rho*gR, State::PRESSURE, State::ACCEL_X);
    // dpressure'/dori = rho*gravity.(-ori*crossmat(dt*vel + (0.5*dt^2)*accel))
    J.set_block<1,3>(rho*dp.cross(gR), State::PRESSURE, State::ORI_X);
    // dpressure'/dgravity = rho*ori*(dt*vel + (0.5*dt^2)*accel)
    J.set_block<1,3>(rho*Rdp, State::PRESSURE, State::GRAVITY_X);
    // dpressure'/dpressure = 1 + ((ori^-1)*gravity).(dt*vel + (0.5*dt^2)*accel)/specific_energy
    J.set_value(1.0 + gRdp/specific_energy, State::PRESSURE, State::PRESSURE);
    ////
    // Compute evolved state covariance
    next.covar = EKF::predict_covar(state.covar, dt);
    // Update state
    next.when = state.when + dt;
    state = next;
    return true;
}

//////////////////////////////////////////////////

} // namespace processes
} // namespace navigation
} // namespace unitair
