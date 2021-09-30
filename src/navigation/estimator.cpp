/** @file
Implementation of the Estimator and supporting classes.
*/
#include "navigation/estimator.hpp"
#include "navigation/processes/processes.hpp"
#include "navigation/informants/informants.hpp"

namespace unitair {
namespace navigation {

//////////////////////////////////////////////////

State::State(YAML::Node const& cfg) :
    pos(tools::realvec_from_yaml(tools::yget("pos", cfg), 3)),
    vel(tools::realvec_from_yaml(tools::yget("vel", cfg), 3)),
    accel(tools::realvec_from_yaml(tools::yget("accel", cfg), 3)),
    ori(tools::quat_from_yaml(tools::yget("ori", cfg))),
    angvel(tools::realvec_from_yaml(tools::yget("angvel", cfg), 3)),
    bias_accel(tools::realvec_from_yaml(tools::yget("bias_accel", cfg), 3)),
    bias_angvel(tools::realvec_from_yaml(tools::yget("bias_angvel", cfg), 3)),
    gravity(tools::realvec_from_yaml(tools::yget("gravity", cfg), 3)),
    wind(tools::realvec_from_yaml(tools::yget("wind", cfg), 3)),
    pressure(tools::yget("pressure", cfg).as<Real>()),
    when(tools::yget("when", cfg, 0.0).as<Real>()) {
    // Set the covariance matrix
    try {
        // Specify all coefficients...
        covar = tools::realmat_from_yaml(tools::yget("covar", cfg), DIM, DIM);
    } catch (std::runtime_error const&) {
        try {
            // ... or specify just the root-diagonal
            covar.setZero();
            covar.diagonal() = tools::realvec_from_yaml(tools::yget("stdevs", cfg), DIM).array().square();
        } catch (std::runtime_error const&) {
            throw std::runtime_error("Navigation::State could not load covar as matrix or root-diagonal.");
        }
    }
}

/////////////////////////

void State::operator+=(RealVec<DIM> const& d) {
    pos += d.segment<3>(POS_X);
    vel += d.segment<3>(VEL_X);
    accel += d.segment<3>(ACCEL_X);
    ori *= tools::qexp(d.segment<3>(ORI_X));
    angvel += d.segment<3>(ANGVEL_X);
    bias_accel += d.segment<3>(BIAS_ACCEL_X);
    bias_angvel += d.segment<3>(BIAS_ANGVEL_X);
    gravity += d.segment<3>(GRAVITY_X);
    wind += d.segment<3>(WIND_X);
    pressure += d(PRESSURE);
}

/////////////////////////

State State::operator+(RealVec<DIM> const& d) const {
    State perturbed(*this);
    perturbed += d;
    return perturbed;
}

/////////////////////////

RealVec<State::DIM> State::operator-(State const& other) const {
    RealVec<DIM> d;
    d.segment<3>(POS_X) = pos - other.pos;
    d.segment<3>(VEL_X) = vel - other.vel;
    d.segment<3>(ACCEL_X) = accel - other.accel;
    d.segment<3>(ORI_X) = tools::qlog(other.ori.conjugate() * ori);
    d.segment<3>(ANGVEL_X) = angvel - other.angvel;
    d.segment<3>(BIAS_ACCEL_X) = bias_accel - other.bias_accel;
    d.segment<3>(BIAS_ANGVEL_X) = bias_angvel - other.bias_angvel;
    d.segment<3>(GRAVITY_X) = gravity - other.gravity;
    d.segment<3>(WIND_X) = wind - other.wind;
    d(PRESSURE) = pressure - other.pressure;
    return d;
}

/////////////////////////

void State::print() const {
    std::cout << "=====\nSTATE " << std::endl;
    std::cout << "       when: " << when << std::endl;
    std::cout << "        pos: " << pos.transpose() << std::endl;
    std::cout << "        vel: " << vel.transpose() << std::endl;
    std::cout << "      accel: " << accel.transpose() << std::endl;
    std::cout << "        ori: " << ori.coeffs().transpose() << std::endl;
    std::cout << "     angvel: " << angvel.transpose() << std::endl;
    std::cout << " bias_accel: " << bias_accel.transpose() << std::endl;
    std::cout << "bias_angvel: " << bias_angvel.transpose() << std::endl;
    std::cout << "    gravity: " << gravity.transpose() << std::endl;
    std::cout << "       wind: " << wind.transpose() << std::endl;
    std::cout << "   pressure: " << pressure << std::endl;
    std::cout << "     stdevs: " << covar.diagonal().cwiseSqrt().transpose() << std::endl;
    std::cout << "=====        " << std::endl;
}

//////////////////////////////////////////////////

Estimator::Estimator(YAML::Node const& cfg) :
    state0(tools::yget("state0", cfg)),
    state(state0) {
    // Get the process configuration
    {
        YAML::Node const process_cfg = tools::yget("process", cfg);
        Str const process_type = tools::yget("type", process_cfg).as<Str>();
        // Set the process based on the specified type
        if (process_type == "IMU") {
            process = std::make_shared<processes::IMU>(process_cfg);
        } else {
            throw std::runtime_error("Navigation::Process type '"+process_type+"' has not been implemented.");
        }
    }
    // Iterate through the informant configurations
    for (auto const& name_cfg : tools::yget("informants", cfg)) {
        // Unpack configuration
        Str const informant_name = name_cfg.first.as<Str>();
        YAML::Node const informant_cfg = name_cfg.second;
        Str const informant_type = tools::yget("type", informant_cfg).as<Str>();
        bool const ignore = tools::yget("ignore", informant_cfg, false).as<bool>();
        // Create the informant based on the specified type (or skip if told to ignore)
        if (ignore) {
            continue;
        } else if (informant_type == "Barometer") {
            informants.insert({informant_name, std::make_shared<informants::Barometer>(informant_cfg)});
        } else if (informant_type == "GPS") {
            informants.insert({informant_name, std::make_shared<informants::GPS>(informant_cfg)});
        } else if (informant_type == "Magnetometer") {
            informants.insert({informant_name, std::make_shared<informants::Magnetometer>(informant_cfg)});
        } else if (informant_type == "MotionCapture") {
            informants.insert({informant_name, std::make_shared<informants::MotionCapture>(informant_cfg)});
        } else if (informant_type == "Pitot") {
            informants.insert({informant_name, std::make_shared<informants::Pitot>(informant_cfg)});
        } else {
            throw std::runtime_error("Navigation::Informant type '"+informant_type+"' has not been implemented.");
        }
        informant_names.push_back(informant_name);
    }
    informant_names.shrink_to_fit();
}

//////////////////////////////////////////////////

namespace inference_methods {

/////////////////////////

RealMat<State::DIM, State::DIM> EKF::predict_covar(RealMat<State::DIM, State::DIM> const& P, Real dt) const {
    return J()*P*J().transpose() + dt*C();
}

/////

bool EKF::correct_state(State& x, RealVecN const& e) const {
    static Int constexpr innov_dim(Eigen::Dynamic);
    // Compute the Kalman gain
    RealMat< innov_dim, State::DIM> const JP = J()*x.covar; // info-state cross-covariance
    RealMat< innov_dim,  innov_dim> const S  = JP*J().transpose() + C(); // info-info covariance
    RealMat<State::DIM,  innov_dim> const K  = (S.selfadjointView<Eigen::Lower>().ldlt().solve(JP)).transpose();
    // Correct the state mean and covariance
    x += K*e;
    x.covar.triangularView<Eigen::Lower>() -= K*JP;
    x.covar.triangularView<Eigen::StrictlyUpper>() = x.covar.transpose(); // prevent roundoff errors from breaking symmetry
    return true;
}

/////////////////////////

} // inference_methods

//////////////////////////////////////////////////

} // namespace navigation
} // namespace unitair
