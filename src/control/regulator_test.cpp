/** @file
Validation of the Regulator.
*/
#include "control/regulator.hpp"

using namespace unitair;
using namespace control;

//////////////////////////////////////////////////

/// Entry point
int main(int argc, char** argv) {
    std::srand(time(nullptr));
    bool overall_success = true;
    Real const tol = 5e-2; // rad/s

    // Load and parse the given configuration
    if (argc < 2) {
        std::cerr << "\033[1;31mREGULATOR TEST: cfg file was not provided\033[0m" << std::endl;
        overall_success = false;
        return 1;
    }
    YAML::Node const cfg = tools::yget("regulator", YAML::LoadFile(argv[1]));
    Regulator regulator(cfg);

    // Simple simulation setup
    Real const timestep = 0.005; // s
    Real const duration = 10.0; // s
    RealMat<3,3> const inertia_matrix = regulator.get_inertia_matrix();
    RealMat<3,3> const inertia_inverse = inertia_matrix.inverse();
    RealVec<3> const aerodamper_coeffs = regulator.get_aerodamper_coeffs();
    RealVec<3> const disturbance = regulator.get_disturbance();
    RealVec<3> const airvel = 2.0 * tools::clip(regulator.get_min_airvel(), 0.0, tools::INF) * regulator.get_aero_axis();
    RealVec<3> angvel; angvel.setRandom();
    RealVec<3> angvel_target; angvel_target.setRandom();
    RealVec<3> angaccel_target; angaccel_target.setZero();

    // Run simple simulation of angvel regulation
    Real t = 0;
    while (t < duration) {
        t += timestep;
        RealVec<3> const torque = regulator.regulate(Quat::Identity(), Quat::Identity(), angvel, angvel_target, angaccel_target, airvel, timestep);
        angvel += timestep*inertia_inverse*(torque + disturbance - aerodamper_coeffs.cwiseProduct(angvel) - angvel.cross(inertia_matrix*angvel));
    }

    // Validate stability
    if (not tools::all_close(angvel, angvel_target, tol)) {
        std::cout << "\033[1;31mREGULATOR TEST: failed to regulate simple system\033[0m" << std::endl;
        overall_success = false;
    }

    // Result
    std::cout << (overall_success ? "\033[1;32m" : "\033[1;31m") << "REGULATOR TEST: complete\033[0m" << std::endl;
    return !overall_success;
}

//////////////////////////////////////////////////
