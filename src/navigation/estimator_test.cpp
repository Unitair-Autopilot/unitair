/** @file
Validation of the Estimator and supporting classes.
Currently, the process and informants must be EKF-based.
*/
#include "navigation/estimator.hpp"

using namespace unitair;
using namespace navigation;

//////////////////////////////////////////////////

/// Entry point
int main(int argc, char** argv) {
    std::srand(time(nullptr));
    bool overall_success = true;

    // Load and parse the given configuration
    if (argc < 2) {
        std::cerr << "\033[1;31mESTIMATOR TEST: cfg file was not provided\033[0m" << std::endl;
        overall_success = false;
        return 1;
    }
    YAML::Node const cfg = YAML::LoadFile(argv[1]);
    Real const timestep = tools::yget("timestep", cfg).as<Real>();
    Estimator estimator(tools::yget("estimator", cfg));

    // Construct an arbitrary state to test with
    State const x0 = estimator.get_state() + 100*RealVec<State::DIM>::Random();

    // Verify that State arithmetic is consistent
    {
        RealVec<State::DIM> const a = (tools::PI/2)*RealVec<State::DIM>::Random();
        RealVec<State::DIM> const b = (x0 + a) - x0;
        if (not tools::all_close(a, b, 1e-9)) {
            std::cout << "\033[1;31mESTIMATOR TEST: +- State arithmetic operators are not inverses\033[0m" << std::endl;
            overall_success = false;
        }
    }

    // Extract the process and construct an arbitrary input to test it with
    Process const* process = &estimator.get_process();
    auto ekf_process = dynamic_cast<inference_methods::EKF const*>(process);
    if (ekf_process == nullptr) {
        std::cerr << "\033[1;31mESTIMATOR TEST: currently can only verify EKF-based processes\033[0m" << std::endl;
        overall_success = false;
        return 1;
    }
    RealVecN const u0 = 100*RealVecN::Random(process->input_dim);

    // Compute a finite-difference approximation to the process Jacobian
    RealMat<State::DIM, State::DIM> approx_J_evolve;
    for (Int j=0; j<State::DIM; ++j) {
        // Standard-basis perturbation vector
        RealVec<State::DIM> dx; dx.setZero(); dx(j) = 1e-5;
        Real dx_mag = dx.norm();
        // Finite-difference to approximate d_evolve/d_x_i
        State x1 = x0 + dx;
        assert(process->evolve(x1, u0, timestep));
        State x = x0;
        assert(process->evolve(x, u0, timestep));
        approx_J_evolve.col(j) = (x1 - x) / dx_mag;
    }

    // Verify the analytical process Jacobian against the approximation
    RealMat<State::DIM, State::DIM> exact_J_evolve = ekf_process->get_J()();
    for (Int i=0; i<State::DIM; ++i) {
        for (Int j=0; j<State::DIM; ++j) {
            Real const err = fabs(approx_J_evolve(i,j) - exact_J_evolve(i,j));
            Real const tol = timestep/2;
            if (err > tol) {
                std::cout << "\033[1;31mESTIMATOR TEST: process J(" << i << "," << j << ") " << "has error " << err << "\033[0m" << std::endl;
                overall_success = false;
            }
        }
    }

    // Iterate over informants
    for (Str const& informant_name : estimator.get_informant_names()) {
        // Extract the informant and construct an arbitrary observation to test it with
        Informant const* informant = &estimator.get_informant(informant_name);
        auto ekf_informant = dynamic_cast<inference_methods::EKF const*>(informant);
        if (ekf_informant == nullptr) {
            std::cerr << "\033[1;31mESTIMATOR TEST: currently can only verify EKF-based informants\033[0m" << std::endl;
            overall_success = false;
            return 1;
        }
        RealVecN const y0 = 100*RealVecN::Random(informant->info_dim);
    }

    // Result
    std::cout << (overall_success ? "\033[1;33m" : "\033[1;31m") << "ESTIMATOR TEST: complete, but currently only checks EKF processes\033[0m" << std::endl;
    return !overall_success;
}

//////////////////////////////////////////////////
