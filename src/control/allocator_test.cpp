/** @file
Validation of the Allocator and supporting classes.
*/
#include "control/allocator.hpp"

using namespace unitair;
using namespace control;

//////////////////////////////////////////////////

/// Entry point
int main(int argc, char** argv) {
    std::srand(time(nullptr));
    bool overall_success = true;

    // Load and parse the given configuration
    if (argc < 2) {
        std::cerr << "\033[1;31mALLOCATOR TEST: cfg file was not provided\033[0m" << std::endl;
        overall_success = false;
        return 1;
    }
    YAML::Node const cfg = tools::yget("allocator", YAML::LoadFile(argv[1]));

    // Construct main objects
    Allocator const allocator(cfg);
    Seq<Actuator> const& actuators = allocator.get_actuators();
    Int const k = actuators.size();
    Int const n = 10000;

    // Test over many random arguments
    for (Int t=0; t<n; ++t) {
        // Select random but feasible effort values
        RealVecN feasible(k);
        for (Int i=0; i<k; ++i) {
            feasible(i) = tools::sample_uniform(actuators[i].get_mineff(), actuators[i].get_maxeff());
        }
        // Compute the net wrench that should be generated
        RealVec<6> target = RealVec<6>::Zero();
        for (Int i=0; i<k; ++i) {
            target += actuators[i].scalar_at_effort(feasible(i)) * actuators[i].get_basis();
        }
        // Allocate efforts to achieve that wrench
        RealVecN const allocation = allocator.allocate(target);
        // Verify that the allocated solution achieves a wrench equivalent to the random efforts
        RealVec<6> attained = RealVec<6>::Zero();
        for (Int i=0; i<k; ++i) {
            attained += actuators[i].scalar_at_effort(allocation(i)) * actuators[i].get_basis();
        }
        if (not attained.isApprox(target)) {
            std::cout << "\033[1;31mALLOCATOR TEST: solution was incorrect\033[0m" << std::endl;
            overall_success = false;
        }
        // Verify that the allocated solution is feasible
        for (Int i=0; i<k; ++i) {
            if ((allocation(i) < actuators[i].get_mineff()) or (allocation(i) > actuators[i].get_maxeff())) {
                std::cout << "\033[1;31mALLOCATOR TEST: solution was infeasible\033[0m" << std::endl;
                overall_success = false;
            }
        }
        // Verify that the allocated solution is more efficient than the random choice
        if (allocation.norm() > feasible.norm()+1e-8) {
            std::cout << "\033[1;31mALLOCATOR TEST: solution was suboptimal\033[0m" << std::endl;
            overall_success = false;
        }
    }

    // Result
    std::cout << (overall_success ? "\033[1;32m" : "\033[1;31m") << "ALLOCATOR TEST: complete\033[0m" << std::endl;
    return !overall_success;
}

//////////////////////////////////////////////////
