/** @file
Implementation of the Allocator and supporting classes.
*/
#include "control/allocator.hpp"

namespace unitair {
namespace control {

//////////////////////////////////////////////////

Actuator::Actuator(YAML::Node const& cfg, RealVec<3> const& com/*=RealVec<3>::Zero()*/) :
    center(tools::realvec_from_yaml(tools::yget("center", cfg, Seq<Real>({0,0,0})), 3)),
    linvec(tools::realvec_from_yaml(tools::yget("linvec", cfg, Seq<Real>({0,0,0})), 3)),
    angvec(tools::realvec_from_yaml(tools::yget("angvec", cfg, Seq<Real>({0,0,0})), 3)),
    coeff1(tools::yget("coeff1", cfg, 0).as<Real>()),
    coeff2(tools::yget("coeff2", cfg, 0).as<Real>()),
    mineff(tools::yget("mineff", cfg, -1).as<Real>()),
    maxeff(tools::yget("maxeff", cfg, +1).as<Real>()) {
    basis << linvec, angvec + (center-com).cross(linvec);
    // Validate coefficients
    if ((coeff1 < 0) or (coeff2 < 0)) {
        throw std::runtime_error("Actuator model cannot have negative coefficients.");
    }
    // Resolve singularity if both coefficients are zero
    if ((coeff1 < tools::EPS) and (coeff2 < tools::EPS)) {
        coeff1 = 1; coeff2 = 0; basis.setZero();
    }
}

/////////////////////////

Real Actuator::scalar_at_effort(Real eff) const {
    // Impose saturation
    eff = tools::clip(eff, mineff, maxeff);
    // Monotonic-quadratic with zero offset
    return (coeff1 + coeff2*fabs(eff))*eff;
}

/////////////////////////

Real Actuator::effort_at_scalar(Real scl) const {
    Real eff;
    // Invert monotonic-quadratic with zero offset
    if (coeff2 < tools::EPS) {
        eff = scl/coeff1;
    } else if (scl > 0) {
        eff = -(coeff1 - sqrt(coeff1*coeff1+4*coeff2*scl)) / (2*coeff2);
    } else {
        eff =  (coeff1 - sqrt(coeff1*coeff1-4*coeff2*scl)) / (2*coeff2);
    }
    // Impose saturation
    return tools::clip(eff, mineff, maxeff);
}

//////////////////////////////////////////////////

Allocator::Allocator(YAML::Node const& cfg) :
    com(tools::realvec_from_yaml(tools::yget("com", cfg), 3)) {
    // Store given actuators
    for (auto const& actuator_cfg : tools::yget("actuators", cfg)) {
        actuators.emplace_back(actuator_cfg, com);
    } actuators.shrink_to_fit();
    // Compute wrench basis decomposition
    RealMat<6, Eigen::Dynamic> B(6, actuators.size());
    for (Int i=0; i<B.cols(); ++i) {
        B.col(i) = actuators[i].get_basis();
    } decomp = B.completeOrthogonalDecomposition();
}

/////////////////////////

RealVecN Allocator::allocate(RealVec<6> const& wrench) const {
    RealVecN const scalars = decomp.solve(wrench);
    RealVecN efforts(actuators.size());
    for (Int i=0; i<efforts.rows(); ++i) {
        efforts(i) = actuators[i].effort_at_scalar(scalars(i));
    } return efforts;
}

//////////////////////////////////////////////////

} // namespace control
} // namespace unitair
