/** @file
Validation of the Planner and supporting classes.
*/
#include "guidance/planner.hpp"

using namespace unitair;
using namespace guidance;

//////////////////////////////////////////////////

/// @name Helpers
//@{
/// Computes a finite-difference approximation of the Model::cost state-Jacobian
RealMat<1, State::DIM> approximate_dcost_dstate(Event const& e0, Reference const& r, Model const& model, Real dt) {
    RealMat<1, State::DIM> approx_dcost_dstate;
    for (Int j=0; j<State::DIM; ++j) {
        // Standard-basis perturbation vector
        RealVec<State::DIM> dx; dx.setZero(); dx(j) = 1e-3;
        Real dx_mag = dx.norm();
        // Finite-difference
        Event e1(e0); e1.state += dx;
        model.cost(e1, r, dt);
        Event e(e0);
        model.cost(e, r, dt);
        approx_dcost_dstate(j) = (e1.cost - e.cost) / dx_mag;
    }
    return approx_dcost_dstate;
}
/// Computes a finite-difference approximation of the Model::cost action-Jacobian
RealMat<1, Action::DIM> approximate_dcost_daction(Event const& e0, Reference const& r, Model const& model, Real dt) {
    RealMat<1, Action::DIM> approx_dcost_daction;
    for (Int j=0; j<Action::DIM; ++j) {
        // Standard-basis perturbation vector
        RealVec<Action::DIM> du; du.setZero(); du(j) = 1e-3;
        Real du_mag = du.norm();
        // Finite-difference
        Event e1(e0); e1.action += du;
        model.cost(e1, r, dt);
        Event e(e0);
        model.cost(e, r, dt);
        approx_dcost_daction(j) = (e1.cost - e.cost) / du_mag;
    }
    return approx_dcost_daction;
}
//@}

//////////////////////////////////////////////////

/// Entry point
int main(int argc, char** argv) {
    std::srand(time(nullptr));
    bool overall_success = true;

    // Load and parse the given configuration
    if (argc < 2) {
        std::cerr << "\033[1;31mPLANNER TEST: cfg file was not provided\033[0m" << std::endl;
        overall_success = false;
        return 1;
    }
    YAML::Node const cfg = tools::yget("planner", YAML::LoadFile(argv[1]));
    Planner planner(cfg);
    Model const& model = planner.get_model();

    // Construct an arbitrary event to test with
    Event e0;
    e0.state += 10*RealVec<State::DIM>::Random();
    e0.action += 10*RealVec<Action::DIM>::Random();

    // Isolated test of state and action arithmetic
    {
        State const& x0 = e0.state;
        RealVec<State::DIM> const a = (tools::PI/2)*RealVec<State::DIM>::Random();
        RealVec<State::DIM> const b = (x0 + a) - x0;
        if (not tools::all_close(a, b, 1e-9)) {
            std::cout << "\033[1;31mPLANNER TEST: +- State arithmetic operators are not inverses\033[0m" << std::endl;
            overall_success = false;
        }
    }{
        Action const& u0 = e0.action;
        RealVec<Action::DIM> const a = (tools::PI/2)*RealVec<Action::DIM>::Random();
        RealVec<Action::DIM> const b = (u0 + a) - u0;
        if (not tools::all_close(a, b, 1e-9)) {
            std::cout << "\033[1;31mPLANNER TEST: +- Action arithmetic operators are not inverses\033[0m" << std::endl;
            overall_success = false;
        }
    }

    // Isolated test of the Reference class
    {
        Real const t0 = 0, t1 = 3, t2 = 9, t3 = 50;
        RealVec<3> const r0 = RealVec<3>::Random(), r1 = RealVec<3>::Random(), r2 = RealVec<3>::Random(), r3 = RealVec<3>::Random();
        Reference r({t0, t1, t2, t3}, {r0, r1, r2, r3});
        if (not r.at(t3+3).isApprox(r3) or not r.at(0).isApprox(r0) or not r.at((t1+t2)/2).isApprox((r1+r2)/2)) {
                std::cout << "\033[1;31mPLANNER TEST: Reference interpolation failed in terminal-mode\033[0m" << std::endl;
                overall_success = false;
        }
        r.set_loop(0);
        if (not r.at(t3+3).isApprox(r1) or not r.at(-100).isApprox(r0) or not r.at((t1+t2)/2).isApprox((r1+r2)/2)) {
                std::cout << "\033[1;31mPLANNER TEST: Reference interpolation failed in loop(0)-mode\033[0m" << std::endl;
                overall_success = false;
        }
        r.set_loop(1);
        if (not r.at(t3+3).isApprox(r.at((t1+t2)/2)) or not r.at(t3+47+1e-9).isApprox(r1, 1e-9) or not r.at(t3+47-1e-9).isApprox(r3, 1e-9)) {
                std::cout << "\033[1;31mPLANNER TEST: Reference interpolation failed in loop(1)-mode\033[0m" << std::endl;
                overall_success = false;
        }
        r.set_loop(2);
        if (not r.at(t3+20.5).isApprox(r.at((t2+t3)/2)) or not r.at(t2).isApprox(r2) or not r.at(t3-1e-12).isApprox(r3)) {
                std::cout << "\033[1;31mPLANNER TEST: Reference interpolation failed in loop(2)-mode\033[0m" << std::endl;
                overall_success = false;
        }
        r.set_loop(3);
        if (not r.at(t3+3).isApprox(r3) or not r.at(0).isApprox(r0) or not r.at((t1+t2)/2).isApprox((r1+r2)/2)) {
                std::cout << "\033[1;31mPLANNER TEST: Reference interpolation failed in loop(n)-mode\033[0m" << std::endl;
                overall_success = false;
        }
        r.set_loop(-4);
        r.equalize(1);
        if (not r.at(t3+3).isApprox(r1) or not r.at(-2).isApprox(r0) or not r.at((t1+t2)/2).isApprox((r1+r2)/2)) {
                std::cout << "\033[1;31mPLANNER TEST: Reference interpolation failed in loop(-n)-mode equalized\033[0m" << std::endl;
                overall_success = false;
        }
        r = Reference({t0, t1, t2, t3}, {r0, r1, r2, r3}, -4);
        r.connect(r1, 3.0);
        Real const dti = (r0-r1).norm() / 3.0, dtl = (r0-r3).norm() / 3.0;
        if (not r.at(-2).isApprox(r1) or not r.at(t0+dti).isApprox(r0) or not r.at(t3+dti+dtl).isApprox(r0)) {
                std::cout << "\033[1;31mPLANNER TEST: Reference interpolation failed in loop(-n)-mode connected\033[0m" << std::endl;
                overall_success = false;
        }
        r = Reference({r0}, 5, -1);
        r.equalize(999);
        r.set_loop(0);
        r.connect(r0, 1.0);
        if (not r.at(456).isApprox(r0) or not r.at(-2).isApprox(r0)) {
                std::cout << "\033[1;31mPLANNER TEST: Reference interpolation failed in singleton edge-case\033[0m" << std::endl;
                overall_success = false;
        }
    }

    // Construct an arbitrary timestep and reference to test with
    Real const dt = 0.001;
    Reference r(5, {RealVec<3>::Random(), RealVec<3>::Random(), RealVec<3>::Random()}, 0);

    // Compute a finite-difference approximation of the Model::predict Jacobians
    RealMat<State::DIM, State::DIM> approx_dpredict_dstate;
    for (Int j=0; j<State::DIM; ++j) {
        // Standard-basis perturbation vector
        RealVec<State::DIM> dx; dx.setZero(); dx(j) = 1e-6;
        Real dx_mag = dx.norm();
        // Finite-difference
        Event e1(e0); e1.state += dx;
        model.predict(e1, e1, dt);
        Event e(e0);
        model.predict(e, e, dt);
        approx_dpredict_dstate.col(j) = (e1.state - e.state) / dx_mag;
    }
    RealMat<State::DIM, Action::DIM> approx_dpredict_daction;
    for (Int j=0; j<Action::DIM; ++j) {
        // Standard-basis perturbation vector
        RealVec<Action::DIM> du; du.setZero(); du(j) = 1e-6;
        Real du_mag = du.norm();
        // Finite-difference
        Event e1(e0); e1.action += du;
        model.predict(e1, e1, dt);
        Event e(e0);
        model.predict(e, e, dt);
        approx_dpredict_daction.col(j) = (e1.state - e.state) / du_mag;
    }

    // Verify the analytical predict Jacobians against the approximations
    model.predict(e0, e0, dt);
    RealMat<State::DIM, State::DIM> const& exact_dpredict_dstate = e0.dpredict_dstate;
    for (Int i=0; i<State::DIM; ++i) {
        for (Int j=0; j<State::DIM; ++j) {
            Real const err = fabs(approx_dpredict_dstate(i,j) - exact_dpredict_dstate(i,j));
            Real const tol = (fabs(approx_dpredict_dstate(i,j))+1e-3) * 0.2;
            if (err > tol) {
                std::cout << "\033[1;31mPLANNER TEST: dpredict_dstate(" << i << "," << j << ") " << "has error " << err << "\033[0m" << std::endl;
                overall_success = false;
            }
        }
    }
    RealMat<State::DIM, Action::DIM> const& exact_dpredict_daction = e0.dpredict_daction;
    for (Int i=0; i<State::DIM; ++i) {
        for (Int j=0; j<Action::DIM; ++j) {
            Real const err = fabs(approx_dpredict_daction(i,j) - exact_dpredict_daction(i,j));
            Real const tol = (fabs(approx_dpredict_dstate(i,j))+1e-3) * 0.1;
            if (err > tol) {
                std::cout << "\033[1;31mPLANNER TEST: dpredict_daction(" << i << "," << j << ") " << "has error " << err << "\033[0m" << std::endl;
                overall_success = false;
            }
        }
    }

    // Compute a finite-difference approximation of the Model::cost Jacobians
    RealMat<1, State::DIM> const approx_dcost_dstate = approximate_dcost_dstate(e0, r, model, dt);
    RealMat<1, Action::DIM> const approx_dcost_daction = approximate_dcost_daction(e0, r, model, dt);

    // Verify the analytical cost Jacobians against the approximations
    model.cost(e0, r, dt);
    RealMat<1, State::DIM> const& exact_dcost_dstate = e0.dcost_dstate;
    for (Int j=0; j<State::DIM; ++j) {
        Real const err = fabs(approx_dcost_dstate(j) - exact_dcost_dstate(j));
        Real const tol = (fabs(approx_dcost_dstate(j))+1e-3) * 0.2;
        if (err > tol) {
            std::cout << "\033[1;31mPLANNER TEST: dcost_dstate(" << 0 << "," << j << ") " << "has error " << err << "\033[0m" << std::endl;
            overall_success = false;
        }
    }
    RealMat<1, Action::DIM> const& exact_dcost_daction = e0.dcost_daction;
    for (Int j=0; j<Action::DIM; ++j) {
        Real const err = fabs(approx_dcost_daction(j) - exact_dcost_daction(j));
        Real const tol = (fabs(approx_dcost_daction(j))+1e-3) * 0.1;
        if (err > tol) {
            std::cout << "\033[1;31mPLANNER TEST: dcost_daction(" << 0 << "," << j << ") " << "has error " << err << "\033[0m" << std::endl;
            overall_success = false;
        }
    }

    // Compute a finite-difference approximation of the Model::cost Hessians
    RealMat<State::DIM, State::DIM> approx_d2cost_dstate2;
    for (Int j=0; j<State::DIM; ++j) {
        // Standard-basis perturbation vector
        RealVec<State::DIM> dx; dx.setZero(); dx(j) = 1e-2;
        Real dx_mag = dx.norm();
        // Finite-difference
        Event e1(e0); e1.state += dx;
        RealMat<1, State::DIM> const approx_dcost_dstate1 = approximate_dcost_dstate(e1, r, model, dt);
        approx_d2cost_dstate2.col(j) = (approx_dcost_dstate1 - approx_dcost_dstate).transpose() / dx_mag;
    }
    RealMat<Action::DIM, Action::DIM> approx_d2cost_daction2;
    for (Int j=0; j<Action::DIM; ++j) {
        // Standard-basis perturbation vector
        RealVec<Action::DIM> du; du.setZero(); du(j) = 1e-2;
        Real du_mag = du.norm();
        // Finite-difference
        Event e1(e0); e1.action += du;
        RealMat<1, Action::DIM> const approx_dcost_daction1 = approximate_dcost_daction(e1, r, model, dt);
        approx_d2cost_daction2.col(j) = (approx_dcost_daction1 - approx_dcost_daction).transpose() / du_mag;
    }

    // Verify the analytical cost Hessians against the approximations
    model.cost(e0, r, dt);
    RealMat<State::DIM, State::DIM> const& exact_d2cost_dstate2 = e0.d2cost_dstate2;
    for (Int i=0; i<State::DIM; ++i) {
        for (Int j=0; j<State::DIM; ++j) {
            Real const err = fabs(approx_d2cost_dstate2(i,j) - exact_d2cost_dstate2(i,j));
            Real const tol = (fabs(approx_d2cost_dstate2(i,j))+1e-3) * 0.6;
            if (err > tol) {
                std::cout << "\033[1;31mPLANNER TEST: d2cost_dstate2(" << i << "," << j << ") " << "has error " << err << "\033[0m" << std::endl;
                overall_success = false;
            }
        }
    }
    RealMat<Action::DIM, Action::DIM> const& exact_d2cost_daction2 = e0.d2cost_daction2;
    for (Int i=0; i<Action::DIM; ++i) {
        for (Int j=0; j<Action::DIM; ++j) {
            Real const err = fabs(approx_d2cost_daction2(i,j) - exact_d2cost_daction2(i,j));
            Real const tol = (fabs(approx_d2cost_daction2(i,j))+1e-3) * 0.3;
            if (err > tol) {
                std::cout << "\033[1;31mPLANNER TEST: d2cost_daction2(" << i << "," << j << ") " << "has error " << err << "\033[0m" << std::endl;
                overall_success = false;
            }
        }
    }

    // Test planner optimization
    State s;
    s.pos(2) = tools::clip(model.get_min_height(), 0.0, tools::INF); // assumes gravity along z
    s.vel(0) = tools::clip(model.get_min_airvel(), 0.0, tools::INF);
    planner.set_reference(Reference({0, planner.get_horizon()}, {RealVec<3>(0,0,0), RealVec<3>(5+planner.get_horizon()*s.vel(0),0,0)}));
    for (Nat i=0; i<100; ++i) {planner.optimize(s);}
    Real const err0 = (s.pos - planner.get_reference().at(planner.get_plan().back().when)).norm();
    Real const err1 = (planner.get_plan().back().state.pos - planner.get_reference().at(planner.get_plan().back().when)).norm();
    if (err1 > err0) {
        std::cout << "\033[1;31mPLANNER TEST: Failed to optimize simple situation\033[0m" << std::endl;
        overall_success = false;
    }
    //planner.print_plan();

    // Result
    std::cout << (overall_success ? "\033[1;32m" : "\033[1;31m") << "PLANNER TEST: complete\033[0m" << std::endl;
    return !overall_success;
}

//////////////////////////////////////////////////
