/** @file
Implementation of the Planner and supporting classes.
*/
#include "guidance/planner.hpp"

namespace unitair {
namespace guidance {

//////////////////////////////////////////////////

State::State() {
    pos.setZero();
    vel.setZero();
    ori.setIdentity();
}

/////////////////////////

State::State(YAML::Node const& cfg) :
    pos(tools::realvec_from_yaml(tools::yget("pos", cfg), 3)),
    vel(tools::realvec_from_yaml(tools::yget("vel", cfg), 3)),
    ori(tools::quat_from_yaml(tools::yget("ori", cfg))) {
}

/////////////////////////

void State::operator+=(RealVec<DIM> const& d) {
    pos += d.segment<3>(POS_X);
    vel += d.segment<3>(VEL_X);
    ori *= tools::qexp(d.segment<3>(ORI_X));
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
    d.segment<3>(ORI_X) = tools::qlog(other.ori.conjugate() * ori);
    return d;
}

//////////////////////////////////////////////////

Action::Action() {
    thrust = 0.0;
    angvel.setZero();
}

/////////////////////////

void Action::operator+=(RealVec<DIM> const& d) {
    thrust += d(THRUST);
    angvel += d.segment<3>(ANGVEL_X);
}

/////////////////////////

Action Action::operator+(RealVec<DIM> const& d) const {
    Action perturbed(*this);
    perturbed += d;
    return perturbed;
}

/////////////////////////

RealVec<Action::DIM> Action::operator-(Action const& other) const {
    RealVec<DIM> d;
    d(THRUST) = thrust - other.thrust;
    d.segment<3>(ANGVEL_X) = angvel - other.angvel;
    return d;
}

//////////////////////////////////////////////////

Event::Event() {
    when = 0.0;
    cost = 0.0;
    dpredict_dstate.setZero();
    dpredict_daction.setZero();
    dcost_dstate.setZero();
    dcost_daction.setZero();
    d2cost_dstate2.setZero();
    d2cost_daction2.setZero();
    d2cost_daction_dstate.setZero();
}

//////////////////////////////////////////////////

Reference::Reference(YAML::Node const& cfg) {
    // Parse path from the YAML
    for (auto const& point : tools::yget("path", cfg).as<Seq<Seq<Real>>>()) {
        if (point.size() != 3) {throw std::runtime_error("Guidance::Reference path points must be 3D.");}
        path.emplace_back(point.data());
    }
    // Pass to the direct constructor
    *this = Reference(
        tools::yget("times", cfg).as<Seq<Real>>(),
        path,
        tools::yget("loop_index", cfg, -1).as<Int>()
    );
}

//////////////////////////////////////////////////

Reference::Reference(Seq<Real> const& times, Seq<RealVec<3>> const& path, Int loop_index/*=-1*/) :
    times(times),
    path(path),
    timestep(-1) {
    // Validate attributes
    if (times.size() != path.size()) {
        throw std::runtime_error("Guidance::Reference times and path must have the same number of points.");
    }
    if (times.empty()) {
        throw std::runtime_error("Guidance::Reference times and path cannot be empty.");
    }
    if (fabs(times.front()) > tools::EPS) {
        throw std::runtime_error("Guidance::Reference times must begin with zero.");
    }
    for (Nat i=0; i<times.size()-1; ++i) {
        if (times[i] >= times[i+1]) {
            throw std::runtime_error("Guidance::Reference times must be monotonically increasing.");
        }
    }
    set_loop(loop_index);
}

/////////////////////////

Reference::Reference(Real timestep, Seq<RealVec<3>> const& path, Int loop_index/*=-1*/) :
    path(path),
    timestep(timestep) {
    // Validate attributes
    if (path.empty()) {
        throw std::runtime_error("Guidance::Reference path cannot be empty.");
    }
    if (timestep < tools::EPS) {
        throw std::runtime_error("Guidance::Reference timestep must be positive.");
    }
    // Deduce times based on timestep and path size
    times.resize(path.size());
    for (Nat k=0; k<times.size(); ++k) {
        times[k] = k*timestep;
    }
    set_loop(loop_index);
}

/////////////////////////

Reference::Reference(Seq<RealVec<3>> const& path, Real speed, Int loop_index/*=-1*/) :
    path(path),
    timestep(-1) {
    // Validate attributes
    if (path.empty()) {
        throw std::runtime_error("Guidance::Reference path cannot be empty.");
    }
    if (speed < tools::EPS) {
        throw std::runtime_error("Guidance::Reference speed must be positive.");
    }
    // Deduce times based on path spacing and speed
    times.resize(path.size());
    times[0] = 0.0;
    for (Nat i=0; i<path.size()-1; ++i) {
        times[i+1] = times[i] + (path[i+1]-path[i]).norm()/speed;
    }
    set_loop(loop_index);
}

/////////////////////////

void Reference::equalize(Real timestep) {
    // Validate timestep
    if (timestep < tools::EPS) {
        throw std::runtime_error("Guidance::Reference timestep must be positive.");
    }
    // How many steps are needed to cover the original range
    Nat const n = times.back()/timestep + 1;
    Seq<Real> uniform_times(n);
    Seq<RealVec<3>> uniform_path(n);
    // Interpolate
    for (Nat k=0; k<n; ++k) {
        uniform_times[k] = k*timestep;
        uniform_path[k] = at(uniform_times[k] - time0);
    }
    // Replace data
    times = uniform_times;
    path = uniform_path;
    this->timestep = timestep;
    set_loop(loop_time / timestep);
}

/////////////////////////

void Reference::connect(RealVec<3> const& pos0, Real speed/*=-1*/) {
    // Infer speed from average if null was provided
    if (speed < tools::EPS) {
        speed = 0.0;
        for (Nat i=0; i<path.size()-1; ++i) {
            speed += (path[i+1]-path[i]).norm() / (times[i+1]-times[i]) / (path.size()-1);
        }
        // Can't connect with zero speed
        if (speed < tools::EPS) {return;}
    }
    // Prepend lead-in
    Real const lead_duration = (path[0] - pos0).norm() / speed;
    if (lead_duration > tools::EPS) {
        for (auto& t : times) {t += lead_duration;}
        times.insert(times.begin(), 0.0);
        path.insert(path.begin(), pos0);
        if (loop_index >= 0) {loop_index += 1;}
        loop_time += lead_duration;
        if (not tools::is_close(timestep, lead_duration)) {timestep = -1;}
    }
    // Append lead-out
    RealVec<3> const& loop_point = path[tools::imod(loop_index, path.size())];
    Real const loop_duration = (loop_point - path.back()).norm() / speed;
    if (loop_period and (loop_duration > tools::EPS)) {
        times.push_back(times.back() + loop_duration);
        path.push_back(loop_point);
        if (loop_index < 0) {loop_index -= 1;}
        loop_period += loop_duration;
        if (not tools::is_close(timestep, loop_duration)) {timestep = -1;}
    }
}

/////////////////////////

RealVec<3> Reference::at(Real t) const {
    // Offset, wrap, and clip the requested time onto valid times
    t += time0;
    if (loop_period and (t > times.back())) {
        t = fmod(t-loop_time, loop_period) + loop_time;
    }
    t = tools::clip(t, times.front(), times.back());
    // Find times that bracket the requested time
    Nat a, b;
    if (is_uniform()) {
        // Index then interpolate: O(1)
        a = t/timestep;
        if (a >= times.size()-1) {return path.back();}
        b = a+1;
        return path[a] + ((t-times[a])/timestep)*(path[b]-path[a]);
    } else {
        // Search then interpolate: O(log(n))
        b = std::lower_bound(times.begin(), times.end(), t) - times.begin();
        if (b == 0) {return path.front();}
        a = b-1;
        return path[a] + ((t-times[a])/(times[b]-times[a]))*(path[b]-path[a]);
    }
}

/////////////////////////

void Reference::print() const {
    std::cout << "=========" << std::endl;
    std::cout << "REFERENCE" << std::endl;
    for (Nat i=0; i<times.size(); ++i) {
        std::cout << "t = " << times[i] << " | r = " << path[i].transpose() << std::endl;
    }
    if (loop_period) {std::cout << "(loops t = " << times.back() << " -> " << loop_time << ")" << std::endl;}
    std::cout << "=========" << std::endl;
}

/////////////////////////

void Reference::set_loop(Int index) {
    if ((index >= Int(times.size())) or (index < -Int(times.size()))) {
        throw std::runtime_error("Guidance::Reference loop_index is out-of-bounds for path.");
    }
    loop_index = index;
    loop_time = times[tools::imod(loop_index, times.size())];
    loop_period = times.back() - loop_time;
    if (loop_period < tools::EPS) {
        loop_period = false;
    }
}

//////////////////////////////////////////////////

Model::Model(YAML::Node const& cfg) :
    gravity(tools::realvec_from_yaml(tools::yget("gravity", cfg), 3)),
    mass(tools::yget("mass", cfg).as<Real>()),
    coeff_lift(tools::yget("coeff_lift", cfg, 0.0).as<Real>()),
    coeff_drag(tools::yget("coeff_drag", cfg).as<Real>()),
    strictness(tools::yget("strictness", cfg).as<Real>()),
    min_height(tools::yget("min_height", cfg, -tools::YINF).as<Real>()),
    min_airvel(tools::yget("min_airvel", cfg, -tools::YINF).as<Real>()),
    min_thrust(tools::yget("min_thrust", cfg).as<Real>()),
    nom_thrust(tools::yget("nom_thrust", cfg).as<Real>()),
    max_thrust(tools::yget("max_thrust", cfg).as<Real>()),
    lim_angvel(tools::realvec_from_yaml(tools::yget("lim_angvel", cfg), 3)),
    weight_thrust(tools::yget("weight_thrust", cfg).as<Real>()),
    weight_angvel(tools::realvec_from_yaml(tools::yget("weight_angvel", cfg), 3)),
    weight_poserr(tools::realvec_from_yaml(tools::yget("weight_poserr", cfg), 3)),
    weight_velerr(tools::realvec_from_yaml(tools::yget("weight_velerr", cfg), 3)),
    weight_align_fore(tools::yget("weight_align_fore", cfg, 0.0).as<Real>()),
    weight_align_nadir(tools::yget("weight_align_nadir", cfg, 0.0).as<Real>()),
    strictness2(strictness*strictness),
    gdir(gravity.normalized()),
    gdyad(gdir * gdir.transpose()),
    coeffmat(Eigen::DiagonalMatrix<Real,3>(coeff_drag/mass, coeff_drag/mass, (coeff_drag+coeff_lift)/mass)),
    crossmat_fore(tools::crossmat(RealVec<3>(weight_align_fore, 0, 0))),
    crossmat_nadir(tools::crossmat(RealVec<3>(0, 0, weight_align_nadir))) {
    // Validate attributes
    if (mass < tools::EPS) {throw std::runtime_error("Guidance::Model mass must be positive.");}
    if (coeff_lift < 0) {throw std::runtime_error("Guidance::Model coeff_lift must be nonnegative.");}
    if (coeff_drag < tools::EPS) {throw std::runtime_error("Guidance::Model coeff_drag must be positive.");}
    if (strictness < 0) {throw std::runtime_error("Guidance::Model strictness must be nonnegative.");}
    if (nom_thrust < 0) {throw std::runtime_error("Guidance::Model nom_thrust must be nonnegative.");}
    if (nom_thrust < min_thrust) {throw std::runtime_error("Guidance::Model nom_thrust must be greater than min_thrust.");}
    if (max_thrust < nom_thrust) {throw std::runtime_error("Guidance::Model max_thrust must be greater than nom_thrust.");}
    if (lim_angvel.minCoeff() < 0) {throw std::runtime_error("Guidance::Model lim_angvel must be nonnegative.");}
    if (weight_thrust < 0) {throw std::runtime_error("Guidance::Model weight_thrust must be nonnegative.");}
    if (weight_angvel.minCoeff() < 0) {throw std::runtime_error("Guidance::Model weight_angvel must be nonnegative.");}
    if (weight_poserr.minCoeff() < 0) {throw std::runtime_error("Guidance::Model weight_poserr must be nonnegative.");}
    if (weight_velerr.minCoeff() < 0) {throw std::runtime_error("Guidance::Model weight_velerr must be nonnegative.");}
    if (weight_align_fore < 0) {throw std::runtime_error("Guidance::Model weight_align_fore must be nonnegative.");}
    if (weight_align_nadir < 0) {throw std::runtime_error("Guidance::Model weight_align_nadir must be nonnegative.");}
}

/////////////////////////

void Model::predict(Event& now, Event& next, Real dt) const {
    // Memoize
    RealMat<3,3> const Vdt = tools::crossmat(dt*now.state.vel);
    RealMat<3,3> const R = now.state.ori.toRotationMatrix();
    RealVec<3> const gR = R.transpose()*gravity;
    Quat const dq = tools::qexp(dt*now.action.angvel);
    RealVec<3> const f(
        now.action.thrust - (coeff_drag * now.state.vel(0)),
        -coeff_drag * now.state.vel(1),
        -(coeff_drag + coeff_lift) * now.state.vel(2)
    );
    // Compute current sensitivities (skipped entries are zero)
    ////
    // dpos'/dpos = I
    now.dpredict_dstate.block<3,3>(State::POS_X, State::POS_X).setIdentity();
    // dpos'/dvel = dt*ori
    now.dpredict_dstate.block<3,3>(State::POS_X, State::VEL_X) = dt*R;
    // dpos'/dori = -ori*crossmat(dt*vel)
    now.dpredict_dstate.block<3,3>(State::POS_X, State::ORI_X).noalias() = R*Vdt.transpose();
    // dvel'/dvel = I - dt*(diag(c_d,c_d,c_d+c_l)/m + crossmat(angvel))
    now.dpredict_dstate.block<3,3>(State::VEL_X, State::VEL_X) = RealMat<3,3>::Identity() - dt*(coeffmat + tools::crossmat(now.action.angvel));
    // dvel'/dori = dt*crossmat((ori^-1)*gravity)
    now.dpredict_dstate.block<3,3>(State::VEL_X, State::ORI_X) = tools::crossmat(dt*gR);
    // dori'/dori = exp(dt*angvel)^T
    now.dpredict_dstate.block<3,3>(State::ORI_X, State::ORI_X) = dq.toRotationMatrix().transpose();
    // dvel'/dthrust = [dt/m, 0, 0]^T
    now.dpredict_daction(State::VEL_X, Action::THRUST) = dt/mass;
    // dvel'/dangvel = crossmat(dt*vel)
    now.dpredict_daction.block<3,3>(State::VEL_X, Action::ANGVEL_X) = Vdt;
    // dori'/dangvel = dt*I
    now.dpredict_daction.block<3,3>(State::ORI_X, Action::ANGVEL_X).diagonal().setConstant(dt);
    ////
    // Compute next state and time
    next.when      =  now.when      + dt;
    next.state.pos =  now.state.pos + dt*(R*now.state.vel);
    next.state.vel =  now.state.vel + dt*(f/mass + gR - now.action.angvel.cross(now.state.vel));
    next.state.ori = (now.state.ori * dq).normalized();
}

/////////////////////////

void Model::cost(Event& event, Reference const& reference, Real dt) const {
    // Reference and auxiliary states
    RealVec<3> const refpos = reference.at(event.when);
    RealVec<3> const refvel = (reference.at(event.when + dt) - refpos) / dt;
    Real const height = event.state.pos.dot(-gdir);
    Real const airvel = event.state.vel(0); // + coeff_slip*event.action.thrust
    // Tracking errors / deviations
    RealVec<3> const poserr = event.state.pos - refpos;
    RealVec<3> const velerr = event.state.ori*event.state.vel - refvel;
    Real const thrustdev = event.action.thrust - nom_thrust;
    // Constraint violations (indicators and values)
    ////
    // Height
    Real height_viol_ind, height_viol_val;
    if (height < min_height) {
        height_viol_ind = 1.0;
        height_viol_val = height - min_height;
    } else {
        height_viol_ind = 0.0;
        height_viol_val = 0.0;
    }
    // Airspeed
    Real airvel_viol_ind, airvel_viol_val;
    if (airvel < min_airvel) {
        airvel_viol_ind = 1.0;
        airvel_viol_val = airvel - min_airvel;
    } else {
        airvel_viol_ind = 0.0;
        airvel_viol_val = 0.0;
    }
    // Thrust
    Real thrust_viol_ind, thrust_viol_val;
    if (event.action.thrust < min_thrust) {
        thrust_viol_ind = 1.0;
        thrust_viol_val = event.action.thrust - min_thrust;
    } else if (event.action.thrust > max_thrust) {
        thrust_viol_ind = 1.0;
        thrust_viol_val = event.action.thrust - max_thrust;
    } else {
        thrust_viol_ind = 0.0;
        thrust_viol_val = 0.0;
    }
    // Angular velocity
    RealVec<3> angvel_viol_ind, angvel_viol_val;
    for (Nat axis=0; axis<3; ++axis) {
        if (fabs(event.action.angvel(axis)) > lim_angvel(axis)) {
            angvel_viol_ind(axis) = 1.0;
            angvel_viol_val(axis) = event.action.angvel(axis) - copysign(lim_angvel(axis), event.action.angvel(axis));
        } else {
            angvel_viol_ind(axis) = 0.0;
            angvel_viol_val(axis) = 0.0;
        }
    }
    ////
    // Memoization
    RealMat<3,3> const R = event.state.ori.toRotationMatrix();
    RealMat<3,3> const V = tools::crossmat(event.state.vel);
    RealMat<3,3> const U = tools::crossmat(R.transpose()*refvel.normalized());
    RealMat<3,3> const G = tools::crossmat(R.transpose()*gdir);
    RealVec<3> const angvel_weighted = weight_angvel.cwiseProduct(event.action.angvel);
    RealVec<3> const poserr_weighted = weight_poserr.cwiseProduct(poserr);
    RealVec<3> const velerr_weighted = weight_velerr.cwiseProduct(velerr);
    Real const weight_upsidedown = (G(1,0) > 0.0 ? strictness : 1.0);
    // Evaluation
    event.cost = 0.5 * (
        weight_thrust*(thrustdev*thrustdev) +
        angvel_weighted.dot(event.action.angvel) +
        poserr_weighted.dot(poserr) +
        velerr_weighted.dot(velerr) +
        2.0*(weight_align_fore*U(1,2) + weight_upsidedown*weight_align_nadir*G(1,0)) +
        strictness*(strictness*height_viol_val*height_viol_val + airvel_viol_val*airvel_viol_val + thrust_viol_val*thrust_viol_val + angvel_viol_val.squaredNorm())
    );
    // Cost sensitivities (skipped entries are zero)
    ////
    // State jacobian
    event.dcost_dstate.segment<3>(State::POS_X) = poserr_weighted - (strictness2*height_viol_val)*gdir;
    event.dcost_dstate.segment<3>(State::VEL_X).noalias() = velerr_weighted.transpose()*R;
    event.dcost_dstate.segment<3>(State::ORI_X).noalias() = event.dcost_dstate.segment<3>(State::VEL_X)*V.transpose() - weight_align_fore*U.row(0) + (weight_upsidedown*weight_align_nadir)*G.row(2);
    // Action jacobian
    event.dcost_daction(Action::THRUST) = weight_thrust*thrustdev + strictness*thrust_viol_val;
    event.dcost_daction.segment<3>(Action::ANGVEL_X) = angvel_weighted + strictness*angvel_viol_val;
    // State hessian
    event.d2cost_dstate2.block<3,3>(State::POS_X, State::POS_X) = (strictness2*height_viol_ind)*gdyad;
    event.d2cost_dstate2.block<3,3>(State::POS_X, State::POS_X).diagonal() += weight_poserr;
    event.d2cost_dstate2.block<3,3>(State::VEL_X, State::VEL_X).noalias() = R.transpose()*weight_velerr.asDiagonal()*R;
    event.d2cost_dstate2.block<3,3>(State::VEL_X, State::ORI_X).noalias() = tools::crossmat(event.dcost_dstate.segment<3>(State::VEL_X)) - event.d2cost_dstate2.block<3,3>(State::VEL_X, State::VEL_X)*V;
    event.d2cost_dstate2.block<3,3>(State::ORI_X, State::VEL_X) = event.d2cost_dstate2.block<3,3>(State::VEL_X, State::ORI_X).transpose();
    event.d2cost_dstate2.block<3,3>(State::ORI_X, State::ORI_X).noalias() = V*event.d2cost_dstate2.block<3,3>(State::VEL_X, State::ORI_X) - crossmat_fore*U + weight_upsidedown*crossmat_nadir*G;
    // Action hessian
    event.d2cost_daction2(Action::THRUST, Action::THRUST) = weight_thrust + strictness*thrust_viol_ind;
    event.d2cost_daction2.block<3,3>(Action::ANGVEL_X, Action::ANGVEL_X).diagonal() = weight_angvel + strictness*angvel_viol_ind;
    // Cross hessian
    // (happens to be all zero)
    // Additional terms post-memoization
    event.dcost_dstate(State::VEL_X) += strictness*airvel_viol_val;
    event.d2cost_dstate2(State::VEL_X, State::VEL_X) += strictness*airvel_viol_ind;
    ////
}

//////////////////////////////////////////////////

Planner::Planner(YAML::Node const& cfg) :
    timestep(tools::yget("timestep", cfg).as<Real>()),
    horizon(tools::yget("horizon", cfg).as<Real>()),
    prox_tol(tools::yget("prox_tol", cfg).as<Real>()),
    cost_tol(tools::yget("cost_tol", cfg).as<Real>()),
    max_iter(tools::yget("max_iter", cfg).as<Int>()),
    model(tools::yget("model", cfg)),
    reference(timestep, {RealVec<3>::Zero()}),
    plan(abs(horizon/timestep)),
    bill(tools::INF),
    regularizer(1.0),
    candidate(plan.size()),
    policy_bias(plan.size()),
    policy_gain(plan.size()) {
    // Validate attributes
    if (timestep < tools::EPS) {throw std::runtime_error("Planner timestep must be positive.");}
    if (horizon < 2.0*timestep) {throw std::runtime_error("Planner horizon must be at least two timesteps long.");}
    if (prox_tol < tools::EPS) {throw std::runtime_error("Planner prox_tol must be positive.");}
    if (cost_tol < 0) {throw std::runtime_error("Planner cost_tol must be nonnegative.");}
    if (max_iter < 1) {throw std::runtime_error("Planner max_iter must be a positive integer.");}
    // Randomize initial action plan interior
    for (Nat t=1; t<plan.size()-1; ++t) {
        plan[t].action += RealVec<Action::DIM>::Random().cwiseAbs();
    }
}

/////////////////////////

bool Planner::optimize(State const& state0, Real proceed/*=0.0*/) {
    // Progress the reference if the current state is within proximity or ahead
    bool const lead = ((state0.pos-reference.at(+horizon)).norm() < (state0.pos-reference.at(-horizon)).norm());
    Real const prox = (lead ? 0.0 : (state0.pos-reference.at(0.0)).norm());
    reference.set_time0(reference.get_time0() + proceed*tools::clip(1.0-prox/prox_tol, 0.0, 1.0));
    // https://en.wikipedia.org/wiki/Differential_dynamic_programming
    bool success = false;
    for (Int iter=0; iter<max_iter; ++iter) {
        // Bellman cost-to-go Jacobian and Hessian boundary conditions
        RealMat<1, State::DIM> dVdx;
        RealMat<State::DIM, State::DIM> d2Vdx2;
        dVdx.setZero();
        d2Vdx2.setZero();
        // Bellman backpropagation
        for (Int t=plan.size()-2; t>=0; --t) {
            Event const& pt = plan[t];
            // Expressions for the local quadratic approximation of the state-action cost-to-go
            auto const dQdx    =  pt.dcost_dstate          + (dVdx * pt.dpredict_dstate);
            auto const dQdu    =  pt.dcost_daction         + (dVdx * pt.dpredict_daction);
            auto const d2Qdx2  =  pt.d2cost_dstate2        + (pt.dpredict_dstate.transpose()  * d2Vdx2 * pt.dpredict_dstate);
            auto const d2Qdu2  = (pt.d2cost_daction2       + (pt.dpredict_daction.transpose() * d2Vdx2 * pt.dpredict_daction)).eval();
            auto const d2Qdudx =  pt.d2cost_daction_dstate + (pt.dpredict_daction.transpose() * d2Vdx2 * pt.dpredict_dstate);
            // Regularized inverse of the action Hessian
            Eigen::SelfAdjointEigenSolver<RealMat<Action::DIM, Action::DIM>> const H(d2Qdu2);
            RealMat<Action::DIM, Action::DIM> const d2Qdu2_inv =
                H.eigenvectors() * ((H.eigenvalues().array().max(0.0)+regularizer).inverse().matrix().asDiagonal()) * H.eigenvectors().transpose();
            // Local policy that optimizes the quadratic approximation
            policy_bias[t].noalias() = -dQdu * d2Qdu2_inv;
            policy_gain[t].noalias() = -d2Qdu2_inv * d2Qdudx;
            // Dynamic programming principle
            dVdx   = dQdx   - (policy_bias[t].transpose() * d2Qdu2 * policy_gain[t]);
            d2Vdx2 = d2Qdx2 - (policy_gain[t].transpose() * d2Qdu2 * policy_gain[t]);
        }
        // Forward simulation of improved plan
        Real candidate_bill = 0.0;
        candidate[0].state = state0;
        for (Nat t=0; t<plan.size()-1; ++t) {
            candidate[t].action = plan[t].action + (policy_bias[t] + policy_gain[t]*(candidate[t].state - plan[t].state));
            model.predict(candidate[t], candidate[t+1], timestep);
            model.cost(candidate[t], reference, timestep);
            candidate_bill += candidate[t].cost;
        }
        candidate_bill *= timestep / plan.size();
        // Verify that the "improved" plan is actually better
        Real const improvement = bill - candidate_bill;
        if (improvement > -tools::EPS) {
            // If it is, then accept it and reduce the level of regularization
            success = true;
            plan = candidate;
            bill = candidate_bill;
            if (regularizer > 1e-9) {regularizer /= 10.0;}
        } else {
            // If it isn't, then increase the level of regularization and try again
            success = false;
            if (regularizer < 1e9) {regularizer *= 10.0;}
            else {
                // If the regularization has been increased this far to no avail, just accept the new plan and hope for the best
                regularizer = 1.0;
                plan = candidate;
                bill = candidate_bill;
            }
        }
        // Check for convergence
        if (fabs(improvement) < cost_tol*bill) {break;}
    }
    return success; // only true if the last iteration had positive improvement
}

/////////////////////////

void Planner::set_reference(Reference const& new_reference) {
    reference = new_reference;
    reference.equalize(timestep);
    bill = tools::INF;
    regularizer = 1.0;
}

/////////////////////////

void Planner::print_plan() const {
    std::cout << "=========================" << std::endl;
    std::cout << "          PLAN           " << std::endl;
    std::cout << "          ====           " << std::endl;
    for (Event const& event : plan) {
        std::cout << "t: " << event.when                           << std::endl;
        std::cout << "r: " << reference.at(event.when).transpose() << std::endl;
        std::cout << "p: " << event.state.pos.transpose()          << std::endl;
        std::cout << "v: " << event.state.vel.transpose()          << std::endl;
        std::cout << "q: " << event.state.ori.coeffs().transpose() << std::endl;
        std::cout << "u: " << event.action.thrust                  << std::endl;
        std::cout << "w: " << event.action.angvel.transpose()      << std::endl;
        std::cout << "          ====           " << std::endl;
    }
    std::cout << "Bill: " << bill            << std::endl;
    std::cout << "=========================" << std::endl;
}

//////////////////////////////////////////////////

} // namespace guidance
} // namespace unitair
