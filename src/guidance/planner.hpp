/** @file
Declaration of the Planner and supporting classes.
*/
#pragma once
#include "tools/tools.hpp"

namespace unitair {
namespace guidance {

//////////////////////////////////////////////////

/// Container for the state of a planned event
struct State {
    /// Semantic names for each dimension of the manifold (DIM is total dimensionality)
    enum Dimensions {
        POS_X, POS_Y, POS_Z,
        VEL_X, VEL_Y, VEL_Z,
        ORI_X, ORI_Y, ORI_Z,
    DIM};
    /// @name Values
    //@{
    RealVec<3> pos; ///< Position in world-coordinates of the body-origin
    RealVec<3> vel; ///< Velocity in body-coordinates of the body-origin relative to the world-frame
          Quat ori; ///< Orientation of body-coordinates relative to world-coordinates
    //@}
    /// Constructs at the origin
    State();
    /// Constructs using a configuration YAML that specifies "pos", "vel", and "ori"
    State(YAML::Node const& cfg);
    /// Perturbs this state by the given tangent vector
    void operator+=(RealVec<DIM> const& d);
    /// Computes this state perturbed by the given tangent vector
    State operator+(RealVec<DIM> const& d) const;
    /// Computes the tangent vector that would perturb the given state to this state
    RealVec<DIM> operator-(State const& other) const;
};

/////////////////////////

/// Container for the action of a planned event
struct Action {
    /// Semantic names for each dimension of the manifold (DIM is total dimensionality)
    enum Dimensions {
        THRUST,
        ANGVEL_X, ANGVEL_Y, ANGVEL_Z,
    DIM};
    /// @name Values
    //@{
    Real       thrust; ///< Collective propulsion force along the first body-direction (forward)
    RealVec<3> angvel; ///< Commanded angular velocity in body-coordinates of the body-frame relative to the world-frame
    //@}
    /// Constructs at the origin
    Action();
    /// Perturbs this action by the given tangent vector
    void operator+=(RealVec<DIM> const& d);
    /// Computes this action perturbed by the given tangent vector
    Action operator+(RealVec<DIM> const& d) const;
    /// Computes the tangent vector that would perturb the given action to this action
    RealVec<DIM> operator-(Action const& other) const;
};

/////////////////////////

/// Container for a planned state and action, as well as its cost and sensitivities
struct Event {
    /// @name Values
    //@{
    Real when;     ///< Time for which this event is planned
    State state;   ///< Situation planned for this event
    Action action; ///< Decision planned for this event
    Real cost;     ///< Objective loss accrued for this event per timestep
    //@}
    /// @name Sensitivities
    //@{
    RealMat< State::DIM,  State::DIM> dpredict_dstate;       ///< Jacobian of the model's predict function wrt this state
    RealMat< State::DIM, Action::DIM> dpredict_daction;      ///< Jacobian of the model's predict function wrt this action
    RealMat<          1,  State::DIM> dcost_dstate;          ///< Jacobian of the model's cost function wrt this state
    RealMat<          1, Action::DIM> dcost_daction;         ///< Jacobian of the model's cost function wrt this action
    RealMat< State::DIM,  State::DIM> d2cost_dstate2;        ///< Hessian of the model's cost function wrt this state
    RealMat<Action::DIM, Action::DIM> d2cost_daction2;       ///< Hessian of the model's cost function wrt this action
    RealMat<Action::DIM,  State::DIM> d2cost_daction_dstate; ///< Cross-Hessian of the model's cost function wrt this action / state
    //@}
    /// Constructs at the origin
    Event();
};

//////////////////////////////////////////////////

/// Maintains an interpolated reference trajectory to judge against
class Reference {
    /// @name Given
    //@{
    Seq<Real> times; ///< Timestamps for each point on the path (must start with zero and monotonically increase)
    Seq<RealVec<3>> path; ///< Positions in world-coordinates corresponding to each timestamp
    Int loop_index; ///< Signed-index of the point along the path to wrap the end of the path back onto (DEFAULT: -1 implies no loop)
    //@}

    /// @name Derived
    //@{
    Real loop_time; ///< Time corresponding to loop_index
    Real loop_period; ///< Duration of the cyclic part of the path (i.e. from loop_time to the last time)
    Real timestep; ///< If the times are known to be uniformly spaced, then this is that spacing, else it is -1
    Real time0=0.0; ///< Time origin for interpolation queries (assumed zero until explicitly set)
    //@}

public:
    /// @name Core
    //@{
    /// Constructs using a configuration YAML that recursively specifies all the "Given" attributes
    Reference(YAML::Node const& cfg);
    /// Constructs with the given values and validates them (times are assumed to be nonuniform)
    Reference(Seq<Real> const& times, Seq<RealVec<3>> const& path, Int loop_index=-1);
    /// Constructs by deriving uniform times from the given (positive) timestep and path (enables O(1) interpolation)
    Reference(Real timestep, Seq<RealVec<3>> const& path, Int loop_index=-1);
    /// Constructs by deriving times from the specified (positive) speed and given path (spacing is assumed to be nonuniform)
    Reference(Seq<RealVec<3>> const& path, Real speed, Int loop_index=-1);
    /// Replaces the stored trajectory data with interpolated values at evenly spaced times (enables O(1) interpolation)
    void equalize(Real timestep);
    /// Prepends a lead-in from pos0 onto the path, and mends any discontinuity due to looping (resulting times are nonuniform)
    /** (if a positive speed is not provided, the average speed along the original path is assumed) */
    void connect(RealVec<3> const& pos0, Real speed=-1);
    //@}

    /// @name Utilities
    //@{
    /// Piecewise-linearly interpolates the reference trajectory for a path value at the specified time (relative to time0)
    /** (the algorithm runtime is constant if the times are uniform, else it's logarithmic in the trajectory size) */
    RealVec<3> at(Real t) const;
    /// Displays the contents of this reference to the console
    void print() const;
    //@}

    /// @name Access
    //@{
    void set_loop(Int index);
    inline void set_time0(Real t) {time0 = t;}
    inline Real get_time0() const {return time0;}
    inline Seq<Real> const& get_times() const {return times;}
    inline Seq<RealVec<3>> const& get_path() const {return path;}
    inline bool is_uniform() const {return (timestep > tools::EPS);}
    inline bool is_looping() const {return (time0 >= loop_time);}
    //@}
};

//////////////////////////////////////////////////

/// Deterministic Markov decision process for predicting and judging event trajectories
class Model {
    /// @name Given
    //@{
    // Physics
    RealVec<3> gravity; ///< Apparent gravitation (i.e. with buoyancy) in world-coordinates
    Real mass; ///< Total inertial mass
    Real coeff_lift; ///< Coefficient of lift (DEFAULT: 0)
    Real coeff_drag; ///< Coefficient of drag
    // Constraints
    Real strictness; ///< Scale factor for constraint violation penalties
    Real min_height; ///< Minimum allowed position opposite gravity (DEFAULT: -infinity)
    Real min_airvel; ///< Minimum allowed velocity along the thrust direction (DEFAULT: -infinity)
    Real min_thrust; ///< Minimum allowed collective thrust command
    Real nom_thrust; ///< Nominal desired collective thrust command
    Real max_thrust; ///< Maximum allowed collective thrust command
    RealVec<3> lim_angvel; ///< Maximum allowed angular velocity command magnitude on each body axis
    // Objectives
    Real weight_thrust; ///< Importance weighting factor placed on aggressively off-nominal collective thrust commands
    RealVec<3> weight_angvel; ///< Importance weighting factors placed on aggressive body angular velocity commands
    RealVec<3> weight_poserr; ///< Importance weighting factors placed on world-coordinate position tracking error
    RealVec<3> weight_velerr; ///< Importance weighting factors placed on world-coordinate velocity tracking error
    Real weight_align_fore; ///< Importance weighting factor placed on alignment of the thrust direction and reference velocity (DEFAULT: 0)
    Real weight_align_nadir; ///< Importance weighting factor placed on alignment of the lift direction and gravity (DEFAULT: 0)
    //@}

    /// @name Derived
    //@{
    Real strictness2; ///< Memoized value of strictness squared
    RealVec<3> gdir; ///< Memoized direction vector of gravity
    RealMat<3,3> gdyad; ///< Memoized outer-product of the gravity direction with itself
    RealMat<3,3> coeffmat; ///< Memoized matrix of mass-normalized dynamical parameters
    RealMat<3,3> crossmat_fore; ///< Memoized weighted cross-product matrix for the forward direction
    RealMat<3,3> crossmat_nadir; ///< Memoized weighted cross-product matrix for the nadir direction
    //@}

public:
    /// @name Core
    //@{
    /// Constructs using a configuration YAML that recursively specifies all the "Given" attributes
    Model(YAML::Node const& cfg);
    //@}

    /// @name Utilities
    //@{
    /// Assigns the current event's predict-sensitivities and the next event's state and time
    void predict(Event& now, Event& next, Real timestep) const;
    /// Assigns the given event's cost and cost-sensitivities in comparison to the specified reference trajectory
    void cost(Event& event, Reference const& reference, Real timestep) const;
    //@}

    /// @name Access
    //@{
    inline Real get_min_height() const {return min_height;}
    inline Real get_min_airvel() const {return min_airvel;}
    inline Real get_min_thrust() const {return min_thrust;}
    inline Real get_max_thrust() const {return max_thrust;}
    inline RealVec<3> const& get_lim_angvel() const {return lim_angvel;}
    //@}
};

//////////////////////////////////////////////////

/// Trajectory optimizer for coordinating thrust and angular velocity commands to track a position path
class Planner {
    /// @name Given
    //@{
    Real  timestep; ///< Temporal discretization size for the prediction model
    Real  horizon;  ///< Total time into the future to optimize over
    Real  prox_tol; ///< Maximum tracking error for which the reference time will proceed
    Real  cost_tol; ///< Maximum relative cost improvement between iterations for which the plan is considered converged
    Int   max_iter; ///< Maximum number of iterations per optimization
    Model model;    ///< Predicts and judges event trajectories
    //@}

    /// @name Derived
    //@{
    Reference reference; ///< Desired trajectory to track (assumed origin until explicitly set)
    Seq<Event> plan; ///< Latest planned sequence of events (meaningless until optimized)
    Real bill; ///< Total sum of costs for the planned events (infinite before first optimization per reference change)
    Real regularizer; ///< Levenberg-Marquardt buffer for Hessian inversions
    Seq<Event> candidate; ///< Newly optimized plan before validation
    Seq<RealVec<Action::DIM>> policy_bias; ///< Time-varying feedforwards for the local policy delta
    Seq<RealMat<Action::DIM, State::DIM>> policy_gain; ///< Time-varying feedback-gains for the local policy delta
    //@}

public:
    /// @name Core
    //@{
    /// Constructs using a configuration YAML that recursively specifies all the "Given" attributes
    Planner(YAML::Node const& cfg);
    /// Improves the plan for starting at the given state and progresses the reference by the given duration (returns success)
    bool optimize(State const& state0, Real proceed=0.0);
    //@}

    /// @name Access
    //@{
    inline Real get_timestep() const {return timestep;}
    inline Real get_horizon() const {return horizon;}
    inline Real get_prox_tol() const {return prox_tol;}
    inline Model const& get_model() const {return model;}
    inline Reference const& get_reference() const {return reference;}
    inline Seq<Event> const& get_plan() const {return plan;}
    inline Real get_bill() const {return bill;}
    inline Real get_regularizer() const {return regularizer;}
    inline Seq<RealMat<Action::DIM, State::DIM>> const& get_policy() const {return policy_gain;}
    inline void set_reference(YAML::Node const& cfg) {set_reference(Reference(cfg));}
    void set_reference(Reference const& new_reference);
    void print_plan() const;
    //@}
};

//////////////////////////////////////////////////

} // namespace guidance
} // namespace unitair
