/** @file
Declaration of the Estimator and supporting classes.
*/
#pragma once
#include "tools/tools.hpp"

namespace unitair {
namespace navigation {

//////////////////////////////////////////////////

/// Container for the belief state at a specific time
struct State {
    /// Semantic names for each dimension of the manifold (DIM is total dimensionality)
    enum Dimensions {
                POS_X,         POS_Y,         POS_Z,
                VEL_X,         VEL_Y,         VEL_Z,
              ACCEL_X,       ACCEL_Y,       ACCEL_Z,
                ORI_X,         ORI_Y,         ORI_Z,
             ANGVEL_X,      ANGVEL_Y,      ANGVEL_Z,
         BIAS_ACCEL_X,  BIAS_ACCEL_Y,  BIAS_ACCEL_Z,
        BIAS_ANGVEL_X, BIAS_ANGVEL_Y, BIAS_ANGVEL_Z,
            GRAVITY_X,     GRAVITY_Y,     GRAVITY_Z,
               WIND_X,        WIND_Y,        WIND_Z,
             PRESSURE,
    DIM};
    /// @name Mean
    //@{
    RealVec<3> pos;         ///< Position in world-coordinates of the body-origin
    RealVec<3> vel;         ///< Velocity in body-coordinates of the body-origin relative to the world-frame
    RealVec<3> accel;       ///< Acceleration in body-coordinates of the body-origin relative to the world-frame
          Quat ori;         ///< Orientation of body-coordinates relative to world-coordinates
    RealVec<3> angvel;      ///< Angular velocity in body-coordinates of the body-frame relative to the world-frame
    RealVec<3> bias_accel;  ///< Bias of the inertial acceleration input
    RealVec<3> bias_angvel; ///< Bias of the inertial angular velocity input
    RealVec<3> gravity;     ///< Local gravitational field in world-coordinates
    RealVec<3> wind;        ///< Velocity in world-coordinates of the local wind relative to the world-frame
          Real pressure;    ///< Barometric pressure of the local atmosphere
    //@}
    /// Covariance matrix
    RealMat<DIM, DIM> covar;
    /// Time at which this belief was held
    Real when;
    /// Constructs uninitialized (does not guarantee zeros)
    State() =default;
    /// Constructs using the given YAML, with covar either fully specified or as a diagonal of standard-deviations
    State(YAML::Node const& cfg);
    /// Perturbs the mean of this state by the given tangent vector
    void operator+=(RealVec<DIM> const& d);
    /// Computes this state with mean perturbed by the given tangent vector
    State operator+(RealVec<DIM> const& d) const;
    /// Computes the tangent vector that would perturb the given state's mean to this state's mean
    RealVec<DIM> operator-(State const& other) const;
    /// Displays the contents of this state to the console
    void print() const;
};

//////////////////////////////////////////////////

/// Abstract base for input-driven stochastic process models
struct Process {
    /// Runtime-accessible record of the derived type
    Str const type;
    /// Required input dimensionality
    Int const input_dim;
    /// Propagates the given belief state forward by the given timestep under the given input (returns validity of input)
    virtual bool evolve(State& state, RealVecN const& input, Real timestep) const =0;
    /// Constructs using the given YAML and input dimensionality
    Process(YAML::Node const& cfg, Int input_dim) :
        type(tools::yget("type", cfg).as<Str>()),
        input_dim(input_dim) {
    }
};

/////////////////////////

/// Abstract base for sensors / information sources
struct Informant {
    /// Runtime-accessible record of the derived type
    Str const type;
    /// Required information dimensionality
    Int const info_dim;
    /// Conditions the given belief state on the given information (returns validity of info)
    virtual bool fuse(State& state, RealVecN const& info) const =0;
    /// Constructs using the given YAML and info dimensionality
    Informant(YAML::Node const& cfg, Int info_dim) :
        type(tools::yget("type", cfg).as<Str>()),
        info_dim(info_dim) {
    }
};

//////////////////////////////////////////////////

/// Manages evolving and informing the belief state through time
class Estimator {
    /// @name Given
    //@{
    State const state0; ///< Initial belief state
    Ptr<Process> process; ///< Model for how the belief state evolves without observation
    Dict<Ptr<Informant>> informants; ///< Dictionary of information sources that will be fused into the belief state
    //@}

    /// @name Derived
    //@{
    Seq<Str> informant_names; ///< List of valid keys for the informants dictionary
    State state; ///< Current belief state
    //@}

public:
    /// @name Core
    //@{
    /// Constructs using a configuration YAML that recursively specifies all the "Given" attributes
    Estimator(YAML::Node const& cfg);
    /// Propagates the current belief state forward by the given timestep under the given input (returns validity of input)
    bool evolve(RealVecN const& input, Real timestep) {return process->evolve(state, input, timestep);}
    /// Conditions the current belief state on the given information from the specified informant (returns validity of info)
    bool fuse(Str const& name, RealVecN const& info) {return informants.at(name)->fuse(state, info);}
    /// Abruptly replaces the current belief state with the given one (returns success)
    bool reset_state(State const& other_state) {this->state = other_state; return true;}
    //@}

    /// @name Access
    //@{
    inline      Real        get_time()                     const {return state.when;}
    inline     State const& get_state()                    const {return state;}
    inline     State const& get_state0()                   const {return state0;}
    inline   Process const& get_process()                  const {return *process;}
    inline Informant const& get_informant(Str const& name) const {return *informants.at(name);}
    inline  Seq<Str> const& get_informant_names()          const {return informant_names;}
    inline      bool        has_informant(Str const& name) const {return informants.count(name);}
    //@}
};

//////////////////////////////////////////////////

/// Convenience base classes for inference methods that may be common to multiple processes / informants
namespace inference_methods {

/////////////////////////

/// Extended Kalman filter (exact inference on a linearized HMM)
class EKF {
protected:
    /// @name Core
    //@{
    mutable RealMatS<Eigen::RowMajor> J; ///< Jacobian for the linearization of the process / observation model
    mutable RealMatS<Eigen::ColMajor> C; ///< Covariance matrix for the process / observation model additive noise
    /// Computes the next state covariance given the current state covariance P and timestep dt (used by processes)
    RealMat<State::DIM, State::DIM> predict_covar(RealMat<State::DIM, State::DIM> const& P, Real dt) const;
    /// Corrects the given state x with the given innovation e (returns success) (used by informants)
    bool correct_state(State& x, RealVecN const& e) const;
    //@}

public:
    /// @name Access
    //@{
    inline RealMatS<Eigen::RowMajor> const& get_J() const {return J;}
    inline RealMatS<Eigen::ColMajor> const& get_C() const {return C;}
    //@}
};

/////////////////////////

} // namespace inference_methods

//////////////////////////////////////////////////

} // namespace navigation
} // namespace unitair
