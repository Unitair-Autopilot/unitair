/** @file
ROS node for fusing sensor information into a state estimate.
Currently, the process model must be IMU-driven.
*/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "navigation/estimator.hpp"
#include "navigation/informants/gps.hpp"

#include <unitair/Reconfigure.h>
#include <unitair/Health.h>
#include <unitair/State.h>

#include <unitair/Accelerometer.h>
#include <unitair/Gyroscope.h>
#include <unitair/MotionCapture.h>
#include <unitair/Magnetometer.h>
#include <unitair/Barometer.h>
#include <unitair/Pitot.h>
#include <unitair/GPS.h>

//////////////////////////////////////////////////

namespace unitair {
namespace navigation {

/// Manages the ROS frontend for the Estimator backend
class EstimatorNode {
    /// @name Given
    //@{
    Real timestep; ///< Process model integration timestep and measurement fusion period
    bool filter_stamps; ///< Whether or not to reject measurements stamped at times outside the current fusion period
    Ptr<Estimator> estimator; ///< Manages evolving and informing the belief state through time
    //@}

    /// @name Derived
    //@{
    // ROS handlers
    Ptr<ros::NodeHandle> rosnh; ///< For creating and managing ROS publisher/subscribers
    Ptr<tf::TransformBroadcaster> tfbr; ///< For publishing TF coordinate-system information, mainly used by RViz
    // Services
    ros::ServiceServer reconfigure_srv; ///< Service for reconfiguring the Given attributes
    // Publishers
    ros::Publisher health_pub; ///< Publishes the health status of this node
    ros::Publisher state_pub; ///< Publishes the estimator's belief state
    // Subscribers
    ros::Subscriber accelerometer_sub; ///< Subscribes to the accelerometer part of the IMU process input
    ros::Subscriber gyroscope_sub; ///< Subscribes to the gyroscope part of the IMU process input
    Dict<ros::Subscriber> informant_subs; ///< One subscriber for every informant the estimator was configured for
    // Aggregates
    RealVec<6> imu_input; ///< Latest concatenated vector of accelerometer and gyroscope readings (input to the estimator's process)
    // Timing management
    ros::Duration sleeper; ///< For sleeping this thread during downtime in the main-loop
    ros::Time start; ///< The ROS time at which the main-loop (run) began
    // Calculators
    Ptr<tools::Geoid const> geoid; ///< For converting the estimated position to LLA if GPS is being used
    //@}

    /// @name Callbacks
    //@{
    // Services
    bool reconfigure_callback(Reconfigure::Request& req, Reconfigure::Response& res);
    // Process
    void accelerometer_callback(Accelerometer::ConstPtr const& msg);
    void gyroscope_callback(Gyroscope::ConstPtr const& msg);
    // Informants
    void motioncapture_callback(MotionCapture::ConstPtr const& msg, Str const& informant_name);
    void magnetometer_callback(Magnetometer::ConstPtr const& msg, Str const& informant_name);
    void barometer_callback(Barometer::ConstPtr const& msg, Str const& informant_name);
    void pitot_callback(Pitot::ConstPtr const& msg, Str const& informant_name);
    void gps_callback(GPS::ConstPtr const& msg, Str const& informant_name);
    //@}

    /// @name Helpers
    //@{
    ros::Subscriber sub_for_informant(Str const& name);
    unitair::State msg_from_state(navigation::State const& state, ros::Time const& stamp) const;
    navigation::State state_from_msg(unitair::State const& msg) const;
    bool is_stamp_valid(ros::Time const& stamp) const;
    //@}

public:
    static Str const node_name; ///< Name of the node this class will initialize
    static Str const topic_namespace; ///< Namespace to prepend to advertised topics

    /// Constructs using a configuration YAML that recursively specifies all the "Given" attributes
    EstimatorNode(YAML::Node const& cfg) :
        timestep(tools::yget("timestep", cfg).as<Real>()),
        filter_stamps(tools::yget("filter_stamps", cfg).as<bool>()),
        estimator(std::make_shared<Estimator>(tools::yget("estimator", cfg))) {
        // Initialize ROS handlers
        rosnh = std::make_shared<ros::NodeHandle>();
        tfbr = std::make_shared<tf::TransformBroadcaster>();
        sleeper = ros::Duration(timestep/100);
        // Initialize services
        reconfigure_srv = rosnh->advertiseService(node_name+"/reconfigure", &EstimatorNode::reconfigure_callback, this);
        // Initialize publishers
        health_pub = rosnh->advertise<Health>("health", 3);
        state_pub = rosnh->advertise<unitair::State>(topic_namespace+"/state", 3);
        // Initialize process subscribers
        accelerometer_sub = rosnh->subscribe("hardware/accelerometer", 1, &EstimatorNode::accelerometer_callback, this, ros::TransportHints().tcpNoDelay());
        gyroscope_sub = rosnh->subscribe("hardware/gyroscope", 1, &EstimatorNode::gyroscope_callback, this, ros::TransportHints().tcpNoDelay());
        // Initialize informant subscribers
        for (Str const& name : estimator->get_informant_names()) {
            informant_subs.insert({name, sub_for_informant(name)});
        }
        // Wait for IMU to begin
        imu_input.setZero();
        Health health_msg;
        health_msg.node_name = node_name;
        health_msg.labels.push_back("Received first IMU messages on '"+accelerometer_sub.getTopic()+"' and '"+gyroscope_sub.getTopic()+"'");
        health_msg.requests.push_back(Health::WAIT);
        while ((accelerometer_sub.getNumPublishers() < 1) or (gyroscope_sub.getNumPublishers() < 1)) { // blocks until IMU begins or ROS dies / kills this node
            if (not ros::ok()) {return;}
            health_msg.stamp = ros::Time::now();
            health_pub.publish(health_msg);
            ros::Duration(0.25).sleep();
        }
        health_msg.requests.back() = Health::PASS;
        health_msg.stamp = ros::Time::now();
        health_pub.publish(health_msg);
    }

    /// Main-loop (blocks until ROS dies / kills this node)
    void run() {
        start = ros::Time::now();
        while (ros::ok()) {
            // Record the current time
            ros::Time const now = ros::Time::now();
            // If a timestep has passed...
            if ((now - start).toSec() > (estimator->get_time() + timestep)) {
                // Evolve the state to the next timestep
                ROS_ASSERT(estimator->evolve(imu_input, timestep));
                // Publish the current state
                navigation::State const& state = estimator->get_state();
                state_pub.publish(msg_from_state(state, now));
                tfbr->sendTransform(tools::make_tf(state.pos, state.ori, topic_namespace+"/world", topic_namespace+"/body", now));
            }
            // Fuse available sensor information
            ros::spinOnce();
            // Don't burn a whole CPU checking for new messages
            sleeper.sleep();
        }
    }
};

Str const EstimatorNode::node_name("estimator");
Str const EstimatorNode::topic_namespace("navigation");

} // namespace navigation
} // namespace unitair

//////////////////////////////////////////////////

using namespace unitair;
using namespace navigation;

/// Entry point
int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, EstimatorNode::node_name);
    // Load configuration
    Str cfg_file;
    if (not ros::param::get("cfg_path", cfg_file)) {
        ROS_ERROR_STREAM(EstimatorNode::node_name << ": failed to load the cfg_path ROS-param");
        return 1;
    }
    cfg_file += "/navigation.yaml";
    YAML::Node const cfg = YAML::LoadFile(cfg_file);
    // Run the node
    EstimatorNode(cfg).run();
    return 0;
}

//////////////////////////////////////////////////

bool EstimatorNode::reconfigure_callback(Reconfigure::Request& req, Reconfigure::Response& res) {
    // Use ROS-param "cfg_path" if the requested cfg_path is empty
    Str cfg_file;
    if (req.cfg_path.empty()) {ros::param::get("cfg_path", cfg_file);}
    else {cfg_file = req.cfg_path;}
    cfg_file += "/navigation.yaml";
    // Safely load the YAML
    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(cfg_file);
    } catch (YAML::BadFile const& error) {
        res.success = false;
        res.details = "Invalid or nonexistent cfg file '"+cfg_file+"'";
        return true;
    }
    // Safely reconfigure the Given attributes
    try {
        timestep = tools::yget("timestep", cfg).as<Real>();
        filter_stamps = tools::yget("filter_stamps", cfg).as<bool>();
        estimator = std::make_shared<Estimator>(tools::yget("estimator", cfg));
    } catch (std::runtime_error const& error) {
        // Respond failure
        res.success = false;
        res.details = "Invalid or incomplete cfg file '"+cfg_file+"' ("+error.what()+")";
        return true;
    }
    // Redo the informant subscribers
    for (auto& name_sub : informant_subs) {name_sub.second.shutdown();} // the fact this doesn't happen in ~Subscriber is a ROS bug... ugh
    informant_subs.clear();
    for (Str const& name : estimator->get_informant_names()) {
        informant_subs.insert({name, sub_for_informant(name)});
    }
    // Reset timing management
    sleeper = ros::Duration(timestep/100);
    start = ros::Time::now();
    // Respond success
    res.success = true;
    res.details = "Reloaded everything from '"+cfg_file+"'";
    return true;
}

/////////////////////////

void EstimatorNode::accelerometer_callback(Accelerometer::ConstPtr const& msg) {
    imu_input.head<3>() = tools::realvec3_from_ros_vector3(msg->accel);
}

/////

void EstimatorNode::gyroscope_callback(Gyroscope::ConstPtr const& msg) {
    imu_input.tail<3>() = tools::realvec3_from_ros_vector3(msg->angvel);
}

/////////////////////////

void EstimatorNode::motioncapture_callback(MotionCapture::ConstPtr const& msg, Str const& informant_name) {
    if (not is_stamp_valid(msg->stamp)) {return;}
    RealVec<7> info; info << msg->pos.x, msg->pos.y, msg->pos.z, msg->ori.w, msg->ori.x, msg->ori.y, msg->ori.z;
    ROS_ASSERT(estimator->fuse(informant_name, info));
}

/////

void EstimatorNode::magnetometer_callback(Magnetometer::ConstPtr const& msg, Str const& informant_name) {
    if (not is_stamp_valid(msg->stamp)) {return;}
    ROS_ASSERT(estimator->fuse(informant_name, tools::realvec3_from_ros_vector3(msg->bfield)));
}

/////

void EstimatorNode::barometer_callback(Barometer::ConstPtr const& msg, Str const& informant_name) {
    if (not is_stamp_valid(msg->stamp)) {return;}
    ROS_ASSERT(estimator->fuse(informant_name, RealVec<1>(msg->pressure)));
}

/////

void EstimatorNode::pitot_callback(Pitot::ConstPtr const& msg, Str const& informant_name) {
    if (not is_stamp_valid(msg->stamp)) {return;}
    if (msg->airspeed < -tools::EPS) {
        ROS_WARN_STREAM(node_name << ": ignoring negative airspeed measurement");
        return;
    }
    ROS_ASSERT(estimator->fuse(informant_name, RealVec<1>(msg->airspeed)));
}

/////

void EstimatorNode::gps_callback(GPS::ConstPtr const& msg, Str const& informant_name) {
    if (not is_stamp_valid(msg->stamp) or not msg->fixed) {return;}
    ROS_ASSERT(estimator->fuse(informant_name, RealVec<3>(msg->latitude, msg->longitude, msg->altitude)));
}

/////////////////////////

ros::Subscriber EstimatorNode::sub_for_informant(Str const& name) {
    // Get the informant type
    Informant const& informant = estimator->get_informant(name);
    // Make the appropriate subscriber based on the informant type
    if (informant.type == "MotionCapture") {
        return rosnh->subscribe<MotionCapture>("hardware/"+name, 1, boost::bind(&EstimatorNode::motioncapture_callback, this, _1, name));
    }
    if (informant.type == "Magnetometer") {
        return rosnh->subscribe<Magnetometer>("hardware/"+name, 1, boost::bind(&EstimatorNode::magnetometer_callback, this, _1, name));
    }
    if (informant.type == "Barometer") {
        return rosnh->subscribe<Barometer>("hardware/"+name, 1, boost::bind(&EstimatorNode::barometer_callback, this, _1, name));
    }
    if (informant.type == "Pitot") {
        return rosnh->subscribe<Pitot>("hardware/"+name, 1, boost::bind(&EstimatorNode::pitot_callback, this, _1, name));
    }
    if (informant.type == "GPS") {
        geoid = std::make_shared<tools::Geoid const>(static_cast<informants::GPS const&>(informant).get_geoid());
        return rosnh->subscribe<GPS>("hardware/"+name, 1, boost::bind(&EstimatorNode::gps_callback, this, _1, name));
    }
    ROS_ERROR_STREAM(node_name << ": ignoring unsupported informant type '" << informant.type << "' (named '" << name << "')");
    return ros::Subscriber();
}

/////

unitair::State EstimatorNode::msg_from_state(navigation::State const& state, ros::Time const& stamp) const {
    // Meta
    unitair::State msg;
    msg.stamp = stamp;
    // Means
    msg.pos         = tools::ros_point_from_realvec3(state.pos);
    msg.vel         = tools::ros_vector3_from_realvec3(state.vel);
    msg.accel       = tools::ros_vector3_from_realvec3(state.accel);
    msg.ori         = tools::ros_quaternion_from_quat(state.ori);
    msg.angvel      = tools::ros_vector3_from_realvec3(state.angvel);
    msg.bias_accel  = tools::ros_vector3_from_realvec3(state.bias_accel);
    msg.bias_angvel = tools::ros_vector3_from_realvec3(state.bias_angvel);
    msg.gravity     = tools::ros_vector3_from_realvec3(state.gravity);
    msg.wind        = tools::ros_vector3_from_realvec3(state.wind);
    msg.pressure    = state.pressure;
    // Standard-deviations
    RealVec<navigation::State::DIM> const stdevs = state.covar.diagonal().cwiseSqrt();
    msg.pos_stdev         = tools::ros_vector3_from_realvec3(stdevs.segment<3>(navigation::State::POS_X));
    msg.vel_stdev         = tools::ros_vector3_from_realvec3(stdevs.segment<3>(navigation::State::VEL_X));
    msg.accel_stdev       = tools::ros_vector3_from_realvec3(stdevs.segment<3>(navigation::State::ACCEL_X));
    msg.ori_stdev         = tools::ros_vector3_from_realvec3(stdevs.segment<3>(navigation::State::ORI_X));
    msg.angvel_stdev      = tools::ros_vector3_from_realvec3(stdevs.segment<3>(navigation::State::ANGVEL_X));
    msg.bias_accel_stdev  = tools::ros_vector3_from_realvec3(stdevs.segment<3>(navigation::State::BIAS_ACCEL_X));
    msg.bias_angvel_stdev = tools::ros_vector3_from_realvec3(stdevs.segment<3>(navigation::State::BIAS_ANGVEL_X));
    msg.gravity_stdev     = tools::ros_vector3_from_realvec3(stdevs.segment<3>(navigation::State::GRAVITY_X));
    msg.wind_stdev        = tools::ros_vector3_from_realvec3(stdevs.segment<3>(navigation::State::WIND_X));
    msg.pressure_stdev    = stdevs(navigation::State::PRESSURE);
    // Derived quantities
    if (geoid) {
        tools::LLA const lla_rad = geoid->lla_from_enu(state.pos);
        msg.lla.x = tools::RAD2DEG * lla_rad.lat;
        msg.lla.y = tools::RAD2DEG * lla_rad.lon;
        msg.lla.z = lla_rad.alt;
    } else {
        msg.lla = msg.pos;
    }
    tools::Euler const euler(estimator->get_state0().ori.conjugate() * state.ori, true);
    msg.rpy.x = euler.roll();
    msg.rpy.y = euler.pitch();
    msg.rpy.z = euler.yaw();
    // Finish
    return msg;
}

/////

navigation::State EstimatorNode::state_from_msg(unitair::State const& msg) const {
    // Meta
    navigation::State state;
    state.when = (msg.stamp - start).toSec();
    // Means
    state.pos         = tools::realvec3_from_ros_point(msg.pos);
    state.vel         = tools::realvec3_from_ros_vector3(msg.vel);
    state.accel       = tools::realvec3_from_ros_vector3(msg.accel);
    state.ori         = tools::quat_from_ros_quaternion(msg.ori);
    state.angvel      = tools::realvec3_from_ros_vector3(msg.angvel);
    state.bias_accel  = tools::realvec3_from_ros_vector3(msg.bias_accel);
    state.bias_angvel = tools::realvec3_from_ros_vector3(msg.bias_angvel);
    state.gravity     = tools::realvec3_from_ros_vector3(msg.gravity);
    state.wind        = tools::realvec3_from_ros_vector3(msg.wind);
    state.pressure    = msg.pressure;
    // Covariance
    RealVec<navigation::State::DIM> stdevs;
    stdevs.segment<3>(navigation::State::POS_X)         = tools::realvec3_from_ros_vector3(msg.pos_stdev);
    stdevs.segment<3>(navigation::State::VEL_X)         = tools::realvec3_from_ros_vector3(msg.vel_stdev);
    stdevs.segment<3>(navigation::State::ACCEL_X)       = tools::realvec3_from_ros_vector3(msg.accel_stdev);
    stdevs.segment<3>(navigation::State::ORI_X)         = tools::realvec3_from_ros_vector3(msg.ori_stdev);
    stdevs.segment<3>(navigation::State::ANGVEL_X)      = tools::realvec3_from_ros_vector3(msg.angvel_stdev);
    stdevs.segment<3>(navigation::State::BIAS_ACCEL_X)  = tools::realvec3_from_ros_vector3(msg.bias_accel_stdev);
    stdevs.segment<3>(navigation::State::BIAS_ANGVEL_X) = tools::realvec3_from_ros_vector3(msg.bias_angvel_stdev);
    stdevs.segment<3>(navigation::State::GRAVITY_X)     = tools::realvec3_from_ros_vector3(msg.gravity_stdev);
    stdevs.segment<3>(navigation::State::WIND_X)        = tools::realvec3_from_ros_vector3(msg.wind_stdev);
    stdevs(navigation::State::PRESSURE)                 = msg.pressure_stdev;
    state.covar.setZero();
    state.covar.diagonal() = stdevs.array().square();
    // Verify quaternion and finish
    if (state.ori.norm() < 1e-9) {
        ROS_WARN_STREAM(node_name << ": interpreting near-zero quaternion as identity");
        state.ori = Quat::Identity();
    }
    return state;
}

/////

bool EstimatorNode::is_stamp_valid(ros::Time const& stamp) const {
    // Checks whether the stamp is in the current fusion window from t to t+dt
    Real const arrival_time = (stamp - start).toSec();
    return ((estimator->get_time() < arrival_time) and (arrival_time < estimator->get_time()+timestep)) or not filter_stamps;
}

//////////////////////////////////////////////////
