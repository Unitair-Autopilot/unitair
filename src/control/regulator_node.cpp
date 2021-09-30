/** @file
ROS node for computing feedback torques to regulate attitude tracking error.
*/
#include <ros/ros.h>
#include "control/regulator.hpp"

#include <unitair/Reconfigure.h>
#include <unitair/Dictionary.h>
#include <unitair/TorqueCmd.h>

#include <unitair/AttitudeCmd.h>
#include <unitair/State.h>

//////////////////////////////////////////////////

namespace unitair {
namespace control {

/// Manages the ROS frontend for the Regulator backend
class RegulatorNode {
    /// @name Given
    //@{
    Ptr<Regulator> regulator; ///< Implements the feedback policy
    //@}

    /// @name Derived
    //@{
    Ptr<ros::NodeHandle> rosnh; ///< For creating and managing ROS publisher/subscribers
    ros::ServiceServer reconfigure_srv; ///< Service for reconfiguring the Given attributes
    ros::Publisher torque_pub; ///< Publishes the regulating torque command
    ros::Publisher adapt_pub; ///< Publishes the adaptation parameters (for introspection)
    ros::Subscriber target_sub; ///< Subscribes to the desired attitude state
    ros::Subscriber state_sub; ///< Subscribes to the actual system state
    Quat ori_target; ///< Latest desired orientation (zero implies ignore)
    RealVec<3> angvel_target; ///< Latest desired angular velocity
    RealVec<3> angaccel_target; ///< Latest desired angular acceleration
    ros::Time stamp; ///< Time of the latest state update
    //@}

    /// @name Callbacks
    //@{
    bool reconfigure_callback(Reconfigure::Request& req, Reconfigure::Response& res) {
        // Use ROS-param "cfg_path" if the requested cfg_path is empty
        Str cfg_file;
        if (req.cfg_path.empty()) {ros::param::get("cfg_path", cfg_file);}
        else {cfg_file = req.cfg_path;}
        cfg_file += "/control.yaml";
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
            regulator = std::make_shared<Regulator>(tools::yget("regulator", cfg));
        } catch (std::runtime_error const& error) {
            // Respond failure
            res.success = false;
            res.details = "Invalid or incomplete cfg file '"+cfg_file+"' ("+error.what()+")";
            return true;
        }
        // Respond success
        res.success = true;
        res.details = "Reloaded everything from '"+cfg_file+"'";
        return true;
    }

    void target_callback(AttitudeCmd::ConstPtr const& msg) {
        ori_target = tools::quat_from_ros_quaternion(msg->ori);
        angvel_target = tools::realvec3_from_ros_vector3(msg->angvel);
        angaccel_target = tools::realvec3_from_ros_vector3(msg->angaccel);
    }

    void state_callback(State::ConstPtr const& msg) {
        // Update time
        ros::Time const now = ros::Time::now();
        Real const timestep = (now - stamp).toSec();
        stamp = now;
        if ((timestep > 0.1) or (timestep < 0.0)) {return;} // skip anomalously delayed updates
        // Update state
        Quat const ori = tools::quat_from_ros_quaternion(msg->ori);
        RealVec<3> const angvel = tools::realvec3_from_ros_vector3(msg->angvel);
        RealVec<3> airvel = tools::realvec3_from_ros_vector3(msg->vel) - ori.conjugate()*tools::realvec3_from_ros_vector3(msg->wind);
        if (not airvel.allFinite()) {airvel.setZero();} // ignore invalid airvel estimates
        // Update regulation
        RealVec<3> const torque = regulator->regulate(ori, ori_target, angvel, angvel_target, angaccel_target, airvel, timestep);
        TorqueCmd torque_msg;
        torque_msg.stamp = stamp;
        torque_msg.torque = tools::ros_vector3_from_realvec3(torque);
        torque_pub.publish(torque_msg);
        // Provide adaptation parameters
        RealVec<9> adapt_params; adapt_params << regulator->get_aerospring_coeffs(), regulator->get_aerodamper_coeffs(), regulator->get_disturbance();
        Dictionary adapt_msg;
        adapt_msg.stamp = stamp;
        adapt_msg.names = Seq<Str>({"spring_x", "spring_y", "spring_z", "damper_x", "damper_y", "damper_z", "disturb_x", "disturb_y", "disturb_z"});
        adapt_msg.values = Seq<Real>(adapt_params.data(), adapt_params.data() + adapt_params.size());
        adapt_pub.publish(adapt_msg);
    }
    //@}

public:
    static Str const node_name; ///< Name of the node this class will initialize
    static Str const topic_namespace; ///< Namespace to prepend to advertised topics

    /// Constructs using a configuration YAML that recursively specifies all the "Given" attributes
    RegulatorNode(YAML::Node const& cfg) :
        regulator(std::make_shared<Regulator>(tools::yget("regulator", cfg))),
        ori_target(0, 0, 0, 0), // implies ignore
        angvel_target(0, 0, 0),
        angaccel_target(0, 0, 0) {
        rosnh = std::make_shared<ros::NodeHandle>();
        reconfigure_srv = rosnh->advertiseService(node_name+"/reconfigure", &RegulatorNode::reconfigure_callback, this);
        torque_pub = rosnh->advertise<TorqueCmd>(topic_namespace+"/torque", 3);
        adapt_pub = rosnh->advertise<Dictionary>(topic_namespace+"/adaptation", 1);
        target_sub = rosnh->subscribe("mission/attitude", 1, &RegulatorNode::target_callback, this, ros::TransportHints().tcpNoDelay());
        state_sub = rosnh->subscribe("navigation/state", 1, &RegulatorNode::state_callback, this, ros::TransportHints().tcpNoDelay());
        stamp = ros::Time::now();
    }

    /// Main-loop (blocks until ROS dies / kills this node)
    void run() {
        ros::spin();
    }
};

Str const RegulatorNode::node_name("regulator");
Str const RegulatorNode::topic_namespace("control");

} // namespace control
} // namespace unitair

//////////////////////////////////////////////////

using namespace unitair;
using namespace control;

/// Entry point
int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, RegulatorNode::node_name);
    // Load configuration
    Str cfg_file;
    if (not ros::param::get("cfg_path", cfg_file)) {
        ROS_ERROR_STREAM(RegulatorNode::node_name << ": failed to load the cfg_path ROS-param");
        return 1;
    }
    cfg_file += "/control.yaml";
    YAML::Node const cfg = YAML::LoadFile(cfg_file);
    // Run the node
    RegulatorNode(cfg).run();
    return 0;
}

//////////////////////////////////////////////////
