/** @file
ROS node for selecting an actuator effort allocation to achieve a desired wrench.
*/
#include <ros/ros.h>
#include "control/allocator.hpp"

#include <unitair/Reconfigure.h>
#include <unitair/Actuation.h>

#include <unitair/ThrustCmd.h>
#include <unitair/TorqueCmd.h>

//////////////////////////////////////////////////

namespace unitair {
namespace control {

/// Manages the ROS frontend for the Allocator backend
class AllocatorNode {
    /// @name Given
    //@{
    Ptr<Allocator> allocator; ///< Performs the allocation optimization
    //@}

    /// @name Derived
    //@{
    Ptr<ros::NodeHandle> rosnh; ///< For creating and managing ROS publisher/subscribers
    ros::ServiceServer reconfigure_srv; ///< Service for reconfiguring the Given attributes
    ros::Publisher actuation_pub; ///< Publishes the allocated actuator efforts
    ros::Subscriber thrust_sub; ///< Subscribes to the force part of the desired wrench
    ros::Subscriber torque_sub; ///< Subscribes to the torque part of the desired wrench and triggers new allocation
    RealVec<6> wrench; ///< Latest desired wrench
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
            allocator = std::make_shared<Allocator>(tools::yget("allocator", cfg));
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

    void thrust_callback(ThrustCmd::ConstPtr const& msg) {
        wrench(0) = msg->thrust;
    }

    void torque_callback(TorqueCmd::ConstPtr const& msg) {
        wrench(3) = msg->torque.x;
        wrench(4) = msg->torque.y;
        wrench(5) = msg->torque.z;
        // New allocation
        RealVecN const allocation = allocator->allocate(wrench);
        Actuation out;
        out.stamp = ros::Time::now();
        out.efforts = Seq<Real>(allocation.data(), allocation.data() + allocation.size());
        actuation_pub.publish(out);
    }
    //@}

public:
    static Str const node_name; ///< Name of the node this class will initialize
    static Str const topic_namespace; ///< Namespace to prepend to advertised topics

    /// Constructs using a configuration YAML that recursively specifies all the "Given" attributes
    AllocatorNode(YAML::Node const& cfg) :
        allocator(std::make_shared<Allocator>(tools::yget("allocator", cfg))),
        wrench(RealVec<6>::Zero()) {
        rosnh = std::make_shared<ros::NodeHandle>();
        reconfigure_srv = rosnh->advertiseService(node_name+"/reconfigure", &AllocatorNode::reconfigure_callback, this);
        actuation_pub = rosnh->advertise<Actuation>(topic_namespace+"/actuation", 3);
        thrust_sub = rosnh->subscribe("mission/thrust", 1, &AllocatorNode::thrust_callback, this, ros::TransportHints().tcpNoDelay());
        torque_sub = rosnh->subscribe("control/torque", 1, &AllocatorNode::torque_callback, this, ros::TransportHints().tcpNoDelay());
    }

    /// Main-loop (blocks until ROS dies / kills this node)
    void run() {
        ros::spin();
    }
};

Str const AllocatorNode::node_name("allocator");
Str const AllocatorNode::topic_namespace("control");

} // namespace control
} // namespace unitair

//////////////////////////////////////////////////

using namespace unitair;
using namespace control;

/// Entry point
int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, AllocatorNode::node_name);
    // Load configuration
    Str cfg_file;
    if (not ros::param::get("cfg_path", cfg_file)) {
        ROS_ERROR_STREAM(AllocatorNode::node_name << ": failed to load the cfg_path ROS-param");
        return 1;
    }
    cfg_file += "/control.yaml";
    YAML::Node const cfg = YAML::LoadFile(cfg_file);
    // Run the node
    AllocatorNode(cfg).run();
    return 0;
}

//////////////////////////////////////////////////
