/** @file
ROS node for coordinating thrust and attitude
commands to track a specified positional trajectory.
*/
#include <ros/ros.h>
#include "guidance/planner.hpp"

#include <unitair/Reconfigure.h>
#include <unitair/SetPath.h>
#include <unitair/State.h>

#include <unitair/Health.h>
#include <unitair/ThrustCmd.h>
#include <unitair/AttitudeCmd.h>
#include <unitair/Plan.h>

//////////////////////////////////////////////////

namespace unitair {
namespace guidance {

/// Manages the ROS frontend for the Planner backend
class PlannerNode {
    /// @name Given
    //@{
    Real rate; ///< Frequency at which the commands are published
    bool use_angaccel; ///< Whether or not to include angular acceleration in the attitude command
    Ptr<Planner> planner; ///< Performs the trajectory optimization
    //@}

    /// @name Derived
    //@{
    Ptr<ros::NodeHandle> rosnh; ///< For creating and managing ROS publisher/subscribers
    Ptr<ros::Rate> rateman; ///< Manages the publication rate
    ros::ServiceServer reconfigure_srv; ///< Service for reconfiguring the Given attributes
    ros::ServiceServer setpath_srv; ///< Service for setting the planner's reference path
    ros::Publisher health_pub; ///< Publishes the health status of this node
    ros::Publisher thrust_pub; ///< Publishes the planned collective thrust command
    ros::Publisher attitude_pub; ///< Publishes the planned attitude command
    ros::Publisher plan_pub; ///< Publishes the full planned trajectory for introspection
    ros::Subscriber state_sub; ///< Subscribes to the system state estimate
    State state0; ///< Latest state value to plan from
    bool got_state; ///< Flag for having received the first state
    bool got_path; ///< Flag for having received the first path
    //@}

    /// @name Callbacks
    //@{
    bool reconfigure_callback(Reconfigure::Request& req, Reconfigure::Response& res) {
        // Use ROS-param "cfg_path" if the requested cfg_path is empty
        Str cfg_file;
        if (req.cfg_path.empty()) {ros::param::get("cfg_path", cfg_file);}
        else {cfg_file = req.cfg_path;}
        cfg_file += "/guidance.yaml";
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
            rate = tools::yget("rate", cfg).as<Real>();
            rateman = std::make_shared<ros::Rate>(rate);
            use_angaccel = tools::yget("use_angaccel", cfg).as<bool>();
            Ptr<Planner> const new_planner = std::make_shared<Planner>(tools::yget("planner", cfg));
            new_planner->set_reference(planner->get_reference());
            planner = new_planner;
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

    bool setpath_callback(SetPath::Request& req, SetPath::Response& res) {
        // Default response if all goes well
        res.issue = SetPath::Response::NONE;
        res.details = "Accepted";
        // Convert the message path type
        Seq<RealVec<3>> req_path(req.path.size());
        for (Nat i=0; i<req_path.size(); ++i) {req_path[i] = tools::realvec3_from_ros_point(req.path[i]);}
        // Attempt construction and assignment of reference
        try {
            Ptr<Reference> new_reference;
            // Via speed
            if (req.speed > tools::EPS) {
                new_reference = std::make_shared<Reference>(req_path, req.speed, req.loop_index);
                res.details += " (via speed)";
            // Via timestep
            } else if (req.timestep > tools::EPS) {
                new_reference = std::make_shared<Reference>(req.timestep, req_path, req.loop_index);
                res.details += " (via timestep)";
            // Via times
            } else {
                new_reference = std::make_shared<Reference>(req.times, req_path, req.loop_index);
                res.details += " (via times)";
            }
            // Prepend lead-in and append closure
            if (got_state) {new_reference->connect(state0.pos, req.speed);}
            // Assign
            planner->set_reference(*new_reference);
            got_path = true;
        } catch (std::runtime_error const& error) {
            // Parsing broke, respond with failure
            res.issue = SetPath::Response::PARSE;
            res.details = error.what();
        }
        // Send response
        return true;
    }

    void state_callback(unitair::State::ConstPtr const& msg) {
        state0.pos = tools::realvec3_from_ros_point(msg->pos);
        state0.vel = tools::realvec3_from_ros_vector3(msg->vel);
        state0.ori = tools::quat_from_ros_quaternion(msg->ori);
        got_state = true;
    }
    //@}

public:
    static Str const node_name; ///< Name of the node this class will initialize
    static Str const topic_namespace; ///< Namespace to prepend to advertised topics

    /// Constructs using a configuration YAML that recursively specifies all the "Given" attributes
    PlannerNode(YAML::Node const& cfg) :
        rate(tools::yget("rate", cfg).as<Real>()),
        use_angaccel(tools::yget("use_angaccel", cfg).as<bool>()),
        planner(std::make_shared<Planner>(tools::yget("planner", cfg))),
        got_state(false),
        got_path(false) {
        rosnh = std::make_shared<ros::NodeHandle>();
        reconfigure_srv = rosnh->advertiseService(node_name+"/reconfigure", &PlannerNode::reconfigure_callback, this);
        setpath_srv = rosnh->advertiseService(topic_namespace+"/setpath", &PlannerNode::setpath_callback, this);
        health_pub = rosnh->advertise<Health>("health", 3);
        thrust_pub = rosnh->advertise<ThrustCmd>(topic_namespace+"/thrust", 3);
        attitude_pub = rosnh->advertise<AttitudeCmd>(topic_namespace+"/attitude", 3);
        plan_pub = rosnh->advertise<Plan>(topic_namespace+"/plan", 1);
        state_sub = rosnh->subscribe("navigation/state", 1, &PlannerNode::state_callback, this, ros::TransportHints().tcpNoDelay());
        // Wait for first state and path
        Health health_msg;
        health_msg.node_name = node_name;
        health_msg.labels.push_back("Received first state and path.");
        health_msg.requests.push_back(Health::WAIT);
        while (not (got_state and got_path)) {
            if (not ros::ok()) {return;}
            health_msg.stamp = ros::Time::now();
            health_pub.publish(health_msg);
            ros::Duration(0.25).sleep();
            ros::spinOnce();
        }
        health_msg.requests.back() = Health::PASS;
        health_msg.stamp = ros::Time::now();
        health_pub.publish(health_msg);
    }

    /// Main-loop (blocks until ROS dies / kills this node)
    void run() {
        Nat i = 0;
        rateman = std::make_shared<ros::Rate>(rate);
        while (ros::ok()) {
            // Get latest state and path
            ros::spinOnce();
            // Improve the plan
            planner->optimize(state0, 1.0/rate);
            Seq<Event> const& plan = planner->get_plan();
            ros::Time const ros_time = ros::Time::now();
            // Publish the commands
            ThrustCmd   thrust_msg;
            AttitudeCmd attitude_msg;
            thrust_msg.stamp = ros_time;
            attitude_msg.stamp = ros_time;
            thrust_msg.thrust = plan[0].action.thrust;
            attitude_msg.ori = tools::ros_quaternion_from_quat(plan[1].state.ori);
            attitude_msg.angvel = tools::ros_vector3_from_realvec3(plan[0].action.angvel);
            if (use_angaccel) {
                attitude_msg.angaccel = tools::ros_vector3_from_realvec3(
                    (plan[1].action.angvel - plan[0].action.angvel) / (plan[1].when - plan[0].when)
                );
            }
            thrust_pub.publish(thrust_msg);
            attitude_pub.publish(attitude_msg);
            // Publish the full plan (at a slower rate)
            if (i == 0) {
                Plan plan_msg;
                plan_msg.stamp = ros_time;
                for (Event const& pt : plan) {
                    plan_msg.time.push_back(pt.when);
                    plan_msg.pos.push_back(tools::ros_point_from_realvec3(pt.state.pos));
                    plan_msg.vel.push_back(tools::ros_vector3_from_realvec3(pt.state.vel));
                    plan_msg.ori.push_back(tools::ros_quaternion_from_quat(pt.state.ori));
                    plan_msg.thrust.push_back(pt.action.thrust);
                    plan_msg.angvel.push_back(tools::ros_vector3_from_realvec3(pt.action.angvel));
                }
                for (RealVec<3> const& r : planner->get_reference().get_path()) {
                    plan_msg.path.push_back(tools::ros_point_from_realvec3(r));
                }
                plan_msg.proxy = tools::ros_point_from_realvec3(planner->get_reference().at(0.0));
                plan_msg.looping = planner->get_reference().is_looping();
                plan_msg.bill = planner->get_bill();
                plan_msg.regularizer = planner->get_regularizer();
                plan_pub.publish(plan_msg);
            }
            // Rest
            rateman->sleep();
            i = (i+1)%5;
        }
    }
};

Str const PlannerNode::node_name("planner");
Str const PlannerNode::topic_namespace("guidance");

} // namespace guidance
} // namespace unitair

//////////////////////////////////////////////////

using namespace unitair;
using namespace guidance;

/// Entry point
int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, PlannerNode::node_name);
    // Load configuration
    Str cfg_file;
    if (not ros::param::get("cfg_path", cfg_file)) {
        ROS_ERROR_STREAM(PlannerNode::node_name << ": failed to load the cfg_path ROS-param");
        return 1;
    }
    cfg_file += "/guidance.yaml";
    YAML::Node const cfg = YAML::LoadFile(cfg_file);
    // Run the node
    PlannerNode(cfg).run();
    return 0;
}

//////////////////////////////////////////////////
