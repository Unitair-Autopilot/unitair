/** @file
This ROS node is a Stokesim plugin. It provides Unitair with an interface
to Stokesim that is essentially identical to its interface with hardware
sensors and actuators. It also publishes simulation-only insights, like
ground-truth state.
*/
#include <ros/ros.h>
#include <ros/master.h>
#include <tf/transform_broadcaster.h>
#include <stokesim/io/plugin.h>
#include "tools/tools.hpp"

#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <unitair/Dictionary.h>
#include <unitair/Actuation.h>
#include <unitair/Battery.h>
#include <unitair/State.h>
#include <unitair/MotionCapture.h>
#include <unitair/Accelerometer.h>
#include <unitair/Gyroscope.h>
#include <unitair/Magnetometer.h>
#include <unitair/Barometer.h>
#include <unitair/Pitot.h>
#include <unitair/GPS.h>

namespace unitair {
namespace simulation {

//////////////////////////////////////////////////

/// Establishes a ROS node for communicating with Stokesim
class StokesimNode : public stokesim::io::Plugin {
    /// @name Given
    //@{
    Str craft_name; ///< Name of the UAV of interest within Stokesim
    bool use_craft_namespace; ///< Whether or not to include the craft_name in advertised topic names (helpful when simulating multiple crafts)
    Seq<Str> actuator_order; ///< Correspondence between the actuators in Stokesim and a unitair::Actuation message
    //@}

    /// @name Derived
    //@{
    Str tns; ///< Namespace for all advertised ROS topics
    ros::Time ros_time; ///< The current ROS-clock time
    Ptr<tf::TransformBroadcaster> tfbr; ///< For publishing TF coordinate-system information, mainly used by RViz
    ros::Publisher state_pub; ///< Publisher for truth body state information
    ros::Publisher battery_pub; ///< Publisher for truth battery state
    Dict<ros::Publisher> wrench_pubs; ///< Publishers for each actuator's truth wrench
    Dict<ros::Publisher> sensor_state_pubs; ///< Publishers for each sensor's truth internal state
    Dict<ros::Publisher> sensor_pubs; ///< Publishers for each sensor's measurements
    Dict<uint32_t> sensor_seqs; ///< Last sequence number seen for each sensor
    ros::Subscriber actuation_sub; ///< Subscriber for actuator effort command information
    ros::Subscriber kill_sub; ///< Subscriber for kill-toggling strings, e.g. "some_actuator", "some_sensor", or "craft"
    stokesim::physics::Craft const* pcraft; ///< Pointer to the (read-only) Stokesim craft with craft_name
    Plugin::Output output; ///< Current response to give Stokesim based on the latest input messages received
    //@}

public:
    /// @name Core
    //@{
    /// Configuration function called once at the beginning of the simulation
    bool configure(YAML::Node const& cfg, stokesim::physics::World const& world) override {
        // Read plugin-specific data from the config YAML with proper error handling
        craft_name = stokesim::yget("craft_name", cfg).as<Str>();
        use_craft_namespace = stokesim::yget("use_craft_namespace", cfg).as<bool>();
        actuator_order = stokesim::yget("actuator_order", cfg).as<Seq<Str>>();
        // Set the topic namespace
        tns = "simulation";
        if (use_craft_namespace) {tns += "/"+craft_name;}
        // Verify that the world actually has the specified craft and then get it
        if (!world.has_craft(craft_name)) {
            ROS_ERROR_STREAM(tns+": world is missing craft with name '"+craft_name+"'");
            return false;
        }
        pcraft = &(world.get_craft(craft_name));
        // Initialize this ROS node
        int ros_argc = 0; // must be an lvalue for some reason
        ros::init(ros_argc, nullptr, "stokesim"); // lack of access to argv precludes ROS Launch topic-remapping
        // Connect to ROS
        if (not ros::master::check()) {
            ROS_ERROR_STREAM(tns+": could not connect to ROS Master");
            return false;
        }
        ros::NodeHandle nh;
        tfbr = std::make_shared<tf::TransformBroadcaster>();
        // Establish publishers
        state_pub = nh.advertise<unitair::State>(tns+"/truth/state", 1);
        battery_pub = nh.advertise<unitair::Battery>(tns+"/truth/battery", 1);
        for (Str const& actuator_name : pcraft->actuator_names) {
            // Each actuator publishes a ROS Wrench message
            wrench_pubs.insert({actuator_name, nh.advertise<geometry_msgs::WrenchStamped>(tns+"/truth/wrench/"+actuator_name, 1)});
        }
        for (Str const& sensor_name : pcraft->sensor_names) {
            stokesim::physics::Sensor const& sensor = pcraft->get_sensor(sensor_name);
            // If the sensor has internal states, provide that truth
            if (sensor.has_states()) {
                sensor_state_pubs.insert({sensor_name, nh.advertise<unitair::Dictionary>(tns+"/truth/state/"+sensor_name, 1)});
            }
            // Sort measurement publishers based on type
            if (sensor.type == "accelerometer") {
                sensor_pubs.insert({sensor_name, nh.advertise<unitair::Accelerometer>(tns+"/measured/"+sensor_name, 3)});
            } else if (sensor.type == "gyroscope") {
                sensor_pubs.insert({sensor_name, nh.advertise<unitair::Gyroscope>(tns+"/measured/"+sensor_name, 3)});
            } else if (sensor.type == "magnetometer") {
                sensor_pubs.insert({sensor_name, nh.advertise<unitair::Magnetometer>(tns+"/measured/"+sensor_name, 3)});
            } else if (sensor.type == "barometer") {
                sensor_pubs.insert({sensor_name, nh.advertise<unitair::Barometer>(tns+"/measured/"+sensor_name, 3)});
            } else if (sensor.type == "pitot") {
                sensor_pubs.insert({sensor_name, nh.advertise<unitair::Pitot>(tns+"/measured/"+sensor_name, 3)});
            } else if (sensor.type == "motion_capture") {
                sensor_pubs.insert({sensor_name, nh.advertise<unitair::MotionCapture>(tns+"/measured/"+sensor_name, 3)});
            } else if (sensor.type == "gps") {
                sensor_pubs.insert({sensor_name, nh.advertise<unitair::GPS>(tns+"/measured/"+sensor_name, 3)});
            }
            // Sequence numbers keep track of when each sensor's measurement should be new
            sensor_seqs.insert({sensor_name, 0});
        }
        // Establish subscribers
        actuation_sub = nh.subscribe("control/actuation", 3, &StokesimNode::actuation_callback, this);
        kill_sub = nh.subscribe(tns+"/kill", 3, &StokesimNode::kill_callback, this);
        return true;
    }

    /// Coroutine function called every Plugin::call_period simulation seconds
    Plugin::Output work(stokesim::physics::World const& world) override {
        // Handle subscriber queues
        ros::spinOnce();
        // Record current time for stamping publications
        ros_time = ros::Time::now();
        // Publish truth state and corresponding TFs
        stokesim::physics::Body const& body = pcraft->body;
        tools::Euler const body_rpy(body.ori0.conjugate() * body.ori, true);
        unitair::State state_msg;
        state_msg.stamp = ros_time;
        state_msg.pos = tools::ros_point_from_realvec3(body.pos);
        state_msg.vel = tools::ros_vector3_from_realvec3(body.vel);
        state_msg.accel = tools::ros_vector3_from_realvec3(body.veldot);
        state_msg.ori = tools::ros_quaternion_from_quat(body.ori);
        state_msg.angvel = tools::ros_vector3_from_realvec3(body.avel);
        state_msg.gravity = tools::ros_vector3_from_realvec3(world.env.geoid.ecef_enu * world.env.geoid.gravity_at_ecef(body.pos_ecef));
        state_msg.wind = tools::ros_vector3_from_realvec3(world.env.geoid.ecef_enu * world.env.atmos.wind_at_lla(body.lla));
        state_msg.pressure = world.env.atmos.pressure_at_alt(body.lla.alt);
        state_msg.lla.x = body.lla.lat;    state_msg.lla.y = body.lla.lon;     state_msg.lla.z = body.lla.alt;
        state_msg.rpy.x = body_rpy.roll(); state_msg.rpy.y = body_rpy.pitch(); state_msg.rpy.z = body_rpy.yaw();
        state_pub.publish(state_msg);
        tfbr->sendTransform(tools::make_tf(body.pos, body.ori, tns+"/truth/enu", tns+"/truth/body", ros_time));
        tfbr->sendTransform(tools::make_tf(RealVec<3>::Zero(), Quat::Identity(), tns+"/truth/enu", "navigation/world", ros_time));
        // Publish truth battery
        Real const battery_fraction = pcraft->energy / pcraft->energy0;
        unitair::Battery battery_msg;
        battery_msg.stamp = ros_time;
        battery_msg.voltage = pcraft->voltage0 * battery_fraction;
        battery_msg.percent = 100*battery_fraction;
        battery_pub.publish(battery_msg);
        // Publish truth wrenches and corresponding TFs
        for (Str const& actuator_name : pcraft->actuator_names) {
            stokesim::physics::Actuator const& actuator = pcraft->get_actuator(actuator_name);
            geometry_msgs::WrenchStamped wrench_msg;
            wrench_msg.header.stamp = ros_time;
            wrench_msg.header.frame_id = tns+"/truth/actuators/"+actuator_name;
            wrench_msg.wrench.force = tools::ros_vector3_from_realvec3(actuator.force);
            wrench_msg.wrench.torque = tools::ros_vector3_from_realvec3(actuator.torque);
            wrench_pubs.at(actuator_name).publish(wrench_msg);
            if (actuator.type == "thruster") {
                // Cast to obtain thruster-specific information
                auto const& thruster = static_cast<stokesim::physics::actuator_types::Thruster const&>(actuator);
                tfbr->sendTransform(tools::make_tf(thruster.pos, Quat::Identity(), tns+"/truth/body", wrench_msg.header.frame_id, ros_time));
            } else if (actuator.type == "surface") {
                // Cast to obtain surface-specific information
                auto const& surface = static_cast<stokesim::physics::actuator_types::Surface const&>(actuator);
                RealVec<3> const surface_center = surface.origin + surface.orient*surface.cenpres;
                tfbr->sendTransform(tools::make_tf(surface_center, surface.orient, tns+"/truth/body", wrench_msg.header.frame_id, ros_time));
            }
        }
        // Publish sensor info
        for (Str const& sensor_name : pcraft->sensor_names) {
            stokesim::physics::Sensor const& sensor = pcraft->get_sensor(sensor_name);
            // Current truth internal states
            if (sensor.has_states()) {
                unitair::Dictionary sensor_state_msg;
                sensor_state_msg.stamp = ros_time;
                for (Str const& state_name : sensor.state_names) {
                    sensor_state_msg.names.push_back(state_name);
                    sensor_state_msg.values.push_back(sensor.get_state(state_name));
                }
                sensor_state_pubs.at(sensor_name).publish(sensor_state_msg);
            }
            // New measurements
            if (sensor_seqs.at(sensor_name) != sensor.sequence_number) {
                sensor_seqs.at(sensor_name) = sensor.sequence_number;
                if (sensor.type == "accelerometer") {
                    unitair::Accelerometer accelerometer_msg;
                    accelerometer_msg.stamp = ros_time;
                    accelerometer_msg.accel.x = sensor.get_measurement("x");
                    accelerometer_msg.accel.y = sensor.get_measurement("y");
                    accelerometer_msg.accel.z = sensor.get_measurement("z");
                    sensor_pubs.at(sensor_name).publish(accelerometer_msg);
                } else if (sensor.type == "gyroscope") {
                    unitair::Gyroscope gyroscope_msg;
                    gyroscope_msg.stamp = ros_time;
                    gyroscope_msg.angvel.x = sensor.get_measurement("x");
                    gyroscope_msg.angvel.y = sensor.get_measurement("y");
                    gyroscope_msg.angvel.z = sensor.get_measurement("z");
                    sensor_pubs.at(sensor_name).publish(gyroscope_msg);
                } else if (sensor.type == "magnetometer") {
                    unitair::Magnetometer magnetometer_msg;
                    magnetometer_msg.stamp = ros_time;
                    magnetometer_msg.bfield.x = sensor.get_measurement("x");
                    magnetometer_msg.bfield.y = sensor.get_measurement("y");
                    magnetometer_msg.bfield.z = sensor.get_measurement("z");
                    sensor_pubs.at(sensor_name).publish(magnetometer_msg);
                } else if (sensor.type == "barometer") {
                    unitair::Barometer barometer_msg;
                    barometer_msg.stamp = ros_time;
                    barometer_msg.pressure = sensor.get_measurement("pressure");
                    sensor_pubs.at(sensor_name).publish(barometer_msg);
                } else if (sensor.type == "pitot") {
                    unitair::Pitot pitot_msg;
                    pitot_msg.stamp = ros_time;
                    pitot_msg.pressure = sensor.get_measurement("differential_pressure");
                    pitot_msg.airspeed = sensor.get_measurement("airspeed");
                    sensor_pubs.at(sensor_name).publish(pitot_msg);
                } else if (sensor.type == "motion_capture") {
                    unitair::MotionCapture motioncapture_msg;
                    motioncapture_msg.stamp = ros_time;
                    motioncapture_msg.pos.x = sensor.get_measurement("pos_x");
                    motioncapture_msg.pos.y = sensor.get_measurement("pos_y");
                    motioncapture_msg.pos.z = sensor.get_measurement("pos_z");
                    motioncapture_msg.ori.w = sensor.get_measurement("ori_w");
                    motioncapture_msg.ori.x = sensor.get_measurement("ori_x");
                    motioncapture_msg.ori.y = sensor.get_measurement("ori_y");
                    motioncapture_msg.ori.z = sensor.get_measurement("ori_z");
                    sensor_pubs.at(sensor_name).publish(motioncapture_msg);
                } else if (sensor.type == "gps") {
                    unitair::GPS gps_msg;
                    gps_msg.stamp = ros_time;
                    gps_msg.fixed = true;
                    gps_msg.latitude = sensor.get_measurement("lat");
                    gps_msg.longitude = sensor.get_measurement("lon");
                    gps_msg.altitude = sensor.get_measurement("alt");
                    sensor_pubs.at(sensor_name).publish(gps_msg);
                }
            }
        }
        // Terminate Stokesim if ROS has stopped
        if (not ros::ok()) {output.terminate = true;}
        // Give back to Stokesim
        return output;
    }
    //@}

private:
    /// @name Callbacks
    //@{
    /// Processes messages on the actuation topic
    void actuation_callback(unitair::Actuation::ConstPtr const& msg) {
        // Validate message
        if (msg->efforts.size() != actuator_order.size()) {
            ROS_WARN_STREAM(tns+": ignored actuation message with invalid number of efforts");
            return;
        }
        // Write the corresponding effort commands to each actuator in the actuator order list
        for (Nat i=0; i<actuator_order.size(); ++i) {
            output.world_cmd.craft_cmds[craft_name].actuator_cmds[actuator_order[i]].values["effort"] = tools::clip(msg->efforts.at(i), -1.0, 1.0);
        }
    }

    /// Processes messages on the kill topic
    void kill_callback(std_msgs::String::ConstPtr const& msg) {
        // Toggle kill for the whole craft?
        if ((msg->data == craft_name) || (msg->data == "craft") || (msg->data == "")) {
            output.world_cmd.craft_cmds[craft_name].kill = !(pcraft->killed);
            ROS_INFO_STREAM(tns+": toggled kill for craft '"+craft_name+"'");
        // or just an actuator?
        } else if (pcraft->has_actuator(msg->data)) {
            output.world_cmd.craft_cmds[craft_name].actuator_cmds[msg->data].kill = !(pcraft->get_actuator(msg->data).killed);
            ROS_INFO_STREAM(tns+": toggled kill for actuator '"+msg->data+"' of craft '"+craft_name+"'");
        // or just a sensor?
        } else if (pcraft->has_sensor(msg->data)) {
            output.world_cmd.craft_cmds[craft_name].sensor_cmds[msg->data].kill = !(pcraft->get_sensor(msg->data).killed);
            ROS_INFO_STREAM(tns+": toggled kill for sensor '"+msg->data+"' of craft '"+craft_name+"'");
        } else {
            ROS_WARN_STREAM(tns+": no actuator or sensor with name '"+msg->data+"' found for craft '"+craft_name+"'");
        }
    }
    //@}
};

//////////////////////////////////////////////////

} // namespace simulation
} // namespace unitair

/// Creates "factory functions" needed by Stokesim to construct this class
EXPORT_AS_PLUGIN(unitair::simulation::StokesimNode)
