/** @file
    Implementation of pwm driver ROS node
*/

#include "hardware/common/hardware_node_base.hpp"
#include "hardware/pwm_driver/PCA9685.h"

#include <yaml-cpp/yaml.h>
#include "tools/tools.hpp"

// ROS messages
#include <unitair/Actuation.h>
#include <unitair/Health.h>
#include <unitair/Trigger.h>


namespace unitair {
namespace hardware {


class PWMDriverNode : public HardwareNode <unitair::Health> {

public:

    // subscribers
    ros::Subscriber _actuator_command;

    // services
    ros::ServiceServer _enable_srv;
    bool _enabled = true;

    // device driver
    std::unique_ptr<PCA9685> _device_driver;


    PWMDriverNode(std::string config_file_path){

        _actuator_command = _nh.subscribe("control/actuation", 10, &PWMDriverNode::actuator_callback, this);
        _enable_srv = _nh.advertiseService("pwm_driver/enable", &PWMDriverNode::enable, this);

        auto config = YAML::LoadFile(config_file_path);
        auto device_params = tools::yget("pwm_driver", config);
        _device_driver = std::make_unique<PCA9685>(tools::yget("i2c_device", device_params).as<std::string>());
        _device_driver->_output_trims = tools::yget("pwm_mapping", device_params).as<std::vector<std::vector<double>>>();
        if(not _device_driver->configure(tools::yget("pwm_frequency", device_params).as<double>())){
            ROS_ERROR("PWM: Failed to configure device");
        }
    };


    ~PWMDriverNode(){

        if(_device_driver){
            //set all outputs to zero
            std::vector<double> actuator_commands(_device_driver->_NUM_CHANNELS, 0);
            _device_driver->update_outputs(actuator_commands);
        }
    }


    void actuator_callback(const unitair::Actuation::ConstPtr& msg) {

        std::vector<double> commands(msg->efforts.size(), 0.0);
        if (_enabled) {
            for (uint i=0; i<commands.size(); ++i) {
                double const command = msg->efforts[i];
                if (fabs(command) > 1.001) {
                    ROS_ERROR("PWM: Actuator command greater than acceptable range [-1,1] setting to zero (received %f)", command);
                    commands[i] = 0.0;
                } else {
                    commands[i] = command;
                }
            }
        }

        if (not _device_driver->update_outputs(commands)) {
            ROS_ERROR("PWM: Could not set outputs");
        }
    }


    void health_loop(const ros::TimerEvent& event){

        auto now = ros::Time::now();
        unitair::Health health_msg;
        health_msg.stamp = now;
        health_msg.node_name = "pwm_driver";
        health_msg.labels.push_back("Manually disabled");
        if (_enabled) {health_msg.requests.push_back(Health::PASS);}
        else {health_msg.requests.push_back(Health::WARN);}
        //!TODO: add more health info
        publish_health(health_msg);
    }


    void main_loop(const ros::TimerEvent& event){
        //disabled
    }


    bool reconfigure(unitair::Reconfigure::Request& req,
                     unitair::Reconfigure::Response& res) {

        std::string cfg_file;
        if (req.cfg_path.empty()) {ros::param::get("cfg_path", cfg_file);}
        else {cfg_file = req.cfg_path;}
        cfg_file += "/hardware.yaml";

        YAML::Node cfg;
        try {
            cfg = YAML::LoadFile(cfg_file);
        } catch (YAML::BadFile const&) {
            res.success = false;
            res.details = "Invalid or nonexistent cfg file '"+cfg_file+"'";
            return true;
        }

        try {
            YAML::Node device_params = tools::yget("pwm_driver", cfg);
            _device_driver->_output_trims = tools::yget("pwm_mapping", device_params).as<std::vector<std::vector<double>>>();
        } catch (std::runtime_error const&) {
            res.success = false;
            res.details = "Malformed cfg file '"+cfg_file+"' is missing 'pwm_driver/pwm_mapping'";
            return true;
        }

        res.success = true;
        res.details = "Updated 'pwm_driver/pwm_mapping'";
        return true;
    }


    bool enable(unitair::Trigger::Request& req,
                unitair::Trigger::Response& res) {
        _enabled = req.option;
        res.success = true;
        if (_enabled) {res.details = "PWM driver is enabled.";}
        else {res.details = "PWM driver is disabled.";}
        return true;
    }

};


} // namespace hardware
} // namespace unitair


using namespace unitair;
using namespace hardware;

int main(int argc, char** argv){

    std::string node_name = "pwm_driver";
    ros::init(argc, argv, node_name);

    std::string config_file_path;
    if(not ros::param::get("cfg_path", config_file_path)){
        ROS_ERROR("PWM: Failed to load cfg_path parameter");
        return 1;
    }

    PWMDriverNode node(config_file_path + "/hardware.yaml");
    node.configure(node_name, 0);

    ros::spin();

    return 0;
}
