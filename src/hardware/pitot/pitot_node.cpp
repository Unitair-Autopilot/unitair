/** @file
    Implementation of pitot-tube ROS node
*/

#include "hardware/common/hardware_node_base.hpp"
#include "hardware/pitot/MS4525DO.h"

#include <yaml-cpp/yaml.h>
#include "tools/tools.hpp"

// ROS messages
#include "unitair/Pitot.h"
#include "unitair/Health.h"


namespace unitair {
namespace hardware {


class PitotNode : public HardwareNode <unitair::Health> {

public:

    // publisher
    ros::Publisher _pitot_publisher;

    // device driver
    std::unique_ptr<MS4525DO> _device_driver;

    // parameters
    struct param_struct{
        double sample_rate;
        double zero_pressure;
    } _params;


    PitotNode(std::string config_file_path){

        _pitot_publisher = _nh.advertise<unitair::Pitot>("hardware/pitot", 1);

        auto config = YAML::LoadFile(config_file_path);
        auto device_params = tools::yget("pitot", config);
        _params.sample_rate = tools::yget("sample_rate", device_params).as<double>();
        _params.zero_pressure = tools::yget("zero_pressure", device_params).as<double>();

        _device_driver = std::make_unique<MS4525DO>(tools::yget("i2c_device", device_params).as<std::string>());
    };


    void main_loop(const ros::TimerEvent& event){

        const auto now = ros::Time::now();
        const auto data = _device_driver->read_measurement();
        if (!data.is_valid) {
            return;
        }

        // publish result
        unitair::Pitot pitot_msg;
        pitot_msg.stamp = now;
        pitot_msg.pressure = tools::clip(data.differential_pressure - _params.zero_pressure, 0.0, tools::INF);
        pitot_msg.airspeed = std::sqrt((2.0*pitot_msg.pressure)/1.225); // https://en.wikipedia.org/wiki/Pitot_tube
        // pitot_msg.temperature = data.temperature;
        _pitot_publisher.publish(pitot_msg);
    }


    void health_loop(const ros::TimerEvent& event){

        //!TODO: add health info
        // auto now = ros::Time::now();
        // unitair::Health health_msg;
        // health_msg.stamp = now;
        // health_msg.node_name = "pitot_node";
        // publish_health(health_msg);
    }


    bool reconfigure(unitair::Reconfigure::Request& req,
                     unitair::Reconfigure::Response& res) {

        res.success = false;
        res.details = "No reconfigurable parameters";
        return true;
    }

};


} // namespace hardware
} // namespace unitair


using namespace unitair;
using namespace hardware;

int main(int argc, char** argv){

    std::string node_name = "airspeed";
    ros::init(argc, argv, node_name);

    std::string config_file_path;
    if(not ros::param::get("cfg_path", config_file_path)){
        ROS_ERROR("PITOT: Failed to load cfg_path parameter");
        return 1;
    }

    PitotNode node(config_file_path + "/hardware.yaml");
    node.configure(node_name, node._params.sample_rate);

    ros::spin();

    return 0;
}
