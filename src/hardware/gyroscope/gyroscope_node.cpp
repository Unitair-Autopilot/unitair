/** @file
    Gyroscope ROS node
*/

#include "hardware/common/hardware_node_base.hpp"
#include "hardware/gyroscope/FXAS21002.h"

#include <yaml-cpp/yaml.h>
#include "tools/tools.hpp"

// ROS messages
#include "unitair/Gyroscope.h"
#include "unitair/Health.h"


namespace unitair {
namespace hardware {


class GyroNode : public HardwareNode <unitair::Health> {

public:

    // publishers
    ros::Publisher _gyro_publisher;

    // device driver
    std::unique_ptr<FXAS21002> _device_driver;

    // parameters
    struct param_struct{
        double sample_rate;
        int gyro_sensitivity;
    } _params;


    GyroNode(std::string config_file_path){

        _gyro_publisher = _nh.advertise<unitair::Gyroscope>("hardware/gyroscope", 1);

        auto config = YAML::LoadFile(config_file_path);
        auto device_params = tools::yget("gyroscope", config);
        _params.sample_rate = tools::yget("sample_rate", device_params).as<double>();
        _params.gyro_sensitivity = tools::yget("gyro_sensitivity", device_params).as<int>();

        _device_driver = std::make_unique<FXAS21002>(tools::yget("i2c_device", device_params).as<std::string>());
        if(not _device_driver->configure(_params.gyro_sensitivity)){
            ROS_ERROR("GYRO: Failed to configure device");
        }

    };


    void main_loop(const ros::TimerEvent& event){

        auto data = _device_driver->read_data();
        auto now = ros::Time::now();

        unitair::Gyroscope gyro_msg;
        gyro_msg.stamp = now;
        gyro_msg.angvel.x = data.gyro.x();
        gyro_msg.angvel.y = data.gyro.y();
        gyro_msg.angvel.z = data.gyro.z();
        _gyro_publisher.publish(gyro_msg);
    }


    void health_loop(const ros::TimerEvent& event){

        //!TODO: add health info
        // auto now = ros::Time::now();
        // unitair::Health health_msg;
        // health_msg.stamp = now;
        // health_msg.node_name = "gyro_node";
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

    std::string node_name = "gyroscope";
    ros::init(argc, argv, node_name);

    std::string config_file_path;
    if(not ros::param::get("cfg_path", config_file_path)){
        ROS_ERROR("GYRO: Failed to load cfg_path parameter");
        return 1;
    }

    GyroNode node(config_file_path + "/hardware.yaml");
    node.configure(node_name, node._params.sample_rate);

    ros::spin();

    return 0;
}
