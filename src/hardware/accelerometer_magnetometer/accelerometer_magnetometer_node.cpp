/** @file
    Accelerometer & Magnetometer ROS node
*/

#include "hardware/common/hardware_node_base.hpp"
#include "hardware/accelerometer_magnetometer/FXOS8700.h"

#include <yaml-cpp/yaml.h>
#include "tools/tools.hpp"

// ROS messages
#include "unitair/Accelerometer.h"
#include "unitair/Magnetometer.h"
#include "unitair/Health.h"


namespace unitair {
namespace hardware {

class AccelMagNode : public HardwareNode <unitair::Health> {

public:

    // publishers
    ros::Publisher _accel_publisher;
    ros::Publisher _mag_publisher;

    // device driver
    std::unique_ptr<FXOS8700> _device_driver;

    // parameters
    struct param_struct{
        double sample_rate;
        int accel_sensitivity;
    } _params;


    AccelMagNode(std::string config_file_path){

        _accel_publisher = _nh.advertise<unitair::Accelerometer>("hardware/accelerometer", 1);
        _mag_publisher = _nh.advertise<unitair::Magnetometer>("hardware/magnetometer", 1);

        auto config = YAML::LoadFile(config_file_path);
        auto device_params = tools::yget("accelerometer_magnetometer", config);
        _params.sample_rate = tools::yget("sample_rate", device_params).as<double>();
        _params.accel_sensitivity = tools::yget("accel_sensitivity", device_params).as<int>();

        _device_driver = std::make_unique<FXOS8700>(tools::yget("i2c_device", device_params).as<std::string>());
        if(not _device_driver->configure(_params.accel_sensitivity)){
            ROS_ERROR("ACCEL-MAG: Failed to configure device");
        }
    };


    void main_loop(const ros::TimerEvent& event){

        auto data = _device_driver->read_data();
        auto now = ros::Time::now();

        unitair::Accelerometer accel_msg;
        accel_msg.stamp = now;
        accel_msg.accel.x = data.accel.x();
        accel_msg.accel.y = data.accel.y();
        accel_msg.accel.z = data.accel.z();
        _accel_publisher.publish(accel_msg);

        unitair::Magnetometer mag_msg;
        mag_msg.stamp = now;
        mag_msg.bfield.x = data.mag.x();
        mag_msg.bfield.y = data.mag.y();
        mag_msg.bfield.z = data.mag.z();
        _mag_publisher.publish(mag_msg);
    }


    void health_loop(const ros::TimerEvent& event){

        //!TODO: add health info
        // auto now = ros::Time::now();
        // unitair::Health health_msg;
        // health_msg.stamp = now;
        // health_msg.node_name = "accelerometer_magnetometer";
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

    std::string node_name = "accelerometer_magnetometer";
    ros::init(argc, argv, node_name);

    std::string config_file_path;
    if(not ros::param::get("cfg_path", config_file_path)){
        ROS_ERROR("ACCEL-MAG: Failed to load cfg_path parameter");
        return 1;
    }

    AccelMagNode node(config_file_path + "/hardware.yaml");
    node.configure(node_name, node._params.sample_rate);

    ros::spin();

    return 0;
}
