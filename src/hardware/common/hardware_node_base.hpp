/** @file
    Base class for hardware ROS nodes
*/

#pragma once

#include <ros/ros.h>

#include "unitair/Reconfigure.h"


namespace unitair {
namespace hardware {

template <typename health_msg_t>
class HardwareNode {

    private:

        ros::Publisher _health_publisher;
        ros::Timer _main_loop_timer;
        ros::Timer _health_timer;
        ros::ServiceServer _param_service;

    public:

        ros::NodeHandle _nh;
        std::string _config_file_path;

        //! Destructor
        virtual ~HardwareNode() {}


        //! creates health publisher,
        //! kicks off main loop , 1hz health loop, and parameter update service
        /*!
            \param[in] node_name name of the ROS node this instance is handling
            \param[in] loop_rate rate of main loop, <0 disables main loop (Hz)
            \param[in] queue_size ros publisher queue size (default = 1)
        */
        virtual void configure(std::string const& node_name,
                               double loop_rate,
                               double queue_size = 1){

            _health_publisher = _nh.advertise<health_msg_t>("health", queue_size);

            if(loop_rate > 0){
                _main_loop_timer = _nh.createTimer(ros::Duration(1.0/loop_rate),
                                                   &HardwareNode::main_loop,
                                                   this);
            }
            _health_timer = _nh.createTimer(ros::Duration(1.0),
                                            &HardwareNode::health_loop,
                                            this);

            _param_service = _nh.advertiseService(node_name + "/reconfigure",
                                                  &HardwareNode::reconfigure,
                                                  this);
        };


        //! Publish health and status
        /*!
            \param[in] msg ros message to publish
        */
        virtual void publish_health(health_msg_t msg){

            _health_publisher.publish(msg);
        };


        //! Main loop
        /*!
            implementation specific
        */
        virtual void main_loop(const ros::TimerEvent& event) = 0;


        //! Health loop
        /*!
            implementation specific
        */
        virtual void health_loop(const ros::TimerEvent& event) = 0;


        //! Update all parameters
        /*!
            implementation specific
        */
        virtual bool reconfigure(unitair::Reconfigure::Request& req,
                                 unitair::Reconfigure::Response& res) = 0;

};


} // namespace hardware
} // namespace unitair
