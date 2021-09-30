#!/usr/bin/env python
"""
Publishes a ping from offboard to onboard for communications monitoring.

"""
import rospy
import yaml
from unitair.srv import Reconfigure, ReconfigureRequest, ReconfigureResponse
from unitair.msg import Health

##################################################

# Establish node
namespace = "mission"
node_name = "heartbeat"
rospy.init_node(node_name)
cfg = None
rate = None

# Configure parameters
def configure(req=ReconfigureRequest()):
    global cfg, rate
    try:
        if req.cfg_path: cfg_path = req.cfg_path
        else: cfg_path = rospy.get_param("cfg_path", default=".")
        with open(cfg_path+"/"+namespace+".yaml") as cfg_file: cfg = yaml.safe_load(cfg_file)[node_name]
        rate = float(cfg["rate"])
    except (ValueError, IOError, KeyError, AssertionError) as what:
        return ReconfigureResponse(success=False, details="Invalid or incomplete cfg file '{0}' ({1})".format(cfg_path, what))
    return ReconfigureResponse(success=True, details="Reloaded everything from '{0}'".format(cfg_path))
reconfigure_srv = rospy.Service(node_name+"/reconfigure", Reconfigure, configure)
if not configure().success: raise IOError(node_name+": failed to load initial cfg (is the cfg_path ROS-param set?)")

##################################################

publisher = rospy.Publisher(namespace+"/"+node_name, Health, queue_size=1, tcp_nodelay=True)

def heartbeat(_):
    publisher.publish(
        Health(node_name=node_name,
               stamp=rospy.get_rostime(),
               requests=[Health.PASS],
               labels=["This heartbeat should be heard at {} Hz.".format(rate)]))

main = rospy.Timer(rospy.Duration(1.0/rate), heartbeat)
rospy.spin()

##################################################
