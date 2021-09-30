#!/usr/bin/env python
"""
Publishes the state of a gamepad-style joystick.

"""
# Math
from __future__ import division
import math

# Communication
import signal
import sys
import rospy

# Configuration
import yaml
from unitair.srv import Reconfigure, ReconfigureRequest, ReconfigureResponse

# Main class
from joystick import Joystick, JoystickMsg

##################################################

# Establish node
namespace = "hardware"
node_name = "joystick"
rospy.init_node(node_name)
rate = None
joystick = None

# Connect to hardware
def connect(checkper):
    global joystick
    joystick = None
    while not joystick:
        if rospy.is_shutdown():
            quit()
        try:
            last = joystick
            joystick = Joystick()
            joystick.start_joystick_thread()
            if last is False:
                rospy.loginfo(node_name+": connected to joystick")
        except IOError:
            if joystick is None:
                joystick = False
                rospy.logwarn(node_name+": could not find/connect, will retry every {} seconds".format(checkper))
            rospy.sleep(checkper)
connect(checkper=3)

##################################################

# Prepare reconfiguration service
def configure(req=ReconfigureRequest()):
    global rate, joystick
    try:
        # Load cfg
        if req.cfg_path: cfg_path = req.cfg_path
        else: cfg_path = rospy.get_param("cfg_path", default=".")
        with open(cfg_path+"/"+namespace+".yaml") as cfg_file: cfg = yaml.safe_load(cfg_file)[node_name]
        # Assign values
        rate = rospy.Rate(cfg["rate"])
        joystick.set_stick_deadband(cfg["stick_deadband"])
        joystick.set_trigger_deadband(cfg["trigger_deadband"])
    except (IOError, KeyError, AssertionError) as what:
        # Report failure
        return ReconfigureResponse(success=False, details="Invalid or incomplete cfg file '{0}' ({1})".format(cfg_path, what))
    # Report success
    return ReconfigureResponse(success=True, details="Reloaded rate and deadbands from '{0}'".format(cfg_path))
reconfigure_srv = rospy.Service(node_name+"/reconfigure", Reconfigure, configure)

# Initial configuration
if not configure().success: raise IOError(node_name+": failed to load initial cfg (is the cfg_path ROS-param set?)")

##################################################

# Prepare publisher
joystick_pub = rospy.Publisher(namespace+"/"+node_name, JoystickMsg, queue_size=1, tcp_nodelay=True)

# Handler for ctrl+c exits since Joystick spools a new thread
signal.signal(signal.SIGINT, lambda s, f: sys.exit(0))

##################################################

# Main loop
while not rospy.is_shutdown():
    try:
        msg = joystick.get_state()
        msg.stamp = rospy.get_rostime()
        msg.error = False
        joystick_pub.publish(msg)
        rate.sleep()
    except (IOError, AssertionError):
        connect(checkper=1)
joystick.stop_joystick_thread()

##################################################
