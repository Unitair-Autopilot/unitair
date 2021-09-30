#!/usr/bin/env python
"""
Interfaces with an SBus radio controller.

"""
from __future__ import division
import numpy as np
import rospy
import yaml
from unitair.srv import Reconfigure, ReconfigureRequest, ReconfigureResponse
from unitair.msg import Joystick
from sbus_serial.msg import Sbus  # https://github.com/jenswilly/sbus_serial

##################################################

class RCNode(object):
    """
    Establishes a ROS node that relays the RC Sbus to the Unitair joystick topic.

    """
    def __init__(self, node_name="joystick", namespace="hardware"):
        # Initialize ROS node
        self.namespace = str(namespace)
        self.node_name = str(node_name)
        rospy.init_node(self.node_name)
        # Initialize connections
        rospy.Service(self.node_name+"/reconfigure", Reconfigure, self.configure)
        self.publisher = rospy.Publisher(self.namespace+"/"+self.node_name, Joystick, queue_size=1, tcp_nodelay=True)
        self.subscriber = None
        # Initialize parameters
        if not self.configure().success:
            raise IOError(self.node_name+": failed to load initial cfg (is the cfg_path ROS-param set?)")
        # Run until ROS is killed
        rospy.spin()

    def configure(self, req=ReconfigureRequest()):
        """
        Loads the requested configuration file and constructs self.
        Queries the 'cfg_path' ROS-param by default.

        """
        try:
            # Load configuration file
            if req.cfg_path: cfg_path = req.cfg_path
            else: cfg_path = rospy.get_param("cfg_path", default=".")
            with open(cfg_path+"/"+self.namespace+".yaml") as cfg_file: cfg = yaml.safe_load(cfg_file)[self.node_name]
            # Unpack given attributes
            self.topic = str(cfg["topic"])  # name of the Sbus topic
            self.channels = list(cfg["channels"])  # input names and [min, max] ranges for each channel
            # Preprocess
            test = Joystick()
            for channel in self.channels:
                assert hasattr(test, channel["input"])
                channel["sum"] = float(np.sum(channel["range"]))
                channel["diff"] = float(np.diff(channel["range"]))
                channel["range"].sort()
        except (IOError, KeyError, AssertionError) as what:
            return ReconfigureResponse(success=False, details="Invalid or incomplete cfg file '{0}' ({1})".format(cfg_path, what))
        # Initialize derived attributes
        if self.subscriber: self.subscriber.unregister()
        self.subscriber = rospy.Subscriber(self.topic, Sbus, self.relay, queue_size=3, tcp_nodelay=True)
        return ReconfigureResponse(success=True, details="Reloaded everything from '{0}'".format(cfg_path))

    def relay(self, sbus):
        joystick = Joystick()
        joystick.stamp = sbus.header.stamp
        joystick.error = sbus.frame_lost
        for (i, channel) in enumerate(self.channels):
            # Validate
            if not (channel["range"][0] <= sbus.rawChannels[i] <= channel["range"][1]):
                rospy.logwarn(self.node_name + ": clipping out-of-range RC value on channel {0} ({1})".format(i, channel["input"]))
            # Convert
            value = np.clip((2.0*sbus.rawChannels[i] - channel["sum"]) / channel["diff"], -1.0, 1.0)
            if type(getattr(joystick, channel["input"])) is bool:
                value = (value >= 0.0)
            setattr(joystick, channel["input"], value)
        self.publisher.publish(joystick)

# Main working object
RCNode()

##################################################
