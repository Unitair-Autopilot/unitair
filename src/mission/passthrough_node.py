#!/usr/bin/env python
"""
Raw link between the joystick and the actuators.
This is intended for true manual flight that bypasses the usual GNC.
Usurps the role of the Allocator.

"""
from __future__ import division
from numpy import clip
import rospy
import yaml
from unitair.srv import Reconfigure, ReconfigureRequest, ReconfigureResponse
from unitair.msg import Joystick, Actuation

##################################################

class PassthroughNode(object):
    """
    Establishes a ROS node that converts Joystick messages into Actuation messages.

    """
    def __init__(self, node_name="passthrough", namespace="mission"):
        # Initialize ROS node
        self.namespace = str(namespace)
        self.node_name = str(node_name)
        rospy.init_node(self.node_name)
        # Initialize parameters
        if not self.configure().success:
            raise IOError(self.node_name+": failed to load initial cfg (is the cfg_path ROS-param set?)")
        # Initialize connections
        self.publisher = rospy.Publisher("control/actuation", Actuation, queue_size=1, tcp_nodelay=True)
        rospy.Service(self.node_name+"/reconfigure", Reconfigure, self.configure)
        rospy.Subscriber("hardware/joystick", Joystick, self.relay, queue_size=3, tcp_nodelay=True)
        rospy.Timer(rospy.Duration(0.2), self.failcheck)
        # Run until ROS is killed
        rospy.loginfo(self.node_name + ": engaged direct joystick -> actuators")
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
            self.channels = list(cfg["channels"])  # input names and shift/scale factors for each channel
            self.failsafe = list(cfg["failsafe"])  # actuation efforts to use if joystick connection is lost
            self.failtime = rospy.Duration(cfg["failtime"])  # how long to wait on lost connection before doing failsafe
            # Validation
            assert len(self.channels) == len(self.failsafe)
            assert self.failtime.to_sec() > 0
        except (ValueError, IOError, KeyError, AssertionError) as what:
            return ReconfigureResponse(success=False, details="Invalid or incomplete cfg file '{0}' ({1})".format(cfg_path, what))
        # Initialize derived attributes
        self.failsafe_timer = rospy.Duration(0)
        self.failsafe_time0 = rospy.get_rostime()
        self.failsafe_engaged = False
        return ReconfigureResponse(success=True, details="Reloaded everything from '{0}'".format(cfg_path))

    def relay(self, joystick):
        if not joystick.error:
            self.failsafe_timer = rospy.Duration(0)
            self.failsafe_time0 = rospy.get_rostime()
        if self.failsafe_timer < self.failtime:
            actuation = Actuation()
            actuation.stamp = joystick.stamp
            if self.failsafe_engaged:
                self.failsafe_engaged = False
                rospy.loginfo(self.node_name + ": joystick regained, failsafe disengaged")
            for channel in self.channels:
                value = clip(channel["scale"] * (getattr(joystick, channel["input"]) + channel["shift"]), -1.0, 1.0)
                actuation.efforts.append(value)
            self.publisher.publish(actuation)

    def failcheck(self, _):
        self.failsafe_timer = rospy.get_rostime() - self.failsafe_time0
        if self.failsafe_timer > self.failtime:
            if not self.failsafe_engaged:
                self.failsafe_engaged = True
                rospy.logwarn(self.node_name + ": lost joystick, failsafe engaged")
            self.publisher.publish(Actuation(stamp=rospy.get_rostime(), efforts=self.failsafe))

# Main working object
PassthroughNode()

##################################################
