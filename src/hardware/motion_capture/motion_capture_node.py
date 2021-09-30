#!/usr/bin/env python
"""
Interfaces with an offboard motion-capture system.

"""
# Math
from __future__ import division
import numpy as np
import tf.transformations as trns

# Communication
import rospy
import yaml
from unitair.srv import Reconfigure, ReconfigureRequest, ReconfigureResponse
from unitair.msg import MotionCapture
from geometry_msgs.msg import PoseStamped

##################################################

class MotionCaptureNode(object):
    """
    Establishes a ROS node that relays 1720's VRPN pose message as a
    Unitair message and also rejects sudden orientation-flip glitches.

    """
    def __init__(self, node_name="motion_capture", namespace="hardware"):
        # Initialize ROS node
        self.namespace = str(namespace)
        self.node_name = str(node_name)
        rospy.init_node(self.node_name)
        # Initialize connections
        rospy.Service(self.node_name+"/reconfigure", Reconfigure, self.configure)
        self.publisher = rospy.Publisher(self.namespace+"/"+self.node_name, MotionCapture, queue_size=3, tcp_nodelay=True)
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
            assert cfg["type"] == "1720"  # (only type currently supported)
            self.name = str(cfg["name"])  # name assigned to this body
            self.angvel_cutoff = float(cfg["angvel_cutoff"])  # orientation changes faster than this are deemed glitches and rejected
            self.rejection_duration = float(cfg["rejection_duration"])  # amount of time that contiguous glitches are rejected before accepting the new orientation
            self.warn = bool(cfg["warn"])  # whether or not to display a warning when glitches are rejected
        except (ValueError, IOError, KeyError, AssertionError) as what:
            return ReconfigureResponse(success=False, details="Invalid or incomplete cfg file '{0}' ({1})".format(cfg_path, what))
        # Initialize derived attributes
        self.last = None  # the last received valid raw pose measurement
        self.timer = 0.0  # the amount of time it has been since the first of a contiguous stream of glitches was rejected, [s]
        callback = self.check if (self.rejection_duration > 0.0) else self.accept
        if self.subscriber: self.subscriber.unregister()
        self.subscriber = rospy.Subscriber("/vrpn_client_node/{0}/pose".format(self.name), PoseStamped, callback, queue_size=3, tcp_nodelay=True)
        return ReconfigureResponse(success=True, details="Reloaded everything from '{0}'".format(cfg_path))

    def check(self, msg):
        """
        Relays the received PoseStamped message as a MotionCapture message
        if it isn't filtered as an orientation-flip glitch, determined by
        evaluating the angular velocity magnitude.

        """
        # Initialize or reset
        if (self.last is None) or (self.timer > self.rejection_duration): return self.accept(msg)
        # Compute time since last
        dt = (msg.header.stamp - self.last.header.stamp).to_sec()
        if dt <= 0.0: return
        self.last.header = msg.header
        # Compute angular delta
        last_ori = np.array([self.last.pose.orientation.__getattribute__(a) for a in "xyzw"], float)
        msg_ori = np.array([msg.pose.orientation.__getattribute__(a) for a in "xyzw"], float)
        dq = 2.0*np.arccos(np.clip(trns.quaternion_multiply(trns.quaternion_inverse(last_ori), msg_ori)[-1], -1.0, 1.0))
        # Ignore new measurement if angular velocity threshold is exceeded
        if abs(dq/dt) > self.angvel_cutoff:
            if self.warn and (self.timer == 0.0):
                rospy.logwarn(self.node_name+": orientation-flip glitch detected")
            self.timer += dt
        else:
            if self.warn and (self.timer > 0.0):
                rospy.loginfo(self.node_name+": orientation-flip glitch resolved")
            self.accept(msg)

    def accept(self, msg):
        """
        Assumes the given PoseStamped message is a valid measurement
        (thus updating 'last' and resetting 'timer') and publishes it
        as a MotionCapture message.

        """
        self.last = msg
        self.timer = 0.0
        self.publisher.publish(MotionCapture(stamp=self.last.header.stamp, pos=self.last.pose.position, ori=self.last.pose.orientation))

# Main working object
MotionCaptureNode()

##################################################
