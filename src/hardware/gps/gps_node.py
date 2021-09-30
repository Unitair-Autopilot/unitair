#!/usr/bin/env python
"""
Interfaces with a loosely-coupled GPS receiver.

"""
from __future__ import division
from numpy import isfinite
import rospy
import yaml
from unitair.srv import Reconfigure, ReconfigureRequest, ReconfigureResponse
from unitair.msg import GPS
from sensor_msgs.msg import NavSatFix

##################################################

class GPSNode(object):
    """
    Establishes a ROS node that relays a NavSatFix message as a Unitair message.

    """
    def __init__(self, node_name="gps", namespace="hardware"):
        # Initialize ROS node
        self.namespace = str(namespace)
        self.node_name = str(node_name)
        rospy.init_node(self.node_name)
        # Initialize connections
        rospy.Service(self.node_name+"/reconfigure", Reconfigure, self.configure)
        self.publisher = rospy.Publisher(self.namespace+"/"+self.node_name, GPS, queue_size=3, tcp_nodelay=True)
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
            self.topic = str(cfg["topic"])  # name of the NavSatFix topic
        except (IOError, KeyError, AssertionError) as what:
            return ReconfigureResponse(success=False, details="Invalid or incomplete cfg file '{0}' ({1})".format(cfg_path, what))
        # Initialize derived attributes
        if self.subscriber: self.subscriber.unregister()
        self.subscriber = rospy.Subscriber(self.topic, NavSatFix, self.relay, queue_size=3, tcp_nodelay=True)
        return ReconfigureResponse(success=True, details="Reloaded everything from '{0}'".format(cfg_path))

    def relay(self, navsatfix):
        self.publisher.publish(
            GPS(stamp=navsatfix.header.stamp,
                fixed=((navsatfix.status.status >= 0)
                       and isfinite(navsatfix.latitude)
                       and isfinite(navsatfix.longitude)
                       and isfinite(navsatfix.altitude)),
                latitude=navsatfix.latitude,
                longitude=navsatfix.longitude,
                altitude=navsatfix.altitude))

# Main working object
GPSNode()

##################################################
