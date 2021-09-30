#!/usr/bin/env python
"""
Relays various Unitair messages as visualizable messages for RViz and RQt.
This is used by a human with an offboard computer.

"""
# Math
from __future__ import division
import numpy as np

# Communication
import rospy
import tf

# Configuration
import yaml
from unitair.srv import Reconfigure, ReconfigureRequest, ReconfigureResponse

# Message types
from unitair.msg import State, ThrustCmd, TorqueCmd, Plan
from geometry_msgs.msg import Point, Vector3, WrenchStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Path

##################################################

# Establish node
namespace = "mission"
node_name = "visualizer"
rospy.init_node(node_name)

# Initialize parameters
rate = None  # frequency at which visualization messages are published
body_shape = None  # dimensions of a prism in body-coordinates to represent the body
plan_count = None  # number of poses to display along the guidance plan

# Configure parameters
def configure(req=ReconfigureRequest()):
    global rate, body_shape, plan_count
    try:
        if req.cfg_path: cfg_path = req.cfg_path
        else: cfg_path = rospy.get_param("cfg_path", default=".")
        with open(cfg_path+"/"+namespace+".yaml") as cfg_file: cfg = yaml.safe_load(cfg_file)[node_name]
        rate = rospy.Rate(cfg["rate"])
        body_shape = np.array(cfg["body_shape"], float).reshape(3,)
        plan_count = int(cfg["plan_count"])
    except (ValueError, IOError, KeyError, AssertionError) as what:
        return ReconfigureResponse(success=False, details="Invalid or incomplete cfg file '{0}' ({1})".format(cfg_path, what))
    return ReconfigureResponse(success=True, details="Reloaded everything from '{0}'".format(cfg_path))
reconfigure_srv = rospy.Service(node_name+"/reconfigure", Reconfigure, configure)
if not configure().success: raise IOError(node_name+": failed to load initial cfg (is the cfg_path ROS-param set?)")

##################################################

# Prepare publishers
tfbr                 = tf.TransformBroadcaster()
scenery_pub          = rospy.Publisher(namespace+"/"+node_name+"/scenery",             PointCloud, queue_size=1, tcp_nodelay=True)
truth_state_pub      = rospy.Publisher(namespace+"/"+node_name+"/truth/state",        MarkerArray, queue_size=1, tcp_nodelay=True)
estimated_state_pub  = rospy.Publisher(namespace+"/"+node_name+"/estimated/state",    MarkerArray, queue_size=1, tcp_nodelay=True)
commanded_wrench_pub = rospy.Publisher(namespace+"/"+node_name+"/commanded/wrench", WrenchStamped, queue_size=1, tcp_nodelay=True)
planned_poses_pub    = rospy.Publisher(namespace+"/"+node_name+"/planned/poses",             Path, queue_size=1, tcp_nodelay=True)
desired_path_pub     = rospy.Publisher(namespace+"/"+node_name+"/desired/path",              Path, queue_size=1, tcp_nodelay=True)
desired_proxy_pub    = rospy.Publisher(namespace+"/"+node_name+"/desired/proxy",      MarkerArray, queue_size=1, tcp_nodelay=True)

# Prepare subscribers
received = {}
rospy.Subscriber("simulation/truth/state",     State, lambda msg: received.__setitem__("truth_state",     msg), queue_size=1)
rospy.Subscriber(      "navigation/state",     State, lambda msg: received.__setitem__("estimated_state", msg), queue_size=1)
rospy.Subscriber(       "guidance/thrust", ThrustCmd, lambda msg: received.__setitem__("thrust_command",  msg), queue_size=1)
rospy.Subscriber(        "control/torque", TorqueCmd, lambda msg: received.__setitem__("torque_command",  msg), queue_size=1)
rospy.Subscriber(         "guidance/plan",      Plan, lambda msg: received.__setitem__("plan",            msg), queue_size=1)

##################################################

def publish_frames():
    # Follow sim if it is running, else follow nav
    if "truth_state" in received:
        world = "simulation/truth/enu"
        body = "simulation/truth/body"
    else:
        world = "navigation/world"
        body = "navigation/body"
    # Relay frames for what we're following
    stamp = rospy.get_rostime()
    tfbr.sendTransform((0, 0, 0), (0, 0, 0, 1), stamp, node_name+"/world", world)
    tfbr.sendTransform((0, 0, 0), (0, 0, 0, 1), stamp, node_name+"/body", body)

#########################

def publish_scenery():
    # If we haven't already built the (unchanging) scenery message, do it
    if not hasattr(publish_scenery, "msg"):
        msg = PointCloud(points=[])
        msg.header.frame_id = node_name+"/world"
        grid = np.mgrid[-1000:1000:100, -1000:1000:100]
        for i in range(grid[0].shape[0]):
            for j in range(grid[0].shape[1]):
                x = grid[0, i, j]
                y = grid[1, i, j]
                msg.points.append(Point(x, y, -100))
                msg.points.append(Point(x, y, 2000))
        setattr(publish_scenery, "msg", msg)
    # Update the time and publish it
    publish_scenery.msg.header.stamp = rospy.get_rostime()
    scenery_pub.publish(publish_scenery.msg)

#########################

def publish_truth_state():
    # Get the latest truth state data
    try: state = received["truth_state"]
    except KeyError: return
    # Build a prism marker to represent the body state
    msg = Marker(id=0)
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "simulation/truth/enu"
    msg.type = Marker.CUBE
    msg.pose.position, msg.pose.orientation = state.pos, state.ori
    msg.scale.x, msg.scale.y, msg.scale.z = body_shape
    msg.color.r, msg.color.g, msg.color.b, msg.color.a = [1, 1, 0, 0.5]
    # Publish all markers
    truth_state_pub.publish(MarkerArray([msg]))

#########################

def publish_estimated_state():
    # Get the latest estimated state data
    try: state = received["estimated_state"]
    except KeyError: return
    # Build a prism marker to represent the body state
    msg = Marker(id=0)
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "navigation/world"
    msg.type = Marker.CUBE
    msg.pose.position, msg.pose.orientation = state.pos, state.ori
    msg.scale.x, msg.scale.y, msg.scale.z = body_shape
    msg.color.r, msg.color.g, msg.color.b, msg.color.a = [0, 1, 1, 1]
    # Publish all markers
    estimated_state_pub.publish(MarkerArray([msg]))

#########################

def publish_commanded_wrench():
    # Construct a wrench that combines the latest thrust and torque commands
    msg = WrenchStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = node_name+"/body"
    try:
        msg.wrench.force.x = received["thrust_command"].thrust
        msg.wrench.torque = received["torque_command"].torque
    except KeyError:
        pass
    # Publish the wrench
    commanded_wrench_pub.publish(msg)

#########################

def publish_guidance_plan():
    # Get the latest plan data
    try: plan = received["plan"]
    except KeyError: return
    # Only relay new data
    if (plan.stamp - publish_guidance_plan.last_stamp) < rate.sleep_dur: return
    publish_guidance_plan.last_stamp = plan.stamp
    # Visualize the expected poses (downsampled)
    msg = Path()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "navigation/world"
    for t in np.linspace(0, len(plan.time)-1, num=plan_count, dtype=int):
        m = PoseStamped()
        m.pose.position = plan.pos[t]
        m.pose.orientation = plan.ori[t]
        msg.poses.append(m)
    planned_poses_pub.publish(msg)
    # Visualize the desired path
    msg = Path()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "navigation/world"
    for t in range(len(plan.path)):
        m = PoseStamped()
        m.pose.position = plan.path[t]
        msg.poses.append(m)
    desired_path_pub.publish(msg)
    # Visualize the "carrot"
    msg = Marker(id=0)
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "navigation/world"
    msg.type = Marker.SPHERE
    msg.pose.position = plan.proxy
    msg.pose.orientation.w = 1
    msg.scale.x, msg.scale.y, msg.scale.z = [body_shape[1]/2]*3
    msg.color.r, msg.color.g, msg.color.b, msg.color.a = [0, 1, 1, 1]
    desired_proxy_pub.publish(MarkerArray([msg]))

publish_guidance_plan.last_stamp = rospy.get_rostime()  # for checking freshness

##################################################

# Main loop
while not rospy.is_shutdown():
    rate.sleep()
    publish_frames()
    publish_scenery()
    publish_truth_state()
    publish_estimated_state()
    publish_commanded_wrench()
    publish_guidance_plan()

##################################################
