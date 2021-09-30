#!/usr/bin/env python
"""
Manages the system mode, health aggregation, command multiplexing, and failsafes.
??? (unfinished)

"""
# Math
from __future__ import division
import numpy as np
from tf import transformations as trns
assert np.allclose(trns.quaternion_from_euler(0, 0, 0), [0, 0, 0, 1])  # check quaternion convention

# Communication
import rospy
from unitair.msg import Mode, Health
from unitair.msg import Joystick, ThrustCmd, AttitudeCmd
from unitair.msg import Battery, State, GPS
from geometry_msgs.msg import Vector3, Quaternion

# Configuration
import yaml
from unitair.srv import Reconfigure, ReconfigureRequest, ReconfigureResponse
from unitair.srv import SetPath, SetPathRequest, SetPathResponse

##################################################

class ManagerNode(object):
    def __init__(self, node_name="manager", namespace="mission"):
        # Initialize ROS node
        self.namespace = str(namespace)
        self.node_name = str(node_name)
        rospy.init_node(self.node_name)
        # Initialize parameters
        if not self.configure().success:
            raise IOError(self.node_name+": failed to load initial cfg (is the cfg_path ROS-param set?)")
        # Initialize connections
        self.thrustcmd_pub = rospy.Publisher("mission/thrust", ThrustCmd, queue_size=3, tcp_nodelay=True)
        self.attitudecmd_pub = rospy.Publisher("mission/attitude", AttitudeCmd, queue_size=3, tcp_nodelay=True)
        self.mode_pub = rospy.Publisher("mission/mode", Mode, queue_size=3, tcp_nodelay=True)
        rospy.Subscriber("hardware/joystick", Joystick, self.joystick_cbk, queue_size=3, tcp_nodelay=True)
        rospy.Subscriber("guidance/thrust", ThrustCmd, self.thrustcmd_cbk, queue_size=3, tcp_nodelay=True)
        rospy.Subscriber("guidance/attitude", AttitudeCmd, self.attitudecmd_cbk, queue_size=3, tcp_nodelay=True)
        rospy.Subscriber("navigation/state", State, self.state_cbk, queue_size=3, tcp_nodelay=True)
        rospy.Service(self.node_name+"/reconfigure", Reconfigure, self.configure)
        rospy.Service(self.namespace+"/setpath", SetPath, self.setpath)
        rospy.Timer(rospy.Duration(0.1), self.failcheck)
        rospy.Timer(rospy.Duration(0.5), self.modecheck)
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
            self.failsafe_delay = rospy.Duration(cfg["failsafe_delay"])
            self.throttle_style = str(cfg["throttle_style"])
            self.attitude_style = str(cfg["attitude_style"])
            self.thrust_range = float(cfg["thrust_range"])
            self.roll_range = np.deg2rad(float(cfg["roll_range"]))
            self.pitch_range = np.deg2rad(float(cfg["pitch_range"]))
            self.roll_rate = np.deg2rad(float(cfg["roll_rate"]))
            self.pitch_rate = np.deg2rad(float(cfg["pitch_rate"]))
            self.yaw_rate = np.deg2rad(float(cfg["yaw_rate"]))
            # Validation
            assert self.failsafe_delay.to_sec() > 0
            assert self.throttle_style in ("stick", "trigger")
            assert self.attitude_style in ("conventional", "copter")
        except (ValueError, IOError, KeyError, AssertionError) as what:
            return ReconfigureResponse(success=False, details="Invalid or incomplete cfg file '{0}' ({1})".format(cfg_path, what))
        # Initialize derived attributes
        self.autonomous = False; rospy.loginfo(self.node_name + ": toggle autonomous mode with 'switch_east'")
        self.stabilized = False; rospy.loginfo(self.node_name + ": toggle stabilization mode with 'switch_west'")
        self.state = None
        self.failsafe_timer = rospy.Duration(0)
        self.failsafe_time0 = rospy.get_rostime()
        self.failsafe_engaged = False
        self.guidance_setpath = rospy.ServiceProxy("/guidance/setpath", SetPath)
        self.path = None
        self.copter_ori_offset = trns.quaternion_from_euler(0, -np.pi/2, 0)
        return ReconfigureResponse(success=True, details="Reloaded everything from '{0}'".format(cfg_path))

    def setpath(self, req):
        res = self.guidance_setpath(req)
        if res.issue == SetPathResponse.NONE:
            self.path = req
        else:
            rospy.logwarn(self.node_name + f": path request had issue '{res.issue}'")
        return res

    def joystick_cbk(self, msg):
        # Failsafe logic
        if not msg.error:
            self.failsafe_timer = rospy.Duration(0)
            self.failsafe_time0 = rospy.get_rostime()
        if self.failsafe_timer > self.failsafe_delay:
            return
        elif self.failsafe_engaged:
            self.failsafe_engaged = False
            rospy.loginfo(self.node_name + ": joystick regained, autonomy override disengaged")
        # Mode logic
        if msg.switch_east and not self.autonomous:
            rospy.loginfo(self.node_name + ": pilot switched to autonomous mode")
            if self.path:
                self.guidance_setpath(self.path)
        if self.autonomous and not msg.switch_east:
            rospy.loginfo(self.node_name + ": pilot switched to manual mode")
        if msg.switch_west and not self.stabilized:
            rospy.loginfo(self.node_name + ": activated attitude stabilization for manual mode")
        if self.stabilized and not msg.switch_west:
            rospy.loginfo(self.node_name + ": deactivated attitude stabilization for manual mode (angvel only)")
        self.autonomous = msg.switch_east
        self.stabilized = msg.switch_west
        if self.autonomous:
            return
        # Thrust
        if self.throttle_style == "stick":
            throttle = 0.5*(msg.left_stick_north + 1.0)
        elif self.throttle_style == "trigger":
            throttle = msg.right_trigger
        thrust = ThrustCmd(stamp=msg.stamp, thrust=throttle*self.thrust_range)
        self.thrustcmd_pub.publish(thrust)
        # Orientation
        if self.stabilized:
            if self.attitude_style == "conventional":
                if self.state is None:
                    rospy.logwarn_once(self.node_name + ": waiting on yaw estimate for conventional attitude control")
                    return
                yaw = trns.euler_from_quaternion([getattr(self.state.ori, c) for c in "xyzw"])[2]
                ori = trns.quaternion_from_euler(msg.right_stick_east*self.roll_range, msg.right_stick_north*self.pitch_range, yaw)
            elif self.attitude_style == "copter":
                cmd = trns.quaternion_from_euler(0, msg.right_stick_north*self.pitch_range, -msg.right_stick_east*self.roll_range)
                ori = trns.quaternion_multiply(self.copter_ori_offset, cmd)
            attitude = AttitudeCmd(stamp=msg.stamp, ori=Quaternion(x=ori[0], y=ori[1], z=ori[2], w=ori[3]))
        # Angular velocity
        else:
            if self.attitude_style == "conventional":
                attitude = AttitudeCmd(stamp=msg.stamp, angvel=Vector3(
                    x=msg.right_stick_east*self.roll_rate,
                    y=msg.right_stick_north*self.pitch_rate,
                    z=-msg.left_stick_east*self.yaw_rate
                ))
            elif self.attitude_style == "copter":
                attitude = AttitudeCmd(stamp=msg.stamp, angvel=Vector3(
                    x=-msg.left_stick_east*self.yaw_rate,
                    y=msg.right_stick_north*self.pitch_rate,
                    z=-msg.right_stick_east*self.roll_rate,
                ))
        self.attitudecmd_pub.publish(attitude)

    def thrustcmd_cbk(self, msg):
        if not np.isfinite(msg.thrust):
            rospy.logwarn_once(self.node_name + ": guidance thrust is NaN!")
        if self.autonomous:
            self.thrustcmd_pub.publish(msg)

    def attitudecmd_cbk(self, msg):
        if not (np.all([np.isfinite(getattr(msg.ori, c)) for c in "xyzw"]) and np.all([np.isfinite(getattr(msg.angvel, c)) for c in "xyz"])):
            rospy.logwarn_once(self.node_name + ": guidance attitude is NaN!")
        if self.autonomous:
            self.attitudecmd_pub.publish(msg)

    def state_cbk(self, msg):
        self.state = msg

    def failcheck(self, _):
        self.failsafe_timer = rospy.get_rostime() - self.failsafe_time0
        if self.failsafe_timer > self.failsafe_delay:
            if not self.failsafe_engaged:
                if self.path:
                    self.guidance_setpath(self.path)
                self.failsafe_engaged = True
                rospy.logwarn(self.node_name + ": lost joystick, autonomy override engaged")
            self.autonomous = True

    def modecheck(self, _):
        self.mode_pub.publish(
            Mode(
                stamp=rospy.get_rostime(),
                mode=Mode.TRACKING if self.path else Mode.STANDBY,
                details="",
                autonomous=self.autonomous
            )
        )

# Main working object
ManagerNode()

##################################################
