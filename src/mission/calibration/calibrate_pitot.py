#!/usr/bin/env python
"""
Computes the average raw pitot reading to use as the zero-point offset.
Make sure that the driver is running with an offset of zero and that
the tube is gently covered during the entire data collection.
Calibration is complete when the changes are very small.

"""
# Math
from __future__ import division
from math import sqrt

# Communication
import rospy
from unitair.msg import Pitot

##################################################

# Initialize ROS node
namespace = "mission/calibration"
node_name = "calibrate_pitot"
rospy.init_node(node_name)

# Create averager
n = 0
s = 0.0
s2 = 0.0
def callback(pitot):
    global n, s, s2
    n += 1
    s += pitot.pressure
    s2 += pitot.pressure**2
    a = s / n
    print(node_name+": current average is {}".format(a))
subscriber = rospy.Subscriber("hardware/pitot", Pitot, callback, queue_size=1)

# Run until manually terminated
print("")
print(node_name+": recording... (press 'Enter' to finish)\n")
try: raw_input()
except NameError: input()
subscriber.unregister()

# Display the results
print("=======================")
print("CALIBRATE PITOT RESULTS")
print("=======================")
print("Average differential pressure +- standard deviation:")
if n:
    a = s / n
    v = (s2 / n) - a**2
    print(a)
    print("+-")
    print(sqrt(v))
else:
    print("(no data)")
print("=======================\n")

##################################################
