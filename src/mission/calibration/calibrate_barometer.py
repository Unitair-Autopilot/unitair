#!/usr/bin/env python
"""
Averages barometer readings to get a nominal value for this position.
Run this while the platform is stationary at the launch site.
Make sure that navigation is ignoring the uncalibrated barometer.
Calibration is complete when the changes are very small.

"""
# Math
from __future__ import division
import numpy as np

# Communication
import rospy
from unitair.msg import State, Barometer

##################################################

# Initialize ROS node
namespace = "mission/calibration"
node_name = "calibrate_barometer"
rospy.init_node(node_name)

# Position averager
pos_num = 0
pos_sum = [0.0, 0.0, 0.0]
def rec_pos(state):
    global pos_num, pos_sum
    pos_num += 1
    pos_sum[0] += state.pos.x
    pos_sum[1] += state.pos.y
    pos_sum[2] += state.pos.z
pos_sub = rospy.Subscriber("navigation/state", State, rec_pos, queue_size=1)

# Pressure averager
bar_num = 0
bar_sum = 0.0
bar_sum2 = 0.0
def rec_bar(barometer):
    global bar_num, bar_sum, bar_sum2
    bar_num += 1
    bar_sum += barometer.pressure
    bar_sum2 += barometer.pressure**2
    bar_avg = bar_sum / bar_num
    print(node_name+": current average is {}".format(bar_avg))
bar_sub = rospy.Subscriber("hardware/barometer", Barometer, rec_bar, queue_size=1)

# Run until manually terminated
print("")
print(node_name+": recording... (press 'Enter' to finish)\n")
try: raw_input()
except NameError: input()
pos_sub.unregister()
bar_sub.unregister()

# Display the results
print("===========================")
print("CALIBRATE BAROMETER RESULTS")
print("===========================")
print("Average position:")
if pos_num:
    print(np.array2string(np.divide(pos_sum, pos_num), separator=", "))
else:
    print("(no data)")
print("=====")
print("Average pressure +- standard deviation:")
if bar_num:
    bar_avg = bar_sum / bar_num
    bar_var = (bar_sum2 / bar_num) - bar_avg**2
    print(bar_avg)
    print("+-")
    print(np.sqrt(bar_var))
else:
    print("(no data)")
print("===========================\n")

##################################################
