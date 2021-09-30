#!/usr/bin/env python
"""
Runs a magnetometer hard/soft calibration.
View the calibration dataset in RViz and manually rotate the
platform until the points significantly cover an ellipsoid.
Then press 'Enter' to compute and view the results.
The calibrated points should form a unit-sphere at the origin.

Theory: https://teslabs.com/articles/magnetometer-calibration/

"""
# Math
from __future__ import division
import numpy as np
from scipy.linalg import sqrtm
from scipy.optimize import minimize

# Communication
import rospy
from unitair.msg import Magnetometer
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

# Configuration
import yaml

##################################################

# Initialize ROS node
namespace = "mission/calibration"
node_name = "calibrate_magnetometer"
rospy.init_node(node_name)

# Load configuration
try:
    cfg_path = rospy.get_param("cfg_path", default=".")
    with open(cfg_path+"/navigation.yaml") as cfg_file:
        cfg = yaml.safe_load(cfg_file)["estimator"]["informants"]["magnetometer"]
        ref_norm = np.linalg.norm(np.reshape(cfg["field"], (3,)))
        diff_tol = 0.05  # [rad]
except IOError:
    raise IOError(node_name+": failed to load cfg (is the cfg_path ROS-param set?)")
except KeyError as exception:
    raise KeyError(node_name+": invalid cfg missing key '"+exception.args[0]+"'")
except ValueError as exception:
    raise ValueError(node_name+": invalid cfg caused error '"+exception.args[0]+"'")

##################################################

# Initialize records
dataset = []
visualization = PointCloud()
visualization.header.frame_id = "visualizer/world"

# Callback for filling the records
def record_magnetometer(msg):
    # Unpack message
    measurement = np.array([msg.bfield.x, msg.bfield.y, msg.bfield.z], float)
    # Ignore invalid readings
    norm = np.linalg.norm(measurement)
    if np.isclose(norm, 0.0) or not np.all(np.isfinite(measurement)):
        return
    # Ignore new readings that aren't different enough from previous
    if dataset:
        proj = np.dot(measurement, dataset[-1]) / (norm*np.linalg.norm(dataset[-1]))
        diff = np.arccos(np.clip(proj, -1.0, 1.0))
        if diff < diff_tol:
            return
    # Store the measurement
    dataset.append(measurement)
    visualization.points.append(Point32(*(measurement / ref_norm)))
    # Provide the visualization
    visualization.header.stamp = msg.stamp
    visualization_pub.publish(visualization)

# Initialize connections
visualization_pub = rospy.Publisher(namespace+"/magnetometer_dataset", PointCloud, queue_size=1)
magnetometer_sub = rospy.Subscriber("hardware/magnetometer", Magnetometer, record_magnetometer, queue_size=10)

# Run until manually terminated
print("")
print(node_name+": recording... (press 'Enter' to finish)")
try: raw_input()  # (Python 2)
except NameError: input()  # (Python 3)
magnetometer_sub.unregister()

# Finalize dataset
dataset = np.array(dataset, float)
if len(dataset) == 0:
    print(node_name+": did not record any data!\n")
    quit()

##################################################

# Fits an ellipsoid to the dataset
def initial_fit(X):
    """
    See: https://ieeexplore.ieee.org/document/1290055 (Section 3)

    """
    D = np.array([X[0]**2,
                  X[1]**2,
                  X[2]**2,
                  2.0*X[1]*X[2],
                  2.0*X[0]*X[2],
                  2.0*X[0]*X[1],
                  2.0*X[0],
                  2.0*X[1],
                  2.0*X[2],
                  np.ones_like(X[0])])

    S = np.dot(D, D.T)
    S_11 = S[:6,:6]
    S_12 = S[:6,6:]
    S_21 = S[6:,:6]
    S_22 = S[6:,6:]

    k = 4.0
    k_2_1 = k/2.0 - 1.0
    C_1 = np.array([[   -1, k_2_1, k_2_1,  0,  0,  0],
                    [k_2_1,    -1, k_2_1,  0,  0,  0],
                    [k_2_1, k_2_1,    -1,  0,  0,  0],
                    [    0,     0,     0, -k,  0,  0],
                    [    0,     0,     0,  0, -k,  0],
                    [    0,     0,     0,  0,  0, -k]], float)

    S_22_inv = np.linalg.pinv(S_22)
    E = np.dot(np.linalg.inv(C_1), S_11 - np.dot(S_12, np.dot(S_22_inv, S_21)))
    E_w, E_v = np.linalg.eig(E)

    v_1 = E_v[:, np.argmax(E_w)]
    if v_1[0] < 0: v_1 = -v_1
    v_2 = np.dot(np.dot(-S_22_inv, S_21), v_1)

    M = np.array([[v_1[0], v_1[5], v_1[4]],
                  [v_1[5], v_1[1], v_1[3]],
                  [v_1[4], v_1[3], v_1[2]]])
    n = np.array([[v_2[0]],
                  [v_2[1]],
                  [v_2[2]]])
    d = v_2[3]

    M_inv = np.linalg.inv(M)
    A_inv = ref_norm / np.sqrt(np.dot(n.T, np.dot(M_inv, n)) - d) * sqrtm(M)
    b = -np.dot(M_inv, n)

    return (np.linalg.inv(A_inv), b.reshape(3,))

# This initial solution will next be fine-tuned by iterative optimization
print(node_name+": initializing...\n")
soft_init, hard_init = initial_fit(dataset.T)

##################################################

# Flattens a symmetric 3-by-3 matrix into a 6-vector
def vec_from_sym3(sym3):
    return np.array([sym3[0,0], sym3[1,1], sym3[2,2], sym3[0,1], sym3[0,2], sym3[1,2]], float)

# Unpacks a 6-vector into a symmetric 3-by-3 matrix
def sym3_from_vec(vec):
    return np.array([[vec[0], vec[3], vec[4]],
                     [vec[3], vec[1], vec[5]],
                     [vec[4], vec[5], vec[2]]], float)

# Objective for fine-tuned fitting
def misalignment(parameters, dataset=dataset):
    a = sym3_from_vec(parameters[0:6])
    b = parameters[6:9]
    d = dataset - b
    r = np.sum(np.dot(d, a) * d, axis=1)
    return np.mean(np.square(r - ref_norm**2))

# Perform the fine-tuning
print(node_name+": optimizing...\n")
fit = minimize(
    fun=misalignment,
    x0=np.concatenate((vec_from_sym3(np.linalg.matrix_power(soft_init, -2)), hard_init)),
    method="Nelder-Mead",
    tol=1e-9*ref_norm,
    options={"maxiter": 10000, "disp": False})

# Extract the soft/hard from the result
soft = np.linalg.inv(sqrtm(sym3_from_vec(fit.x[0:6])))
hard = fit.x[6:9]

##################################################

# Display the results
print("==============================")
print("CALIBRATE MAGNETOMETER RESULTS")
print("==============================")
print("Soft Distortion Matrix:")
print(np.array2string(soft, separator=", "))
print("=====")
print("Hard Distortion Vector:")
print(np.array2string(hard, separator=", "))
print("==============================\n")

# Provide visualization of the calibrated dataset
calibrated = np.dot(dataset - hard, np.linalg.inv(soft))
visualization.points = [Point32(*(point / ref_norm)) for point in calibrated]
visualization.header.stamp = rospy.get_rostime()
for _ in range(10):
    visualization_pub.publish(visualization)
    rospy.sleep(0.1)

##################################################
