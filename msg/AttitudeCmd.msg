# Attitude command from the Planner to the Regulator.
##################################################

time stamp  # [s]

geometry_msgs/Quaternion ori  # (zero implies ignore), []

geometry_msgs/Vector3 angvel  # [rad/s]

geometry_msgs/Vector3 angaccel  # (zero implies ignore), [rad/s^2]
