# The Planner's full plan including metadata.
# All arrays must have the same size except for path.
##################################################

# Timing
time      stamp  # [s]
float64[] time   # [s]

# States
geometry_msgs/Point[]      pos  # [m]
geometry_msgs/Vector3[]    vel  # [m/s]
geometry_msgs/Quaternion[] ori  # []

# Actions
float64[]               thrust  # [N]
geometry_msgs/Vector3[] angvel  # [rad/s]

# Desire
geometry_msgs/Point[] path     # positional reference path, [m]
geometry_msgs/Point   proxy    # point of tracking progress along the path (the "carrot"), [m]
bool                  looping  # true if progress along the path is past the loop point

# Meta
float64 bill         # total cost, [$]
float64 regularizer  # Levenberg-Marquardt buffer used in last optimization
