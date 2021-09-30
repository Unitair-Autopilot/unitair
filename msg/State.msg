# Navigation state estimate.
##################################################

# Time at which this belief was held
time stamp  # [s]

# Means
geometry_msgs/Point      pos          # position in world-coordinates of the body-origin, [m]
geometry_msgs/Vector3    vel          # velocity in body-coordinates of the body-origin relative to the world-frame, [m/s]
geometry_msgs/Vector3    accel        # acceleration in body-coordinates of the body-origin relative to the world-frame, [m/s^2]
geometry_msgs/Quaternion ori          # orientation of body-coordinates relative to world-coordinates, []
geometry_msgs/Vector3    angvel       # angular velocity in body-coordinates of the body-frame relative to the world-frame, [rad/s]
geometry_msgs/Vector3    bias_accel   # bias of the inertial acceleration input, [m/s^2]
geometry_msgs/Vector3    bias_angvel  # bias of the inertial angular velocity input, [rad/s]
geometry_msgs/Vector3    gravity      # local gravitational field in world-coordinates, [m/s^2]
geometry_msgs/Vector3    wind         # velocity in world-coordinates of the local wind relative to the world-frame, [m/s]
float64                  pressure     # barometric pressure of the local atmosphere, [Pa]

# Standard-deviations (square-roots of the covariance matrix diagonal)
geometry_msgs/Vector3 pos_stdev          # [m]
geometry_msgs/Vector3 vel_stdev          # [m/s]
geometry_msgs/Vector3 accel_stdev        # [m/s^2]
geometry_msgs/Vector3 ori_stdev          # [rad]
geometry_msgs/Vector3 angvel_stdev       # [rad/s]
geometry_msgs/Vector3 bias_accel_stdev   # [m/s^2]
geometry_msgs/Vector3 bias_angvel_stdev  # [rad/s]
geometry_msgs/Vector3 gravity_stdev      # [m/s^2]
geometry_msgs/Vector3 wind_stdev         # [m/s]
float64               pressure_stdev     # [Pa]

# Derived quantities (for convenience)
geometry_msgs/Point lla  # pos with {x,y,z} as latitude, longitude, and altitude, [deg, deg, m]
geometry_msgs/Point rpy  # ori with {x,y,z} as roll (i''), pitch (j'), and yaw (k) Euler angles, [deg]
