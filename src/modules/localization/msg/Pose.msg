# Position of the vehicle reference point (VRP) in the map reference frame.
# The VRP is the center of rear axle.
geometry_msgs/Point position

# A quaternion that represents the rotation from the IMU coordinate
# (Right/Forward/Up) to the
# world coordinate (East/North/Up).
geometry_msgs/Quaternion orientation

# Linear velocity of the VRP in the map reference frame.
# East/north/up in meters per second.
geometry_msgs/Point linear_velocity

# Linear acceleration of the VRP in the map reference frame.
# East/north/up in meters per second.
geometry_msgs/Point linear_acceleration

# Angular velocity of the vehicle in the map reference frame.
# Around east/north/up axes in radians per second.
geometry_msgs/Point angular_velocity

# Heading
# The heading is zero when the car is facing East and positive when facing
# North.
float64 heading

# Linear acceleration of the VRP in the vehicle reference frame.
# Right/forward/up in meters per square second.
geometry_msgs/Point linear_acceleration_vrf

# Angular velocity of the VRP in the vehicle reference frame.
# Around right/forward/up axes in radians per second.
geometry_msgs/Point angular_velocity_vrf

# Roll/pitch/yaw that represents a rotation with intrinsic sequence z-x-y.
# in world coordinate (East/North/Up)
# The roll, in (-pi/2, pi/2), corresponds to a rotation around the y-axis.
# The pitch, in [-pi, pi), corresponds to a rotation around the x-axis.
# The yaw, in [-pi, pi), corresponds to a rotation around the z-axis.
# The direction of rotation follows the right-hand rule.
geometry_msgs/Point euler_angles