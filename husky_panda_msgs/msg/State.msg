std_msgs/Header header
geometry_msgs/Vector3 base_pose
geometry_msgs/Twist base_twist
# # for articulated object manipulation
# geometry_msgs/Vector3 base_effort
sensor_msgs/JointState arm_state
# # for articulated object manipulation
# float64 object_position
# float64 object_velocity
# bool in_contact

int64 SIZE = 24