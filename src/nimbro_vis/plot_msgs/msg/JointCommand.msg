## @ingroup plot_msgs
# @defgroup JointCommand JointCommand
#
# &nbsp;
#
# Message type used to publish/read in the current joint state commands for the robot.
# This complements the joint state information provided by `sensor_msgs/JointState`.
#
# @param "Header header" Standard ROS header (includes time stamp)
# @param "string[] name" Array of joint names
# @param "float64[] position" Array of position commands (rad)
# @param "float64[] velocity" Array of velocity commands (rad/s)
# @param "float64[] acceleration" Array of acceleration commands (rad/s^2)
# @param "float64[] rawPosition" Array of raw commanded joint positions (rad)
# @param "float64[] torque" Array of required servo torques computed using inverse dynamics (Nm)
# @param "float64[] effort" Array of commanded joint efforts (in the range `[0,1]`)

Header header
string[] name
float64[] position
float64[] velocity
float64[] acceleration
float64[] rawPosition
float64[] torque
float64[] effort
