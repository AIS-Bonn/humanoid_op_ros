## @ingroup robotcontrol_msgs
# @defgroup RobotHeading RobotHeading
#
# &nbsp;
#
# Message type used by RobotModel to publish the current estimated robot heading.
#
# @param "time stamp" Time stamp of the message
# @param "float32 heading" Estimated robot heading (one of ZYX yaw and fused yaw depending on the configuration)
# @param "Quaternion orientationPR" Estimated fused pitch/roll/hemi orientation of the robot expressed as a quaternion (fused without magnetometer)
# 
# @sa <a href="http://docs.ros.org/api/geometry_msgs/html/msg/Quaternion.html">geometry_msgs::Quaternion</a>

time stamp
float32 heading
geometry_msgs/Quaternion orientationPR
