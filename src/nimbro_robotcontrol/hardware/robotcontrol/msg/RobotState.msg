## @ingroup robotcontrol_msgs
# @defgroup RobotState RobotState
#
# &nbsp;
#
# Message type used by RobotModel to publish the current robot state.
#
# @param "time stamp" Time stamp of the message
# @param "string frame_id" The ID of the coordinate frame to which the data in the message pertains
# @param "Quaternion orientation" Estimated orientation of the robot expressed as a quaternion
# @param "Quaternion orientationPR" Estimated fused pitch/roll/hemi orientation of the robot expressed as a quaternion (fused without magnetometer)
# @param "Quaternion orientationNoEYaw" Estimated orientation of the robot with the ZYX Euler yaw component removed, expressed as a quaternion (derived from orientation)
# @param "Quaternion orientationNoFYaw" Estimated orientation of the robot with the fused yaw component removed, expressed as a quaternion (derived from orientation)
# @param "Quaternion orientationNoHeading" Estimated orientation of the robot with the heading component removed, expressed as a quaternion (one of orientationNoEYaw and orientationNoFYaw depending on the configuration, derived from orientation)
# @param "Vector3 angVel" Measured angular velocity of the robot in body-fixed coordinates
# @param "Vector3 accVec" Measured acceleration vector of the robot in body-fixed coordinates
# @param "Vector3 magVec" Measured magnetic field vector of the robot in body-fixed coordinates
# @param "float32 EYaw" Estimated robot ZYX Euler yaw angle (derived from orientation)
# @param "float32 EPitch" Estimated robot ZYX Euler pitch angle (derived from orientation)
# @param "float32 ERoll" Estimated robot ZYX Euler roll angle (derived from orientation)
# @param "float32 FYaw" Estimated robot fused yaw angle (derived from orientation)
# @param "float32 FPitch" Estimated robot fused pitch angle (derived from orientation)
# @param "float32 FRoll" Estimated robot fused roll angle (derived from orientation)
# @param "int8 FHemi" Estimated robot orientation hemisphere (1 for positive z hemisphere, -1 for negative z hemisphere, derived from orientation)
# @param "float32 FPitchPR" Estimated robot fused pitch angle (derived from orientationPR)
# @param "float32 FRollPR" Estimated robot fused roll angle (derived from orientationPR)
# @param "int8 FHemiPR" Estimated robot orientation hemisphere (1 for positive z hemisphere, -1 for negative z hemisphere, derived from orientationPR)
# @param "float32 heading" Estimated robot heading (one of EYaw and FYaw depending on the configuration, derived from orientation)
# 
# @sa <a href="http://docs.ros.org/api/geometry_msgs/html/msg/Quaternion.html">geometry_msgs::Quaternion</a>
# @sa <a href="http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html">geometry_msgs::Vector3</a>

time stamp
string frame_id
geometry_msgs/Quaternion orientation
geometry_msgs/Quaternion orientationPR
geometry_msgs/Quaternion orientationNoEYaw
geometry_msgs/Quaternion orientationNoFYaw
geometry_msgs/Quaternion orientationNoHeading
geometry_msgs/Vector3 angVel
geometry_msgs/Vector3 accVec
geometry_msgs/Vector3 magVec
float32 EYaw
float32 EPitch
float32 ERoll
float32 FYaw
float32 FPitch
float32 FRoll
int8 FHemi
float32 FPitchPR
float32 FRollPR
int8 FHemiPR
float32 heading
