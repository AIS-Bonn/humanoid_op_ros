## @ingroup robotcontrol_msgs
# @defgroup Diagnostics Diagnostics
#
# &nbsp;
#
# Message type used to publish current diagnostics about the robot, such as battery voltage
# and servo temperatures.
#
# @param "Header header" Standard ROS header (includes time stamp)
# @param "float32 batteryVoltage" Measured battery voltage (in volts)
# @param "float32 servoTemperature" Measured maximum servo temperature (in degrees Celcius)
# @param "string state" String literal of the current robot state as per robotcontrol (e.g. "standing")
# @param "ServoDiag[] servos" Array of diagnostics information for each of the individual servos
#
# @sa @ref ServoDiag

Header header
float32 batteryVoltage
float32 servoTemperature
string state
ServoDiag[] servos
