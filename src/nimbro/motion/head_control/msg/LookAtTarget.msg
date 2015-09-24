bool enabled               # Specify whether to enable/disable head control (all other fields are ignored if this is set)
bool is_angular_data       # Specifies whether vec is angular pitch/yaw data, or a camera frame vector
bool is_relative           # Specifies whether the angular data is relative to the current position, or absolute (ignored if not angular data)
geometry_msgs/Vector3 vec  # Vector specifying what head control should aim to look at => (#, pitch, yaw) OR (x, y, z)
float32 pitchEffort        # Effort value in (0,1] for the pitch servo (0 means use default, anything else means relax)
float32 yawEffort          # Effort value in (0,1] for the yaw servo (0 means use default, anything else means relax)
