//It is a modified version of https://github.com/ros/ros_comm/
// Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#include <string>
#include "macros.h"

namespace nimbro_relay
{

// Strip any leading namespace qualification from a topic (or other kind
// of) ROS name
NIMBRO_RELAY_DECL bool getBaseName(const std::string& full_name, std::string& base_name);

}
