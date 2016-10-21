//It is a modified version of https://github.com/ros/ros_comm/
// Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#ifndef NIMBRO_RELAY_MACROS_H_
#define NIMBRO_RELAY_MACROS_H_

#include <ros/macros.h> // for the DECL's

// Import/export for windows dll's and visibility for gcc shared libraries.

#ifdef ROS_BUILD_SHARED_LIBS // ros is being built around shared libraries
  #ifdef topic_tools_EXPORTS // we are building a shared lib/dll
    #define NIMBRO_RELAY_DECL ROS_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define NIMBRO_RELAY_DECL ROS_HELPER_IMPORT
  #endif
#else // ros is being built around static libraries
  #define NIMBRO_RELAY_DECL
#endif

#endif /* NIMBRO_RELAY_MACROS_H_ */

