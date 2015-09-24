cmake_minimum_required(VERSION 2.8.3)
project(gait)

#######################
## Find dependencies ##
#######################

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp robotcontrol tf gait_msgs std_srvs plot_msgs sensor_msgs nimbro_utils pluginlib)

###################################################
## Declare things to be passed to other projects ##
###################################################

## INCLUDE_DIRS: include directories of this project that dependent projects also need
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES gait gait_util
	CATKIN_DEPENDS robotcontrol nimbro_utils
# 	DEPENDS
)

###########
## Build ##
###########

## Specify include directories
include_directories(include ${catkin_INCLUDE_DIRS})

## Build the gait utility library
add_library(gait_util
	src/util/gait_joint_pose.cpp
	src/util/gait_inverse_pose.cpp
	src/util/gait_abstract_pose.cpp
)
target_link_libraries(gait_util
	${catkin_LIBRARIES}
)

## Build the gait library
add_library(gait
	src/gait.cpp
	src/gait_engine.cpp
)
target_link_libraries(gait
	gait_util
	${catkin_LIBRARIES}
)

##########
## Test ##
##########

## Build the gait utility pose tests
catkin_add_gtest(test_gait_util_pose
	test/test_gait_util_pose.cpp
)
target_link_libraries(test_gait_util_pose
	gait_util
	${catkin_LIBRARIES}
	${GTEST_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS gait gait_util
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} # Install the library TARGETS here
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} # Install the executable TARGETS here
)
install(FILES plugin_gait.xml plugin_robotcontrol.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
