cmake_minimum_required(VERSION 2.8.3)
project(humanoid_kinematics)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS config_server rot_conv rc_utils roscpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

###################################################
## Declare things to be passed to other projects ##
###################################################

## INCLUDE_DIRS: include directories of this project that dependent projects also need
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES humanoid_kinematics
	CATKIN_DEPENDS config_server rot_conv rc_utils
# 	DEPENDS
)

###########
## Build ##
###########

## Include directories
include_directories(include
	${catkin_INCLUDE_DIRS}
)

## Build the humanoid kinematics library
add_library(humanoid_kinematics
	src/pose_classes.cpp
	src/robot_kinematics.cpp
	src/serial/serial_pose_classes.cpp
	src/serial/serial_kinematics.cpp
)
target_link_libraries(humanoid_kinematics
	${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS humanoid_kinematics
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
# EOF
