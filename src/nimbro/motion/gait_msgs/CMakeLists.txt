cmake_minimum_required(VERSION 2.8.3)
project(gait_msgs)

#######################
## Find dependencies ##
#######################

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp message_generation std_msgs geometry_msgs)

###############################
## ROS messages and services ##
###############################

## Add message and service types
add_message_files(DIRECTORY msg FILES
	GaitCommand.msg
	GaitPerformance.msg
	GaitOdom.msg
)
add_service_files(DIRECTORY srv FILES
	SetOdom.srv
)

## Generate added messages and services with any message dependencies listed here
generate_messages(DEPENDENCIES
	std_msgs
	geometry_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## INCLUDE_DIRS: include directories of this project that dependent projects also need
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
# 	INCLUDE_DIRS
# 	LIBRARIES
	CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
# 	DEPENDS
)
# EOF