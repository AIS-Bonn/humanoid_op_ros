cmake_minimum_required(VERSION 2.8.3)
project(state_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-address")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS test_utilities)

#######################################
## Declare ROS messages and services ##
#######################################

#add_message_files(
#  FILES
#)

#add_action_files(
#  FILES
#)

# add_service_files(
#   FILES
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs  # Or other packages containing msgs
#)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES state_controller
#	CATKIN_DEPENDS
#       DEPENDS
)

###########
## Build ##
###########

include_directories(include
	${catkin_INCLUDE_DIRS}
)

add_library(state_controller
	src/state_controller.cpp
)
target_link_libraries(state_controller
	${catkin_LIBRARIES}
)

###############
## Unit Test ##
###############

catkin_add_gtest(test_state_controller
	test/test_state_controller.cpp
)
target_link_libraries(test_state_controller
	state_controller
	test_utilities
	${catkin_LIBRARIES}
	${GTEST_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS state_controller
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
