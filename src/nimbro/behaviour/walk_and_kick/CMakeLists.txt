cmake_minimum_required(VERSION 2.8.3)
project(walk_and_kick)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-address")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	robotcontrol roscpp config_server tf plot_msgs gait_msgs gait timer field_model nimbro_op_interface nimbro_utils head_control
)

find_library(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

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
#	INCLUDE_DIRS
#	LIBRARIES
#	CATKIN_DEPENDS
#       DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(walk_and_kick
	src/walk_and_kick.cpp
	src/Vec2f.cpp
	src/Vec3f.cpp
)

target_link_libraries(walk_and_kick
	${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS walk_and_kick
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
