cmake_minimum_required(VERSION 2.8.3)
project(behaviour_exercise)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-address")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	behaviour_control state_controller test_utilities
	robotcontrol roscpp config_server tf tf_conversions plot_msgs gait_msgs
	head_control field_model motion_player
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
include_directories(include
	${catkin_INCLUDE_DIRS}
)

add_executable(behaviour_exercise
	src/behaviour_exercise.cpp
	src/behaviour_exercise_sample.cpp # Note: Switch this line between 'sample' and 'template' to choose which version to compile (a #include also needs to change in behaviour_exercise.cpp)
)

target_link_libraries(behaviour_exercise
	${catkin_LIBRARIES}

)

#############
## Install ##
#############

install(TARGETS behaviour_exercise
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
