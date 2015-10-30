cmake_minimum_required(VERSION 2.8.3)
project(robotcontrol)

# TODO: Place this somewhere top-level
if(NOT CMAKE_BUILD_TYPE)
	message(STATUS "Build type not set - using RelWithDebInfo per default")
	set(CMAKE_BUILD_TYPE "RelWithDebInfo")
	set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING
		"Choose the type of build, options are: Debug Release
		RelWithDebInfo MinSizeRel."
		FORCE)
endif()

if(CMAKE_CXX_FLAGS STREQUAL "")
	set(CMAKE_CXX_FLAGS "-Wall" CACHE STRING
		"Flags used by the compiler during all build types."
		FORCE
	)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-address")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	timer servomodel message_generation
	roscpp pluginlib rbdl actionlib actionlib_msgs
	config_server tf plot_msgs eigen_conversions
	tf_conversions geometry_msgs vis_utils urdf
	rot_conv control_msgs
)

## System dependencies are found with CMake's conventions
set(Boost_USE_STATIC_LIBS       OFF)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package(Boost REQUIRED COMPONENTS regex)

# Try to find gazebo. If we do not find it, disable GazeboInterface.
find_package(gazebo_msgs QUIET)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/robotcontrol)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  Diagnostics.msg
  RobotHeading.msg
  RobotState.msg
  ServoDiag.msg
  State.msg
)

add_service_files(
  FILES
  MagCalib.srv
  WarpAddPoint.srv
)

add_action_files(
  FILES
  FadeTorque.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES robotmodel
	CATKIN_DEPENDS config_server rbdl message_runtime eigen_conversions vis_utils
#	DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${cm730_INCLUDE_DIRS}
)

add_library(robotmodel SHARED
	src/model/joint.cpp
	src/model/robotmodel.cpp
	src/model/singlesupportmodel.cpp
	src/model/golay.cpp
	src/model/kalmanfilter.cpp
	src/motion_utils/trunk_stabilizer.cpp
	src/hw/angleestimator.cpp
	src/hw/magfilter.cpp
	src/hw/attitude_estimator.cpp
	src/hw/dynamiccommandgenerator.cpp
	src/hw/hardwareinterface.cpp
	src/motionmodule.cpp
)

target_link_libraries(robotmodel
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

add_executable(robotcontrol
	src/robotcontrol.cpp
	src/motionmodule.cpp
)

add_dependencies(robotcontrol
	robotcontrol_gencpp
)

add_dependencies(robotmodel
	robotcontrol_gencpp
)

add_library(dummy_interface
	src/hw/dummyinterface.cpp
)

target_link_libraries(dummy_interface
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	robotmodel
)

add_library(ros_control_interface
	src/hw/roscontrolinterface.cpp
)

target_link_libraries(ros_control_interface
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	robotmodel
)

target_link_libraries(robotcontrol
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	robotmodel
)

if(gazebo_msgs_FOUND)
	add_library(gazebo_interface
		src/hw/gazebointerface.cpp
	)
	target_link_libraries(gazebo_interface
		${catkin_LIBRARIES}
		${Boost_LIBRARIES}
		robotmodel
		ros_control_interface
	)
	add_dependencies(gazebo_interface
		${gazebo_msgs_EXPORTED_TARGETS}
	)
endif()

#############
## Install ##
#############

install(TARGETS robotmodel dummy_interface robotcontrol
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_robotcontrol.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Attitude estimator test
catkin_add_gtest(test_attitude_estimator
	test/test_attitude_estimator.cpp
)
target_link_libraries(test_attitude_estimator
	robotmodel
	${catkin_LIBRARIES}
	${GTEST_LIBRARIES}
)

## Golay test
catkin_add_gtest(test_golay
	test/test_golay.cpp
	src/model/golay.cpp
)
target_link_libraries(test_golay
	${catkin_LIBRARIES}
	${GTEST_LIBRARIES}
)

## FIR filter test
catkin_add_gtest(test_firfilter
	test/test_firfilter.cpp
)
target_link_libraries(test_firfilter
	${catkin_LIBRARIES}
	${GTEST_LIBRARIES}
)
# EOF
