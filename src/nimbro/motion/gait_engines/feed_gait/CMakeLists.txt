cmake_minimum_required(VERSION 2.8.3)
project(feed_gait)

#######################
## Find dependencies ##
#######################

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS config_server gait humanoid_kinematics rot_conv roscpp test_utilities)

###################################################
## Declare things to be passed to other projects ##
###################################################

## INCLUDE_DIRS: include directories of this project that dependent projects also need
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES feed_gait
	CATKIN_DEPENDS rot_conv
# 	DEPENDS
)

###########
## Build ##
###########

## Specify include directories
include_directories(include
	${catkin_INCLUDE_DIRS}
)

## Build the feed_gait library
add_library(feed_gait
	src/feed_gait.cpp
	src/feed_utils.cpp
	src/feed_common.cpp
	src/kinematics/feed_kinematics_base.cpp
	src/kinematics/serial/feed_kinematics_impl_serial.cpp
	src/trajectory/keypoint/feed_keypoint_traj.cpp
	src/trajectory/keypoint/feed_keypoint_traj_common.cpp
	src/trajectory/keypoint/feed_kinematics_kt_impl_serial.cpp
	src/odometry/feed_odometry_base.cpp
	src/odometry/trivial/feed_trivial_odom.cpp
	src/model/feed_model_base.cpp
	src/model/trivial/feed_trivial_model.cpp
	src/model/tilt_phase/feed_tilt_phase_model.cpp
	src/model/tilt_phase/tripendulum_model.cpp
)
target_link_libraries(feed_gait
	${catkin_LIBRARIES}
)

## Build the non-unit test nodes
add_executable(test_feed_gait
	test/test_feed_gait.cpp
)
target_link_libraries(test_feed_gait
	feed_gait
	${catkin_LIBRARIES}
)

## Build the unit test nodes
catkin_add_gtest(test_tilt_phase_model
	test/test_tilt_phase_model.cpp
)
target_link_libraries(test_tilt_phase_model
	feed_gait
	${catkin_LIBRARIES}
	${GTEST_LIBRARIES}
)
catkin_add_gtest(test_tripendulum
	test/test_tripendulum.cpp
)
target_link_libraries(test_tripendulum
	feed_gait
	${catkin_LIBRARIES}
	${GTEST_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS feed_gait
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} # Install the library TARGETS here
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} # Install the executable TARGETS here
)
install(FILES plugin_gait.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
