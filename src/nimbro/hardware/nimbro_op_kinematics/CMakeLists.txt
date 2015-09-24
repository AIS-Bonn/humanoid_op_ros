cmake_minimum_required(VERSION 2.8.3)
project(nimbro_op_kinematics)

find_package(catkin REQUIRED COMPONENTS
	robotcontrol
	nimbro_op_model
	roscpp
	rostest
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES nimbro_op_kinematics
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(nimbro_op_kinematics
	src/leg_ik.cpp
	src/kinematic_model.cpp
	src/leg.cpp
)
target_link_libraries(nimbro_op_kinematics
	${catkin_LIBRARIES}
)

add_library(nimbro_op_kinematics_iktest
	src/iktest.cpp
)
target_link_libraries(nimbro_op_kinematics_iktest
	${catkin_LIBRARIES}
	nimbro_op_kinematics
)

# Tests
add_rostest_gtest(test_leg_ik test/test_leg_ik.test
	test/test_leg_ik.cpp
	src/leg_ik.cpp
)
target_link_libraries(test_leg_ik
	${catkin_LIBRARIES}
)

# Install files
install(TARGETS nimbro_op_kinematics
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_robotcontrol.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
