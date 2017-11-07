cmake_minimum_required(VERSION 2.8.3)
project(plot_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg roscpp rostest cmake_modules)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
	FILES
	JointCommand.msg
	Plot.msg
	PlotPoint.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs  # Or other packages containing msgs
)

# Find Eigen
find_package(Eigen3)
if(EIGEN3_FOUND)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
else()
	find_package(Eigen REQUIRED)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()
include_directories(${EIGEN3_INCLUDE_DIRS})

catkin_package(
	INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
# 	DEPENDS
)

## Specify additional locations of header files
include_directories(include
	${catkin_INCLUDE_DIRS}
)

add_rostest_gtest(plot_manager_test
	test/plot_manager_test.test
	test/plot_manager_test.cpp
)
target_link_libraries(plot_manager_test
	${catkin_LIBRARIES}
	${GTEST_LIBRARIES}
)
add_dependencies(plot_manager_test plot_msgs_gencpp)