cmake_minimum_required(VERSION 2.8.3)
project(nimbro_utils)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-address")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rostest config_server cmake_modules)

# Find and include Eigen (for compiling nimbro_utils_test - users of this package may or may not need Eigen depending on which headers they are using of this package)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

###################################################
## Declare things to be passed to other projects ##
###################################################

## INCLUDE_DIRS: include directories of this project that dependent projects also need
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
#	LIBRARIES
#	CATKIN_DEPENDS
	DEPENDS Eigen
)

###########
## Build ##
###########

## Include directories
include_directories(include
	${catkin_INCLUDE_DIRS}
)

###############
## Unit Test ##
###############

## Test ROS-based utilities
add_rostest_gtest(nimbro_utils_test
	test/nimbro_utils_test.test
	test/nimbro_utils_test.cpp
)
target_link_libraries(nimbro_utils_test
	${catkin_LIBRARIES}
	${GTEST_LIBRARIES}
)
add_dependencies(nimbro_utils_test config_server_gencpp)
# EOF
