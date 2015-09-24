cmake_minimum_required(VERSION 2.8.3)
project(vis_utils)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-address")

#######################
## Find dependencies ##
#######################

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rostest visualization_msgs)

###################################################
## Declare things to be passed to other projects ##
###################################################

## INCLUDE_DIRS: include directories of this project that dependent projects also need
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES marker_manager
#	CATKIN_DEPENDS
#	DEPENDS
)

###########
## Build ##
###########

## Specify include directories
include_directories(include
	${catkin_INCLUDE_DIRS}
)

## Build the marker_manager library
add_library(marker_manager
	src/marker_manager.cpp
)
target_link_libraries(marker_manager
	${catkin_LIBRARIES}
)

###############
## Unit Test ##
###############

## Build the marker_manager unit test
add_rostest_gtest(marker_manager_test
	test/marker_manager_test.test
	test/marker_manager_test.cpp
)
target_link_libraries(marker_manager_test
	marker_manager
	${catkin_LIBRARIES}
	${GTEST_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS marker_manager
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} # Install the library TARGETS here
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} # Install the executable TARGETS here
)
# EOF