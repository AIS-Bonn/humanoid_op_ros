cmake_minimum_required(VERSION 2.8.3)
project(rot_conv)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-address")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cmake_modules)

# Find and include Eigen
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
	LIBRARIES rot_conv
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

## Build the rot_conv library
add_library(rot_conv
	src/rot_conv.cpp
)
target_link_libraries(rot_conv
	${catkin_LIBRARIES}
)

################
## Unit tests ##
################

## Rotation conversions test
catkin_add_gtest(test_rot_conv
	test/test_rot_conv.cpp
)
target_link_libraries(test_rot_conv
	rot_conv
	${catkin_LIBRARIES}
	${GTEST_LIBRARIES}
)

#############
## Install ##
#############

## Install the generated libraries
install(TARGETS rot_conv
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
# EOF