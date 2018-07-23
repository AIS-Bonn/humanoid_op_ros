cmake_minimum_required(VERSION 2.8.3)
project(rot_conv)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-address -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cmake_modules)

# Find Eigen
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
if(EIGEN3_FOUND)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
else()
	find_package(Eigen REQUIRED)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()
include_directories(${EIGEN3_INCLUDE_DIRS})

###################################################
## Declare things to be passed to other projects ##
###################################################

## INCLUDE_DIRS: include directories of this project that dependent projects also need
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
	LIBRARIES rot_conv
#	CATKIN_DEPENDS
# 	DEPENDS
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
	src/rot_conv_extras.cpp
)
target_link_libraries(rot_conv
	${catkin_LIBRARIES}
)

## Build the sample executable
add_executable(rot_conv_sample
	test/rot_conv_sample.cpp
)
target_link_libraries(rot_conv_sample
	rot_conv
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
