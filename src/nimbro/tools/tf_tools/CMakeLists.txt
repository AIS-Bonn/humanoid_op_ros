cmake_minimum_required(VERSION 2.8.3)
project(tf_tools)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-address")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp tf)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES tf_tools
#	CATKIN_DEPENDS
#	DEPENDS
)

###########
## Build ##
###########

include_directories(include
	${catkin_INCLUDE_DIRS}
)

add_library(tf_tools
	src/tf_filtered_listener.cpp
)
target_link_libraries(tf_tools
	${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS tf_tools
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
