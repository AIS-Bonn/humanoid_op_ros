cmake_minimum_required(VERSION 2.8.3)
project(cm730)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy config_server plot_msgs)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cm730
   CATKIN_DEPENDS roscpp config_server
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Build the CM730 library
add_library(cm730
	src/CM730.cpp
	src/CM740mp.cpp
	src/read_thread.cpp
)
target_link_libraries(cm730
	${catkin_LIBRARIES}
)

## Build the CM730 test node
add_executable(cm730_test
	src/cm730test.cpp
)
target_link_libraries(cm730_test
	cm730
	${catkin_LIBRARIES}
)

## Build the CM740 megapacket test node
add_executable(cm740mp_test
	src/cm740mptest.cpp
)
target_link_libraries(cm740mp_test
	cm730
	${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS cm730
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
