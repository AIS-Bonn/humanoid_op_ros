cmake_minimum_required(VERSION 2.8.3)
project(timewarp)

find_package(catkin REQUIRED COMPONENTS roscpp topic_tools tf genmsg rosgraph_msgs rosbag)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate services in the 'srv' folder
add_service_files(
	FILES
	TimeWarpControl.srv
	TimeWarpSave.srv
	TimeWarpLoad.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES timewarp
#  CATKIN_DEPENDS roscpp topic_tools
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
	${catkin_INCLUDE_DIRS}
)

add_executable(timewarp
	src/tfhandler.cpp
	src/timewarpnode.cpp
	src/timewarpio.cpp
	src/topichandler.cpp
	src/topicthread.cpp
	src/genericcallback.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(timewarp timewarp_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(timewarp
	${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS timewarp
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(CODE "file(REMOVE_RECURSE \"${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/launch\")")
install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
