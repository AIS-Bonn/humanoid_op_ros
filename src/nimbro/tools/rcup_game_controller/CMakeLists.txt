cmake_minimum_required(VERSION 2.8.3)
project(rcup_game_controller)

find_package(catkin REQUIRED COMPONENTS roscpp genmsg std_msgs config_server)

#######################################
## Declare ROS messages and services ##
#######################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  GCRobotInfo.msg
  GCTeamInfo.msg
  GCData.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES 
#  CATKIN_DEPENDS
#  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(game_controller
  src/game_controller.cpp
  src/packet_parser_v8.cpp
)
target_link_libraries(game_controller
  ${catkin_LIBRARIES}
)

add_dependencies(game_controller rcup_game_controller_gencpp)

install(TARGETS game_controller
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
