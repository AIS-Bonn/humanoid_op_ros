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
   LIBRARIES rcup_game_controller
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(rcup_game_controller
#   src/${PROJECT_NAME}/rcup_game_controller.cpp
# )

## Declare a cpp executable
add_executable(game_controller
  src/gamecontroller.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(rcup_game_controller_node rcup_game_controller_generate_messages_cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(game_controller
  ${catkin_LIBRARIES}
)

add_dependencies(game_controller rcup_game_controller_gencpp)

install(TARGETS game_controller
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

