cmake_minimum_required(VERSION 2.8.3)
project(walk_and_kick)

# Set some compilation flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp robotcontrol config_server tf plot_msgs vis_utils gait_msgs gait timer field_model nimbro_op_interface rc_utils head_control vision_module rcup_game_controller geometry_msgs)

# Find Eigen
find_library(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(FILES
  BehaviourState.msg
  TeamCommsData.msg
)

add_service_files(
  FILES
  VisualiseDBH.srv
  VisualiseDbApp.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES walk_and_kick_gc walk_and_kick_tc walk_and_kick_main
	CATKIN_DEPENDS config_server field_model rc_utils gait gait_msgs
#	DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

# Build the walk_and_kick game controller library
add_library(walk_and_kick_gc
	src/wak_gc_vars.cpp
)
target_link_libraries(walk_and_kick_gc
	${catkin_LIBRARIES}
)
add_dependencies(walk_and_kick_gc walk_and_kick_gencpp ${catkin_EXPORTED_TARGETS})

# Build the walk_and_kick team communications library
add_library(walk_and_kick_tc
	src/wak_tc_ros.cpp
	src/wak_tc_vars.cpp
)
target_link_libraries(walk_and_kick_tc
	${catkin_LIBRARIES}
)
add_dependencies(walk_and_kick_tc walk_and_kick_gencpp ${catkin_EXPORTED_TARGETS})

# Build the walk_and_kick main library
add_library(walk_and_kick_main
	src/walk_and_kick.cpp
	src/wak_actuator_vars.cpp
	src/wak_beh_manager.cpp
	src/wak_beh_shared.cpp
	src/wak_beh_state.cpp
	src/wak_game_manager.cpp
	src/wak_game_shared.cpp
	src/wak_game_state.cpp
	src/wak_game_vars.cpp
	src/wak_ros_interface.cpp
	src/wak_sensor_vars.cpp
	src/wak_utils.cpp
	src/wak_vis.cpp
	src/beh_states/beh_unknown_state.cpp
	src/beh_states/beh_stopped.cpp
	src/beh_states/beh_panic_attack.cpp
	src/beh_states/beh_search_for_ball.cpp
	src/beh_states/beh_go_behind_ball.cpp
	src/beh_states/beh_dribble_ball.cpp
	src/beh_states/beh_kick_ball.cpp
	src/beh_states/beh_dive_for_ball.cpp
	src/beh_states/gaze_beh_look_around.cpp
	src/beh_states/gaze_beh_look_at_ball.cpp
	src/beh_states/gaze_beh_look_down.cpp
	src/beh_states/gaze_beh_look_for_ball.cpp
	src/beh_states/gaze_beh_look_left_right.cpp
	src/beh_states/walk_beh_walk_to_pose.cpp
	src/game_states/game_unknown_state.cpp
	src/game_states/game_stopped.cpp
	src/game_states/game_panic_attack.cpp
	src/game_states/game_positioning.cpp
	src/game_states/game_gaze_for_ball.cpp
	src/game_states/game_wait_for_ball_in_play.cpp
	src/game_states/game_default_ball_handling.cpp
	src/game_states/game_penalty_ball_handling.cpp
	src/game_states/game_default_goalie.cpp
	src/game_states/game_penalty_goalie.cpp
)
target_link_libraries(walk_and_kick_main
	walk_and_kick_gc
	walk_and_kick_tc
	${catkin_LIBRARIES}
)
add_dependencies(walk_and_kick_main walk_and_kick_gencpp ${catkin_EXPORTED_TARGETS})

# Build the walk_and_kick node
add_executable(walk_and_kick
	src/wak_main.cpp
)
target_link_libraries(walk_and_kick
	walk_and_kick_main
	${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Install the required targets
install(TARGETS walk_and_kick walk_and_kick_gc walk_and_kick_tc walk_and_kick_main
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# EOF
