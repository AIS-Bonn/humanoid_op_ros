cmake_minimum_required(VERSION 2.8.3)
project(motion_player)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs robotcontrol genmsg gait plot_msgs message_generation rqt_gui_cpp)

# Decide on a Qt version
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
	set(USE_QT5 true)
else()
	set(USE_QT5 false)
endif()

# Find Qt
if(USE_QT5)
	find_package(Qt5Core REQUIRED)
	set(QT_LIBRARIES ${Qt5Core_LIBRARIES})
	include_directories(${Qt5Core_INCLUDE_DIRS})
else()
	find_package(Qt4 COMPONENTS QtCore REQUIRED)
	include(${QT_USE_FILE})
endif()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_message_files(DIRECTORY msg FILES
	MotionPlayerState.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  PlayMotion.srv
  StreamMotion.srv
  DumpMotions.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
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
   LIBRARIES motion_file
   CATKIN_DEPENDS message_runtime
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

# Declare a cpp library
add_library(motion_player
  src/motionplayer.cpp
  src/keyframe_player/Keyframe.cpp
  src/keyframe_player/KeyframePlayer.cpp
)

add_library(motion_file
  src/motion_file/motionfile.cpp
  src/motion_file/poseconverter.cpp
  src/motion_file/ruleapplier.cpp
)

## Declare a cpp executable
# add_executable(motion_player_node src/motion_player_node.cpp)

# Add cmake target dependencies of the executable/library
# as an example, message headers may need to be generated before nodes
# add_dependencies(motion_player_node motion_player_generate_messages_cpp)

add_dependencies(motion_player motion_player_gencpp)

target_link_libraries(motion_player
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  motion_file
)

target_link_libraries(motion_file
  ${catkin_LIBRARIES}
  yaml-cpp
)

install(TARGETS motion_player motion_file
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_robotcontrol.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


