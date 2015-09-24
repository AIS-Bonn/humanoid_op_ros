cmake_minimum_required(VERSION 2.8.3)
project(sitting_demo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp config_server motion_player limb_control face_tracker
	state_controller nimbro_op_interface demo_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
# LIBRARIES
# CATKIN_DEPENDS
# DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# Declare a cpp executable
add_executable(sitting_demo
  src/sitting_demo.cpp
  src/sitting_demo_fsm.cpp
  src/sitting_demo_actions.cpp
)

add_dependencies(sitting_demo sitting_demo_gencpp)

target_link_libraries(sitting_demo
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark other files for installation (e.g. launch and bag files, etc.)
install(TARGETS sitting_demo
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(CODE "file(REMOVE_RECURSE \"${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/launch\")")
install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
# EOF