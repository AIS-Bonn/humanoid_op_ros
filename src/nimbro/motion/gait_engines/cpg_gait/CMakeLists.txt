cmake_minimum_required(VERSION 2.8.3)
project(cpg_gait)

#######################
## Find dependencies ##
#######################

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp gait config_server nimbro_utils plot_msgs cmake_modules robotcontrol)
find_package(Eigen REQUIRED)

###################################################
## Declare things to be passed to other projects ##
###################################################

## INCLUDE_DIRS: include directories of this project that dependent projects also need
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES cpg_gait
# 	CATKIN_DEPENDS
	DEPENDS Eigen
)

###########
## Build ##
###########

## Specify include directories
include_directories(include
	${Eigen_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

## Build the cpg_gait library
add_library(cpg_gait
	src/cpg_gait.cpp
)
target_link_libraries(cpg_gait
	${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS cpg_gait
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} # Install the library TARGETS here
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} # Install the executable TARGETS here
)
install(FILES plugin_gait.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
