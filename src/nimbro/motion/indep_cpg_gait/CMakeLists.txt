cmake_minimum_required(VERSION 2.8.3)
project(indep_cpg_gait)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp robotcontrol nimbro_op_model gait gait_msgs cmake_modules)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

include_directories(src/gaitcode)

find_package(Qt4 REQUIRED QtCore QtXml QtOpenGL)
include(${QT_USE_FILE})

find_library(QGLVIEWER_LIBRARY NAMES qglviewer-qt4 QGLViewer REQUIRED)
find_path(QGLVIEWER_INCLUDE manipulatedFrame.h PATH_SUFFIXES QGLViewer REQUIRED)

find_library(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

include_directories(${QGLVIEWER_INCLUDE})

###################################################
## Declare things to be passed to other projects ##
###################################################

## INCLUDE_DIRS: include directories of this project that dependent projects also need
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
# 	INCLUDE_DIRS
# 	LIBRARIES
# 	CATKIN_DEPENDS
# 	DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
	${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(indep_cpg_gait
  src/indep_cpg_gait.cpp
  src/gaitcode/Command.cpp
  src/gaitcode/Config.cpp
  src/gaitcode/State.cpp
  src/gaitcode/RobotModel/RobotModel.cpp
  src/gaitcode/RobotControl/RobotControl.cpp
  src/gaitcode/RobotControl/MotionLayer/DynamicGait.cpp
  src/gaitcode/RobotControl/MotionLayer/Fall.cpp
  src/gaitcode/RobotControl/MotionLayer/Halt.cpp
  src/gaitcode/RobotControl/MotionLayer/Motion.cpp
  src/gaitcode/RobotControl/MotionLayer/MotionLayer.cpp
  src/gaitcode/RobotControl/MotionLayer/MotionTest.cpp
  src/gaitcode/RobotControl/MotionLayer/MotionInterfaceTest.cpp
  src/gaitcode/util/StopWatch_linux.cpp
  src/gaitcode/util/Vec2f.cpp
  src/gaitcode/util/Vec3f.cpp
  src/configclient/configclient.cpp
  src/configclient/configitem.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(indep_cpg_gait
  ${catkin_LIBRARIES}
  ${QGLVIEWER_LIBRARY}
)

#############
## Install ##
#############

install(TARGETS indep_cpg_gait
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_robotcontrol.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
