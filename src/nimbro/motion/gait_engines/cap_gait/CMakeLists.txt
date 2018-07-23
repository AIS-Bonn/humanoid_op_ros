cmake_minimum_required(VERSION 2.8.3)
project(cap_gait)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#######################
## Find dependencies ##
#######################

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp gait config_server rc_utils rot_conv vis_utils plot_msgs cmake_modules robotcontrol rqt_gui_cpp feed_gait)
find_package(Eigen REQUIRED)

# Decide on a Qt version
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
	set(USE_QT5 true)
else()
	set(USE_QT5 false)
endif()

# Find Qt
if(USE_QT5)
	find_package(Qt5Core REQUIRED)
	find_package(Qt5Xml REQUIRED)
	find_package(Qt5OpenGL REQUIRED)
	set(QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Xml_LIBRARIES} ${Qt5OpenGL_LIBRARIES})
	include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Xml_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS})
else()
	find_package(Qt4 COMPONENTS QtCore QtXml QtOpenGL REQUIRED)
	include(${QT_USE_FILE})
endif()

# Find QGLViewer
find_path(QGLViewer_INCLUDE_DIR NAMES qglviewer.h PATH_SUFFIXES QGLViewer REQUIRED)
find_library(QGLViewer_LIBRARY NAMES qglviewer QGLViewer QGLViewer2 REQUIRED)
message(STATUS "Found QGLViewer: " ${QGLViewer_LIBRARY})
find_library(GLU_LIBRARY NAMES GLU REQUIRED)
message(STATUS "Found GLU: " ${GLU_LIBRARY}) # For QGLViewer

###################################################
## Declare things to be passed to other projects ##
###################################################

## INCLUDE_DIRS: include directories of this project that dependent projects also need
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES cap_gait
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
	${QGLViewer_INCLUDE_DIR}
)

## Build the cap_gait library
add_library(cap_gait
	src/cap_gait.cpp
	src/contrib/RobotModel.cpp
	src/contrib/Vec2f.cpp
	src/contrib/Vec3f.cpp
	src/contrib/Limp.cpp
	src/contrib/LimpState.cpp
	src/contrib/LimpModel.cpp
	src/contrib/KalmanFilter.cpp
	src/contrib/ComFilter.cpp
)
target_link_libraries(cap_gait
	${catkin_LIBRARIES}
	${QGLViewer_LIBRARY}
	${QT_LIBRARIES}
)

## Build the test nodes
add_executable(test_robotmodel
	test/test_robotmodel.cpp
)
target_link_libraries(test_robotmodel
	cap_gait
	${catkin_LIBRARIES}
	${QGLViewer_LIBRARY}
	${GLU_LIBRARY}
)

#############
## Install ##
#############

install(TARGETS cap_gait
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} # Install the library TARGETS here
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} # Install the executable TARGETS here
)
install(FILES plugin_gait.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
