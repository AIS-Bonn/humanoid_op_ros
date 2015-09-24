cmake_minimum_required(VERSION 2.8.3)
project(calibration)

find_package(catkin REQUIRED COMPONENTS
	roscpp nodelet display_image rosbag genmsg sensor_msgs nodelet_gui_manager
	camera_v4l2
)

# Append to CPACK_SOURCE_IGNORE_FILES a semicolon-separated list of
# directories (or patterns, but directories should suffice) that should
# be excluded from the distro.  This is not the place to put things that
# should be ignored everywhere, like "build" directories; that happens in
# rosbuild/rosbuild.cmake.  Here should be listed packages that aren't
# ready for inclusion in a distro.
#
# This list is combined with the list in rosbuild/rosbuild.cmake.  Note
# that CMake 2.6 may be required to ensure that the two lists are combined
# properly.  CMake 2.4 seems to have unpredictable scoping rules for such
# variables.
#list(APPEND CPACK_SOURCE_IGNORE_FILES /core/experimental)

find_package(Qt4 COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

add_message_files(
	FILES
	CalibrationLUT.msg
	ColorSpec.msg
)

add_service_files(
	FILES
	ClassifyImage.srv
	UpdateLUT.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  sensor_msgs
)

catkin_package(
)

include_directories(${catkin_INCLUDE_DIRS})

qt4_wrap_cpp(MOC_SRCS
	calibscene.h
	calibwidget.h
	calibration.h
	calibrationnodelet.h
	calibimagewidget.h
	histwidget.h
	calibview.h
	cameraparamswidget.h
	calibellipse.h
	playbackcontrol.h
)
qt4_wrap_ui(UI_SRCS calibration.ui)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(calibration
	calibration.cpp calibscene.cpp calibellipse.cpp calibwidget.cpp
	calibrationnodelet.cpp calibimagewidget.cpp histwidget.cpp calibview.cpp
	cameraparamswidget.cpp histogram.cpp playbackcontrol.cpp
	${MOC_SRCS} ${UI_SRCS}
)

add_dependencies(calibration calibration_gencpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(calibration yaml-cpp ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(FILES plugin_nodelet.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
