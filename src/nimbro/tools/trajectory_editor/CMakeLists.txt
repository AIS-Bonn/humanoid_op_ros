cmake_minimum_required(VERSION 2.8.3)
project(trajectory_editor)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS roscpp rviz rbdl urdf motion_player nimbro_op_model gait gait_msgs roslib rqt_gui_cpp)

# Decide on a Qt version
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
	set(USE_QT5 true)
else()
	set(USE_QT5 false)
endif()

# Find Qt
if(USE_QT5)
	find_package(Qt5Widgets REQUIRED)
	set(QT_LIBRARIES ${Qt5Widgets_LIBRARIES})
else()
	find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
	include(${QT_USE_FILE})
endif()

add_definitions(-DQT_NO_KEYWORDS)

include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_editor
  #CATKIN_DEPENDS other_catkin_pkg
  #DEPENDS system_lib
)

set(UI_FILES
	include/trajectory_editor/ui/mainwindow_2.ui
	include/trajectory_editor/ui/headerview.ui
	include/trajectory_editor/ui/frameview.ui
	include/trajectory_editor/ui/ruleview.ui
	include/trajectory_editor/ui/controls.ui
	include/trajectory_editor/ui/about.ui
)
set(H_FILES
	include/trajectory_editor/mainwindow.h
	include/trajectory_editor/rvizwidget.h
	include/trajectory_editor/keyframemodel.h
	include/trajectory_editor/headerview.h
	include/trajectory_editor/frameview.h
	include/trajectory_editor/historykeeper.h
	include/trajectory_editor/savecontroller.h
	include/trajectory_editor/recentfiles.h
	include/trajectory_editor/ruleview.h
	include/trajectory_editor/ruleslider.h
	include/trajectory_editor/spaces/jointdisplay.h
	include/trajectory_editor/spaces/abstractspace.h
	include/trajectory_editor/spaces/inversespace.h
	include/trajectory_editor/spaces/pidspace.h
	include/trajectory_editor/spaces/posveleffview.h
	include/trajectory_editor/spaces/rateangleview.h
	include/trajectory_editor/spaces/positionview.h
	include/trajectory_editor/spaces/pidview.h
	include/trajectory_editor/spaces/basicsmallview.h
	include/trajectory_editor/spaces/basicspace.h
)

if(USE_QT5)
	qt5_wrap_ui(UIC_FILES ${UI_FILES})
	qt5_wrap_cpp(MOC_FILES ${H_FILES})
else()
	qt4_wrap_ui(UIC_FILES ${UI_FILES})
	qt4_wrap_cpp(MOC_FILES ${H_FILES})
endif()

add_executable(trajectory_editor
	${UIC_FILES}
	${MOC_FILES}
	src/main.cpp
	src/mainwindow.cpp
	src/rvizwidget.cpp
	src/robotdisplay.cpp
	src/keyframemodel.cpp
	src/jointdisplay.cpp
	src/headerview.cpp
	src/posveleffview.cpp
	src/frameview.cpp
	src/abstractspace.cpp
	src/rateangleview.cpp
	src/inversespace.cpp
	src/positionview.cpp
	src/historykeeper.cpp
	src/basicsmallview.cpp
	src/mymarkerdisplay.cpp
	src/pidspace.cpp
	src/pidview.cpp
	src/motionplayerclient.cpp
	src/basicspace.cpp
	src/savecontroller.cpp
	src/jointperspective.cpp
	src/recentfiles.cpp
	src/ruleview.cpp
	src/ruleslider.cpp
)

target_link_libraries(trajectory_editor
	${catkin_LIBRARIES}
	${rviz_DEFAULT_PLUGIN_LIBRARIES}
	${QT_LIBRARIES}
	yaml-cpp
)

install(TARGETS trajectory_editor
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
