
cmake_minimum_required(VERSION 2.8)
project(remote_tuner)

find_package(catkin REQUIRED COMPONENTS
	config_server
	roscpp
	rqt_gui
	rqt_gui_cpp
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

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

set(UI_FILES
	src/remote_tuner.ui
)
set(H_FILES
	src/parameter_model.h
	src/remote_tuner.h
	src/parameter_item.h
)

if(USE_QT5)
	qt5_wrap_ui(UIC_FILES ${UI_FILES})
	qt5_wrap_cpp(MOC_FILES ${H_FILES})
else()
	qt4_wrap_ui(UIC_FILES ${UI_FILES})
	qt4_wrap_cpp(MOC_FILES ${H_FILES})
endif()

add_library(remote_tuner
	${MOC_FILES}
	${UIC_FILES}
	src/parameter_model.cpp
	src/remote_tuner.cpp
	src/parameter_item.cpp
)
target_link_libraries(remote_tuner
	${catkin_LIBRARIES}
	${QT_LIBRARIES}
)
