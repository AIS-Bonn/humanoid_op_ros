cmake_minimum_required(VERSION 2.8)
project(rviz_config_injector)

find_package(catkin REQUIRED COMPONENTS rqt_gui_cpp)

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

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_library(rviz_config_injector
	src/rviz_config_injector.cpp
)
target_link_libraries(rviz_config_injector
	${catkin_LIBRARIES}
	${QT_LIBRARIES}
)

install(TARGETS rviz_config_injector
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
