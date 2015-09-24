cmake_minimum_required(VERSION 2.8.3)
project(nodelet_gui_manager)

find_package(catkin REQUIRED COMPONENTS roscpp nodelet)

catkin_package(
	INCLUDE_DIRS .
)

find_package(Qt4 COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

include_directories(. ${catkin_INCLUDE_DIRS})

add_executable(nodelet_gui_manager
	nodelet_gui_manager.cpp
	qtguithread/qtguithread.cpp
)

target_link_libraries(nodelet_gui_manager
	${catkin_LIBRARIES}
	${QT_LIBRARIES}
)

add_library(qtguithread
	qtguithread/qtguithread.cpp
)

install(TARGETS qtguithread nodelet_gui_manager
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
