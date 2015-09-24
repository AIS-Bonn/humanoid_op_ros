cmake_minimum_required(VERSION 2.8.3)
project(display_image)

find_package(catkin REQUIRED COMPONENTS roscpp nodelet nodelet_gui_manager)

find_package(Qt4 COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

qt4_wrap_cpp(MOC_SRCS
	imagewidget.h
)

catkin_package(
	INCLUDE_DIRS .
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(imagewidget
	imagewidget.cpp
	${MOC_SRCS}
)

add_library(display_image
	display_image.cpp
)

target_link_libraries(display_image
	${catkin_LIBRARIES}
	${QT_LIBRARIES}
	imagewidget
)

install(FILES plugin_nodelet.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
