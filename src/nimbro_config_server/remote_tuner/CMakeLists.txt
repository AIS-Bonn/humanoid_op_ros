
cmake_minimum_required(VERSION 2.8)
project(remote_tuner)

find_package(catkin REQUIRED COMPONENTS
	config_server
	roscpp
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

qt4_wrap_cpp(MOC_SRCS
	src/parameter_model.h
	src/remote_tuner.h
	src/parameter_item.h
)

qt4_wrap_ui(UI_HDRS
	src/remote_tuner.ui
)

add_library(remote_tuner
	${MOC_SRCS}
	${UI_HDRS}
	src/parameter_model.cpp
	src/remote_tuner.cpp
	src/parameter_item.cpp
)
target_link_libraries(remote_tuner
	${catkin_LIBRARIES}
	${QT_LIBRARIES}
)
