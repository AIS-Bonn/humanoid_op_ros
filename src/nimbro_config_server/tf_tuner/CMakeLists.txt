
cmake_minimum_required(VERSION 2.8)
project(tf_tuner)

find_package(catkin REQUIRED COMPONENTS
	config_server
	tf
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(tf_tuner
	src/tf_tuner.cpp
)
target_link_libraries(tf_tuner
	${catkin_LIBRARIES}
)
