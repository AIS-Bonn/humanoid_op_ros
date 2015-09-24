cmake_minimum_required(VERSION 2.8.3)
project(head_control)

find_package(catkin REQUIRED COMPONENTS roscpp robotcontrol genmsg config_server nimbro_utils)

add_message_files(
	FILES
	LookAtTarget.msg
	HeadControlStatus.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

catkin_package(
)

include_directories(include
	${catkin_INCLUDE_DIRS}
)

add_library(head_control
	src/headcontrol.cpp
)

target_link_libraries(head_control
	${catkin_LIBRARIES}
)

add_dependencies(head_control head_control_gencpp)

install(TARGETS head_control
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_robotcontrol.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)