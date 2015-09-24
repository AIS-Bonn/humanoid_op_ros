cmake_minimum_required(VERSION 2.8.3)
project(limb_control)

find_package(catkin REQUIRED COMPONENTS
	roscpp robotcontrol message_generation
)

add_message_files(DIRECTORY msg FILES
	LimbCommand.msg
)

add_service_files(DIRECTORY srv FILES
	PlayCommands.srv
)

generate_messages(DEPENDENCIES
	std_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES limb_control
	CATKIN_DEPENDS message_runtime
)

include_directories(include
	${catkin_INCLUDE_DIRS}
)

add_library(limb_control
	src/limb_control.cpp
)

target_link_libraries(limb_control
	${catkin_LIBRARIES}
)

add_dependencies(limb_control limb_control_gencpp)
add_dependencies(limb_control ${catkin_EXPORTED_TARGETS})

install(TARGETS limb_control
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_robotcontrol.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
# EOF