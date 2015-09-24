cmake_minimum_required(VERSION 2.8.3)
project(fall_protection)

find_package(catkin REQUIRED COMPONENTS robotcontrol roscpp)

catkin_package(
	INCLUDE_DIRS src
)

include_directories(include
	${catkin_INCLUDE_DIRS}
)

add_library(fall_protection
	src/fallprotection.cpp
)

target_link_libraries(fall_protection
	${catkin_LIBRARIES}
)

install(TARGETS fall_protection
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_robotcontrol.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
