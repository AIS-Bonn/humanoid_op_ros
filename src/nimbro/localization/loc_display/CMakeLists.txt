cmake_minimum_required(VERSION 2.8.3)
project(loc_display)

find_package(catkin REQUIRED COMPONENTS roscpp rviz field_model)

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)

catkin_package()

include_directories(include
	${catkin_INCLUDE_DIRS}
)

add_library(loc_display
	src/loc_display.cpp
)

target_link_libraries(loc_display
	${catkin_LIBRARIES}
	${QT_LIBRARIES}
)

install(TARGETS loc_display
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_rviz.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
