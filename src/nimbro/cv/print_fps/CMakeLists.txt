cmake_minimum_required(VERSION 2.8.3)
project(print_fps)

find_package(catkin REQUIRED COMPONENTS roscpp nodelet sensor_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_library(print_fps
	print_fps.cpp
)
target_link_libraries(print_fps
	${catkin_LIBRARIES}
)

install(TARGETS print_fps
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_nodelet.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
