cmake_minimum_required(VERSION 2.8.3)
project(downscale)

find_package(catkin REQUIRED COMPONENTS roscpp nodelet sensor_msgs image_transport cv_bridge)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_library(downscale
	src/downscale.cpp
)

target_link_libraries(downscale
	${catkin_LIBRARIES}
)

install(TARGETS downscale
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_nodelet.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
