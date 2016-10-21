
cmake_minimum_required(VERSION 2.8)
project(catch_ros)

find_package(catkin REQUIRED COMPONENTS
	roscpp
)

find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
	INCLUDE_DIRS include
	CFG_EXTRAS catch.cmake
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_library(catch_ros_standalone
	src/standalone_main.cpp
)
target_link_libraries(catch_ros_standalone
	${Boost_LIBRARIES}
)

add_library(catch_ros_rostest
	src/rostest_main.cpp
)
target_link_libraries(catch_ros_rostest
	${Boost_LIBRARIES}
)

install(FILES src/meta_info.cpp DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS catch_ros_standalone catch_ros_rostest
	LIBRARY DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
)
