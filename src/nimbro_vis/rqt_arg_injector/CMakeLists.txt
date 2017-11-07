
cmake_minimum_required(VERSION 2.8)
project(rqt_arg_injector)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_library(rqt_arg_injector
	src/rqt_arg_injector.cpp
)
target_link_libraries(rqt_arg_injector
	${catkin_LIBRARIES}
)

install(TARGETS rqt_arg_injector
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
