
cmake_minimum_required(VERSION 2.8)
project(field_model)

find_package(catkin REQUIRED COMPONENTS roscpp config_server cmake_modules)

find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

catkin_package(
	INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
	LIBRARIES field_model
	CATKIN_DEPENDS config_server
)

add_library(field_model
	src/field_model.cpp
)

target_link_libraries(field_model
	${catkin_LIBRARIES}
)

install(TARGETS field_model
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
