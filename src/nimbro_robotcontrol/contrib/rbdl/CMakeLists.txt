

cmake_minimum_required(VERSION 2.8.3)
project(rbdl)

find_package(catkin REQUIRED COMPONENTS urdf)

catkin_package(
	INCLUDE_DIRS include source/include ${EIGEN3_INCLUDE_DIR}
	LIBRARIES rbdl_ros rbdl
)

add_subdirectory(source)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/source/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/source/include)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/source/CMake )

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Copy rbdl_config.h to devel prefix to comply with ROS standards
add_custom_command(
	OUTPUT "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/rbdl_config.h"
	DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/source/include/rbdl/rbdl_config.h"
	COMMAND ${CMAKE_COMMAND} -E copy
		"${CMAKE_CURRENT_BINARY_DIR}/source/include/rbdl/rbdl_config.h"
		"${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/rbdl_config.h"
)

add_library(rbdl_ros
	"${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/rbdl_config.h"
	src/rbdl_parser.cpp
	src/treestream.cpp
)
target_link_libraries(rbdl_ros
	rbdl
)

add_executable(dump
	src/dump.cpp
)
target_link_libraries(dump
	rbdl_ros
	${catkin_LIBRARIES}
)

install(TARGETS rbdl_ros
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS dump
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Add gtest based cpp test target and link libraries

configure_file(test/test_config.h.in test/test_config.h @ONLY)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/test)

catkin_add_gtest(test_rbdl_parser
	test/test_rbdl_parser.cpp
)
target_link_libraries(test_rbdl_parser
	${catkin_LIBRARIES}
	rbdl_ros
	rbdl
)
