cmake_minimum_required(VERSION 2.8.3)
project(hash_library)

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(include)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hash_library
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_library(hash_library 
	src/md5.cpp
	src/crc32.cpp
	src/keccak.cpp
	src/sha1.cpp
	src/sha3.cpp
	src/sha256.cpp)

install(TARGETS hash_library
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
