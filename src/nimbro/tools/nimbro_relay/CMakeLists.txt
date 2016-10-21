cmake_minimum_required(VERSION 2.8.3)
project(nimbro_relay)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(catkin COMPONENTS cpp_common message_generation rosconsole roscpp rostime std_msgs xmlrpcpp)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime rosconsole roscpp std_msgs xmlrpcpp
 )

add_executable(nimbro_relay src/relay.cpp)
target_link_libraries(nimbro_relay ${catkin_LIBRARIES})

install(TARGETS nimbro_relay
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
