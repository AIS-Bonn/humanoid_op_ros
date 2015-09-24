cmake_minimum_required(VERSION 2.8.3)
project(servomodel)

set(SERVOMODEL_TESTBENCH OFF CACHE BOOL "Build the servomodel testbench")

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs kdl_parser)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES servomodel
   CATKIN_DEPENDS roscpp std_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)
