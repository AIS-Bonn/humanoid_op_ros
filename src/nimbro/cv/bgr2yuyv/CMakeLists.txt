cmake_minimum_required(VERSION 2.8.3)
project(bgr2yuyv)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs image_geometry image_transport  cv_bridge )
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system timer)

catkin_package(
)

include_directories(include 
  ${catkin_INCLUDE_DIRS}
   ${OpenCV_INCLUDE_DIRS}
)

add_executable(bgr2yuyv_node
   src/${PROJECT_NAME}/main.cpp
   src/${PROJECT_NAME}/CameraDummy.cpp
)

target_link_libraries(bgr2yuyv_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
 )

install(TARGETS bgr2yuyv_node
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
