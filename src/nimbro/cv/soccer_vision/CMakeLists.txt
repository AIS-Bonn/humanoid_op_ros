cmake_minimum_required(VERSION 2.8.3)
project(soccer_vision)

find_package(catkin REQUIRED COMPONENTS roscpp tf image_transport message_generation field_model config_server cmake_modules)

add_message_files(
  FILES
  Detections.msg
  LineDetection.msg
  ObjectDetection.msg
  ObstacleDetection.msg
)

generate_messages(
	DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

find_package(GTK2)
if(GTK2_FOUND)
	include_directories(${GTK2_INCLUDE_DIRS})
	add_definitions(-DHAVE_GTK)
endif()


find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})


find_package(OpenCV REQUIRED)


add_executable(soccercv_gui src/soccercv_gui.cpp)
target_link_libraries(soccercv_gui ${catkin_LIBRARIES} ${GTK2_LIBRARIES} ${OpenCV_LIBS})


install(FILES Black.lut Objects.lut White.lut maskForCamera plugin_nodelet.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_library(soccer_vision
	src/soccer_vision.cpp
	src/yuvClasses.cpp 
	src/convexhullfunctions.cpp 
	src/pixelCameraCorrection.cpp  
	src/camera_parameters.cpp  
	src/lsd.c
	src/findField.cpp  
	src/findBall.cpp 
	src/findGoal.cpp 
	src/findObstacles.cpp 
	src/findLandmarksAndLines.cpp
	src/Vec2f.cpp
	src/Vec2i.cpp
	src/RcAngle.cpp
	src/nimbroStyleTools.cpp
	src/findLinesNimbroStyle.cpp)
			
target_link_libraries(soccer_vision ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(soccer_vision soccer_vision_generate_messages_cpp)

install(TARGETS soccer_vision
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
