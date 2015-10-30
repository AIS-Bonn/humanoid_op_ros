cmake_minimum_required(VERSION 2.8.3)
project(vision_module)


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs image_geometry image_transport cv_bridge config_server tf vis_utils nav_msgs head_control)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system chrono timer)
find_package( PkgConfig REQUIRED)
pkg_check_modules( gsl REQUIRED gsl )
catkin_package(
)

include_directories(include 
  ${catkin_INCLUDE_DIRS}
   ${OpenCV_INCLUDE_DIRS}
)

add_library(vision_module
   src/${PROJECT_NAME}/Tools/General.cpp
   src/${PROJECT_NAME}/Tools/LineSegment.cpp
   src/${PROJECT_NAME}/Tools/Parameters.cpp
   src/${PROJECT_NAME}/Tools/MatPublisher.cpp
   src/${PROJECT_NAME}/Tools/HSVPresenter.cpp
   src/${PROJECT_NAME}/Tools/SortFunctions.cpp
   src/${PROJECT_NAME}/Tools/VisionRate.cpp
   src/${PROJECT_NAME}/Projections/IPM.cpp
   src/${PROJECT_NAME}/Projections/CameraProjections.cpp
   src/${PROJECT_NAME}/Projections/DistortionModel.cpp
   src/${PROJECT_NAME}/Inputs/CameraDummy.cpp
   src/${PROJECT_NAME}/Inputs/Camera.cpp   
   src/${PROJECT_NAME}/soccer_objects/FieldDetector.cpp
   src/${PROJECT_NAME}/soccer_objects/BallDetector.cpp
   src/${PROJECT_NAME}/soccer_objects/GoalDetector.cpp
   src/${PROJECT_NAME}/soccer_objects/LineDetector.cpp
   src/${PROJECT_NAME}/soccer_objects/ObstacleDetector.cpp
)

target_link_libraries(vision_module
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
   ${gsl_LIBRARIES}
 )
 
add_dependencies(vision_module ${catkin_EXPORTED_TARGETS})

add_executable(vision_module_node
   src/${PROJECT_NAME}/Vision.cpp
   src/${PROJECT_NAME}/main.cpp
)
target_link_libraries(vision_module_node
   vision_module
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
   ${gsl_LIBRARIES}
 )

add_executable(camera_calibration_node 
   src/${PROJECT_NAME}/Calibration/camera_calibration.cpp
)
target_link_libraries(camera_calibration_node
   vision_module
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
   ${gsl_LIBRARIES}
 )

install(TARGETS vision_module vision_module_node camera_calibration_node 
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

