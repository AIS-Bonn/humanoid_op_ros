cmake_minimum_required(VERSION 2.8.3)
project(vision_module)

add_definitions(-std=c++11)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake) # For finding custom .cmake files
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs gait_msgs plot_msgs image_geometry image_transport cv_bridge config_server tf vis_utils nav_msgs head_control rqt_vision_module field_model tf_tools pcl_conversions pcl_ros)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system chrono timer thread date_time program_options filesystem)
find_package(PkgConfig REQUIRED)
find_package(G2O REQUIRED)
pkg_check_modules(gsl REQUIRED gsl)


add_message_files(
	FILES
	ball_output.msg
	field_output.msg
	goal_output.msg
	line_output.msg
	obstacle_output.msg
	vision_outputs.msg
	localization_output.msg
)
 
generate_messages(
	DEPENDENCIES std_msgs geometry_msgs
)

include_directories(include 
   ${catkin_INCLUDE_DIRS}
   ${OpenCV_INCLUDE_DIRS}
   ${G2O_INCLUDE_DIR}
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES vision_module
   CATKIN_DEPENDS pcl_conversions pcl_ros
)

add_library(vision_module
   src/${PROJECT_NAME}/Tools/General.cpp
   src/${PROJECT_NAME}/Tools/LineSegment.cpp
   src/${PROJECT_NAME}/Tools/Parameters.cpp
   src/${PROJECT_NAME}/Tools/MatPublisher.cpp
   src/${PROJECT_NAME}/Tools/HSVPresenter.cpp
   src/${PROJECT_NAME}/Tools/SortFunctions.cpp
   src/${PROJECT_NAME}/Tools/VisionRate.cpp
   src/${PROJECT_NAME}/Tools/Kalman.cpp
   src/${PROJECT_NAME}/Tools/GuiManager.cpp
   src/${PROJECT_NAME}/Projections/IPM.cpp
   src/${PROJECT_NAME}/Projections/CameraProjections.cpp
   src/${PROJECT_NAME}/Projections/DistortionModel.cpp
   src/${PROJECT_NAME}/Inputs/CameraDummy.cpp
   src/${PROJECT_NAME}/Inputs/Camera.cpp   
   src/${PROJECT_NAME}/Localization/Localization.cpp   
   src/${PROJECT_NAME}/SoccerObjects/FieldDetector.cpp
   src/${PROJECT_NAME}/SoccerObjects/BallDetector.cpp
   src/${PROJECT_NAME}/SoccerObjects/GoalDetector.cpp
   src/${PROJECT_NAME}/SoccerObjects/LineDetector.cpp
   src/${PROJECT_NAME}/SoccerObjects/CircleDetector.cpp
   src/${PROJECT_NAME}/SoccerObjects/ObstacleDetector.cpp
)


target_link_libraries(vision_module
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
   ${gsl_LIBRARIES}
   ${G2O_LIBRARIES}
 )
 
add_dependencies(vision_module vision_module_gencpp ${catkin_EXPORTED_TARGETS})

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
   src/${PROJECT_NAME}/Calibration/CameraCalibration.cpp
)

target_link_libraries(camera_calibration_node
   vision_module
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
   ${gsl_LIBRARIES}
 )
 
## Test General
catkin_add_gtest(test_general
	test/test_general.cpp
)
target_link_libraries(test_general
	vision_module
)
## Test LineSegment
catkin_add_gtest(test_linesegment
	test/test_linesegment.cpp
)
target_link_libraries(test_linesegment
	vision_module
)
## Test LinearInterpolator
catkin_add_gtest(test_linearInterpolator
	test/test_linearInterpolator.cpp
)
target_link_libraries(test_linearInterpolator
	vision_module
)


install(TARGETS vision_module vision_module_node camera_calibration_node 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
        
install(PROGRAMS scripts/watch_vision.py scripts/camera_publisher.py scripts/face_tracker.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

