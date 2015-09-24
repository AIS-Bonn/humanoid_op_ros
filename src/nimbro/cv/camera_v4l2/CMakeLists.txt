cmake_minimum_required(VERSION 2.8.3)
project(camera_v4l2)

find_package(catkin REQUIRED COMPONENTS roscpp nodelet pluginlib genmsg)

add_message_files(
	FILES
	CameraParam.msg
	CameraParamChoice.msg
)

add_service_files(
	FILES
	EnumerateCameraParams.srv
	SetCameraParam.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs  # Or other packages containing msgs
)

catkin_package(
	LIBRARIES camera_v4l2
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(camera_v4l2
	camera_v4l2.cpp
	configmanager.cpp
)

target_link_libraries(camera_v4l2
	${catkin_LIBRARIES}
	yaml-cpp
)

add_dependencies(camera_v4l2 camera_v4l2_gencpp)

install(TARGETS camera_v4l2
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(CODE "file(REMOVE_RECURSE \"${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/launch\")")
install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin_nodelet.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
