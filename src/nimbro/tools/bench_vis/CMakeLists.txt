cmake_minimum_required(VERSION 2.8.3)
project(bench_vis)

find_package(catkin REQUIRED COMPONENTS roscpp timer config_server plot_msgs rc_utils rrlogger vis_utils walk_and_kick rot_conv)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(include
	${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(bench_vis
    src/bench_vis.cpp
)
target_link_libraries(bench_vis
    ${catkin_LIBRARIES}
    walk_and_kick_tc
)
add_dependencies(bench_vis ${catkin_EXPORTED_TARGETS})

install(TARGETS bench_vis
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
