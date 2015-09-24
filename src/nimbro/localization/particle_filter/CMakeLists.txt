
cmake_minimum_required(VERSION 2.8)
project(particle_filter)

find_package(catkin REQUIRED COMPONENTS
	roscpp field_model tf config_server soccer_vision eigen_conversions
	genmsg robotcontrol cmake_modules
)

find_package(Eigen REQUIRED)

find_package(OpenCV REQUIRED)


find_library(GSL_LIBRARY
	NAMES gsl libgsl
	PATHS  ${GSL_POSSIBLE_ROOT_DIRS}
	PATH_SUFFIXES lib
	DOC "GSL library"
)

FIND_LIBRARY(GSLCBLAS_LIBRARY
	NAMES gslcblas libgslcblas
	PATHS  ${GSL_POSSIBLE_ROOT_DIRS}
	PATH_SUFFIXES lib
	DOC "GSL cblas library"
)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_message_files(
	FILES
	Particle.msg
	ParticleSet.msg
)

generate_messages(
	DEPENDENCIES geometry_msgs
)

catkin_package()

add_executable(particle_filter
	src/localization_pf.cpp
)

target_link_libraries(particle_filter
	${catkin_LIBRARIES}
	${GSL_LIBRARY}
	${GSLCBLAS_LIBRARY}
)

add_dependencies(particle_filter particle_filter_gencpp)

add_executable(particleFilterTester
	src/particleFilterTester.cpp
)
target_link_libraries(particleFilterTester ${catkin_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS particle_filter
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
