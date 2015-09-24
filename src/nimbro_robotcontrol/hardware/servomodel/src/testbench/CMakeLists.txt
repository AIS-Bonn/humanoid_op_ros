
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

set(DYNAMIXEL_SDK "" CACHE PATH "Dynamixel SDK path")
find_path(DYNAMIXEL_INCLUDE "dynamixel.h" HINTS ${DYNAMIXEL_SDK}/include)
find_library(DYNAMIXEL_LIB "dxl" HINTS ${DYNAMIXEL_SDK}/lib)

message(STATUS "Using dynamixel SDK at ${DYNAMIXEL_SDK}")
message(STATUS "Include at ${DYNAMIXEL_INCLUDE}")
message(STATUS "Lib at ${DYNAMIXEL_LIB}")

if(NOT DYNAMIXEL_INCLUDE OR NOT DYNAMIXEL_LIB)
	message(FATAL_ERROR "Could not find Dynamixel SDK. Set DYNAMIXEL_SDK properly.")
endif()

find_path(EIGEN_INCLUDE "Eigen/Core" HINTS /usr/include/eigen3)
include_directories(${EIGEN_INCLUDE})

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(WRAP_CPP
	testbench.h
	singleservotestbench.h
	plot.h
	plotlegendwidget.h
	sinustrajectory.h
	statisticswidget.h
	logwidget.h
	ijointinterface.h
	trajectorypart.h
	trajectory.h
	valuelogger.h
	tunablecommandgenerator.h
	dynamicparameter.h
)

qt4_wrap_ui(WRAP_UI
	singleservotestbench.ui
	testbench.ui
	statisticswidget.ui
)

include_directories(${DYNAMIXEL_INCLUDE})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(testbench
	tunablecommandgenerator.cpp
	testbench.cpp
	singleservotestbench.cpp
	Keyframe.cpp
	KeyframePlayer.cpp
	main.cpp
	Vec2f.cpp
	StopWatch_linux.cpp
	ijointinterface.cpp
	dxljointinterface.cpp
	plot.cpp
	plotlegendwidget.cpp
	sinustrajectory.cpp
	statisticswidget.cpp
	positiontrajectory.cpp
	compositetrajectory.cpp
	logwidget.cpp
	trajectorypart.cpp
	commandfitter.cpp
	trajectory.cpp
	valuelogger.cpp
	dynamicparameter.cpp
	${WRAP_UI}
	${WRAP_CPP}
)

target_link_libraries(testbench
	${QT_LIBRARIES}
	${DYNAMIXEL_LIB}
	${catkin_LIBRARIES}
	servomodel
)

