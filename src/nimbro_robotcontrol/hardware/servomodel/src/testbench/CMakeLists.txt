
# Decide on a Qt version
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
	set(USE_QT5 true)
else()
	set(USE_QT5 false)
endif()

# Find Qt
if(USE_QT5)
	find_package(Qt5Widgets REQUIRED)
	set(QT_LIBRARIES ${Qt5Widgets_LIBRARIES})
	message(WARNING "Found Qt5")
else()
	find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
	include(${QT_USE_FILE})
	message(WARNING "Found Qt4")
endif()

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

set(UI_FILES
	singleservotestbench.ui
	testbench.ui
	statisticswidget.ui
)
set(H_FILES
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

if(USE_QT5)
	qt5_wrap_ui(UIC_FILES ${UI_FILES})
	qt5_wrap_cpp(MOC_FILES ${H_FILES})
else()
	qt4_wrap_ui(UIC_FILES ${UI_FILES})
	qt4_wrap_cpp(MOC_FILES ${H_FILES})
endif()

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
	${UIC_FILES}
	${MOC_FILES}
)

target_link_libraries(testbench
	${QT_LIBRARIES}
	${DYNAMIXEL_LIB}
	${catkin_LIBRARIES}
	servomodel
)

