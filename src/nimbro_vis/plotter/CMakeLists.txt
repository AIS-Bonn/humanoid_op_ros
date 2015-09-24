cmake_minimum_required(VERSION 2.8.3)
project(plotter)

find_package(catkin REQUIRED COMPONENTS roscpp rqt_gui plot_msgs rosbag config_server)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

qt4_wrap_cpp(
	MOC_FILES
	include/plotter/plot.h
	include/plotter/plotter.h
	include/plotter/plotwidget.h
	include/plotter/plots/jointstateplot.h
	include/plotter/plotmodel.h
	include/plotter/utils/checkboxdelegate.h
	include/plotter/utils/colordelegate.h
	include/plotter/plotfiltermodel.h
	include/plotter/io/plotio.h
)

qt4_wrap_ui(
	UI_FILES
	src/plotter.ui
)

catkin_package(
	INCLUDE_DIRS include
)

include_directories(include
	${catkin_INCLUDE_DIRS}
)

add_library(plotter
	${UI_FILES}
	${MOC_FILES}
	src/plotter.cpp
	src/plot.cpp
	src/plotwidget.cpp
	src/plots/jointstateplot.cpp
	src/plotmodel.cpp
	src/utils/checkboxdelegate.cpp
	src/utils/colordelegate.cpp
	src/plotfiltermodel.cpp
	src/io/bagformat.cpp
	src/io/csvformat.cpp
	src/io/plotio.cpp
)

target_link_libraries(plotter
	${catkin_LIBRARIES}
	${QT_LIBRARIES}
)

add_dependencies(plotter plotter_gencpp ${catkin_EXPORTED_TARGETS})

install(TARGETS plotter
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_rqt_gui.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
