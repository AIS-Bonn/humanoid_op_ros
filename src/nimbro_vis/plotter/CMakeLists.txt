cmake_minimum_required(VERSION 2.8.3)
project(plotter)

find_package(catkin REQUIRED COMPONENTS roscpp rqt_gui rqt_gui_cpp plot_msgs rosbag config_server timewarp)

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
else()
	find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
	include(${QT_USE_FILE})
endif()

include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(UI_FILES
	src/plotter.ui
)
set(H_FILES
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

if(USE_QT5)
	qt5_wrap_ui(UIC_FILES ${UI_FILES})
	qt5_wrap_cpp(MOC_FILES ${H_FILES})
else()
	qt4_wrap_ui(UIC_FILES ${UI_FILES})
	qt4_wrap_cpp(MOC_FILES ${H_FILES})
endif()

catkin_package(
	INCLUDE_DIRS include
)

include_directories(include
	${catkin_INCLUDE_DIRS}
)

add_library(plotter
	${UIC_FILES}
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

add_dependencies(plotter ${catkin_EXPORTED_TARGETS})

install(TARGETS plotter
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_rqt_gui.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
