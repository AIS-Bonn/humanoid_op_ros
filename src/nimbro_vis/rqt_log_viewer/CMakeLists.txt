
cmake_minimum_required(VERSION 2.8)

project(rqt_log_viewer)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rosgraph_msgs
	rqt_gui_cpp
)

include_directories(${catkin_INCLUDE_DIRS})

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtWebKit QtDBus)
include(${QT_USE_FILE})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

include_directories(${CMAKE_CURRENT_BINARY_DIR} src)

catkin_package()

qt4_wrap_ui(UI_HDRS
	src/rqt_log_viewer.ui
)

qt4_wrap_cpp(MOC_SRCS
	src/log_model.h
	src/rqt_log_viewer.h
	src/log_view.h
	src/node_model.h
	src/checkboxdelegate.h
	src/log_filter.h
	src/node_backend.h
	src/filter_line_edit.h
	src/message_display.h
	src/combobox_delegate.h
	src/editors/kdevelop.h
)

add_library(rqt_log_viewer
	${MOC_SRCS}
	${UI_HDRS}
	src/rqt_log_viewer.cpp
	src/log_model.cpp
	src/log_view.cpp
	src/node_model.cpp
	src/checkboxdelegate.cpp
	src/log_filter.cpp
	src/node_backend.cpp
	src/filter_line_edit.cpp
	src/message_display.cpp
	src/combobox_delegate.cpp
	src/editors/kdevelop.cpp
)
target_link_libraries(rqt_log_viewer
	${catkin_LIBRARIES}
	${QT_LIBRARIES}
)

install(FILES plugin_rqt_gui.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
