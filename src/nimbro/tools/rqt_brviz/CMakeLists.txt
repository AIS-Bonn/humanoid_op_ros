cmake_minimum_required(VERSION 2.8.3)
project(rqt_brviz)
# Load catkin and all dependencies required for this package

find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp rviz)
catkin_package(
	INCLUDE_DIRS ${rqt_brviz_INCLUDE_DIRECTORIES}
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS rqt_gui rqt_gui_cpp rviz
)
catkin_python_setup()

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

set(rqt_brviz_SRCS
  src/rqt_brviz/brviz.cpp
)

set(rqt_brviz_HDRS
  include/rqt_brviz/brviz.h
)

qt4_wrap_cpp(rqt_brviz_MOCS ${rqt_brviz_HDRS})

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_brviz_SRCS} ${rqt_brviz_MOCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${Boost_LIBRARIES})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(PROGRAMS scripts/rqt_brviz
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
