cmake_minimum_required(VERSION 2.8.3)
project(rqt_vision_module)
find_package(catkin REQUIRED COMPONENTS std_msgs rqt_gui rqt_gui_cpp image_transport sensor_msgs cv_bridge message_generation)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

qt4_wrap_cpp(MOCFILES 
   include/${PROJECT_NAME}/main.hpp
   include/${PROJECT_NAME}/layout.hpp
)
qt4_wrap_ui(UIFILES 
   src/${PROJECT_NAME}/main.ui
)
add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/main.cpp
   src/${PROJECT_NAME}/layout.cpp
   ${MOCFILES}
   ${UIFILES}
)
include_directories(include 
   ${catkin_INCLUDE_DIRS} 
   ${CMAKE_CURRENT_BINARY_DIR} 
   ${CMAKE_CURRENT_SOURCE_DIR}
   )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})

catkin_python_setup()
   
add_message_files(DIRECTORY msg FILES
	GuiEvent.msg
	ConsoleMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
   
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS rqt_gui rqt_gui_cpp image_transport sensor_msgs cv_bridge message_runtime
)
find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/${PROJECT_NAME}
   DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


