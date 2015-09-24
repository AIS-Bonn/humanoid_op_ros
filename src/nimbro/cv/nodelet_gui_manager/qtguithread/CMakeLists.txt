
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

rosbuild_add_library( qtguithread SHARED qtguithread.cpp ${QTGUITHREAD_MOC_SRCS} )
