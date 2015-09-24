
find_library(READLINE_LIBRARY readline)
find_path(READLINE_INCLUDE_DIR readline/readline.h)

find_package(catkin REQUIRED)

if(READLINE_LIBRARY AND READLINE_INCLUDE_DIR)
	include_directories(${READLINE_INCLUDE_DIR} "${device_header_dest}/../..")

	add_executable(dynatool
		dynatool.cpp
	)
	target_link_libraries(dynatool
		dynalib
		dynalib_devices
		${READLINE_LIBRARY}
		rt
	)
	add_dependencies(dynatool
		dynalib
		dynalib_devices
	)
	install(TARGETS dynatool
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
endif()