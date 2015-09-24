
cmake_minimum_required(VERSION 2.8)

project(dynalib)

execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -dumpversion
        OUTPUT_VARIABLE GCC_VERSION
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
	add_definitions(-DYAML_NEW_API)
	set(YAML_NEW_API ON)
endif(NEW_YAMLCPP_FOUND)

if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
	set(CMAKE_CXX_FLAGS "-Wall -std=c++11")

	find_package(catkin REQUIRED)

	catkin_package(
		LIBRARIES dynalib dynalib_devices
		INCLUDE_DIRS include
	)

	set(device_header_dest "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/devices")
	set(device_impl_dest "${CMAKE_CURRENT_BINARY_DIR}/devices")
	file(MAKE_DIRECTORY ${device_header_dest})
	file(MAKE_DIRECTORY ${device_impl_dest})

	include_directories(include)

	macro(dynalib_compile_device out_headers out_sources name)
		add_custom_command(
			OUTPUT "${device_impl_dest}/${name}.cpp" "${device_header_dest}/${name}.h"
			DEPENDS "devices/${name}.device" "${CMAKE_CURRENT_SOURCE_DIR}/util/dynalib_dc.py"
			COMMAND python "${CMAKE_CURRENT_SOURCE_DIR}/util/dynalib_dc.py" "${CMAKE_CURRENT_SOURCE_DIR}/devices/${name}.device" "${device_header_dest}/${name}.h" "${device_impl_dest}/${name}.cpp"
		)
		list(APPEND ${out_headers} "${device_header_dest}/${name}.h")
		list(APPEND ${out_sources} "${device_impl_dest}/${name}.cpp")
	endmacro()

	dynalib_compile_device(DEV_HEADERS DEV_SOURCES generic)
	dynalib_compile_device(DEV_HEADERS DEV_SOURCES MX28)
	dynalib_compile_device(DEV_HEADERS DEV_SOURCES MX64)
	dynalib_compile_device(DEV_HEADERS DEV_SOURCES MX106)
	dynalib_compile_device(DEV_HEADERS DEV_SOURCES RX28)
	dynalib_compile_device(DEV_HEADERS DEV_SOURCES RX64)
	dynalib_compile_device(DEV_HEADERS DEV_SOURCES EX106P)
	dynalib_compile_device(DEV_HEADERS DEV_SOURCES CM730)
	dynalib_compile_device(DEV_HEADERS DEV_SOURCES CM730_NOP)
	dynalib_compile_device(DEV_HEADERS DEV_SOURCES CM740)
	dynalib_compile_device(DEV_HEADERS DEV_SOURCES CM740_NOP)
	dynalib_compile_device(DEV_HEADERS DEV_SOURCES USB2AX)


	add_library(dynalib
		${DEV_HEADERS}
		src/io.cpp
		src/impl/linux_serial.cpp
		src/impl/debug_io.cpp
		src/setup.cpp
		src/device_setup.cpp
	)

	add_library(dynalib_devices SHARED
		${DEV_SOURCES}
		src/device_registry.cpp
	)

	target_link_libraries(dynalib
		yaml-cpp
	)

	add_subdirectory(tools)

	install(TARGETS dynalib dynalib_devices
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
	
	install(CODE "file(REMOVE_RECURSE \"${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/robots\")")
	install(DIRECTORY robots
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	)
else()
	message(WARNING "Your gcc version is too old. Not building dynalib")
	message(WARNING "gcc: ${CMAKE_CXX_COMPILER}")
	message(WARNING "Version: ${GCC_VERSION}")
endif()

# Version 1.0 (increment this comment to trigger a CMake update)
catkin_add_env_hooks(50-dynatool SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
