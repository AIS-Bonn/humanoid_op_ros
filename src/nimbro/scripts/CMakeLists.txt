cmake_minimum_required(VERSION 2.8.3)
project(scripts)

catkin_package()

install(PROGRAMS wifi.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Version 1.1 (increment this comment to trigger a CMake update)
catkin_add_env_hooks(50-common-funcs SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
# EOF
