cmake_minimum_required(VERSION 2.8.4)
project(scripts)

catkin_package()

install(PROGRAMS changeRobot.py filecheck.py set_camera.py set_cm7X0.py set_netUdev.py fix_ssh_speed.py dynarecover.py install_openfec.sh
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Version 1.4 (increment this comment to trigger a CMake update)
catkin_add_env_hooks(50-common-funcs SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
# EOF
