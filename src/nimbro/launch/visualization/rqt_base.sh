#!/bin/bash
# Launch script for rqt that manages perspectives and RViz configuration files.

# Retrieve the computer hostname
HOST="$(hostname)"
if [[ ${#1} -ge 3 ]]; then
	HOST="$1"
fi

# Construct the expected configuration file paths
DIRR="$(catkin_find launch visualization)/perspectives/$HOST.rviz"
DIRP="$(catkin_find launch visualization)/perspectives/$HOST.perspective"

# Configure RViz as required
if [[ -f "$DIRR" ]]; then
	echo "[ INFO][/rqt_base->rqt_base.sh] Loading RViz configuration file for $HOST..."
	export RVIZ_DISPLAY_CONFIG_FILE="$DIRR"
else
	echo -e "\e[33m[ WARN][/rqt_base->rqt_base.sh] $HOST.rviz was not found!\e[39m"
	export RVIZ_DISPLAY_CONFIG_FILE=
fi

# Preload the required injector libraries
export LD_PRELOAD="$(rospack libs-only-L rqt_arg_injector)/librqt_arg_injector.so $(rospack libs-only-L rviz_config_injector)/librviz_config_injector.so"

# Launch rqt with the required perspective
if [[ -n "$LV_GDB" ]]; then
	PREFIX="gdb -ex run --args /usr/bin/python"
else
	PREFIX=
fi
RQT_PATH="$(which rqt)"
if [[ -f "$DIRP" ]]; then
	echo "[ INFO][/rqt_base->rqt_base.sh] Loading perspective for $HOST..."
	$PREFIX "$RQT_PATH" --perspective-file "$DIRP"
else
	echo -e "\e[33m[ WARN][/rqt_base->rqt_base.sh] $HOST.perspective was not found!\e[39m"
	$PREFIX "$RQT_PATH"
fi
# EOF
