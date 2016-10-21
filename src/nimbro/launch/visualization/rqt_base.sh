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
if [[ ! -f "$DIRR" ]]; then
	echo -e "\e[33m[ WARN][/rqt_base->rqt_base.sh] $HOST.rviz was not found!\e[39m"
	rosparam set "/brviz_path" "."
else
	echo "[ INFO][/rqt_base->rqt_base.sh] Loading rviz configuration file for $HOST..."
	rosparam set "/brviz_path" "$DIRR"
fi

# Launch rqt with the required perspective
if [[ ! -f "$DIRP" ]]; then
	echo -e "\e[33m[ WARN][/rqt_base->rqt_base.sh] $HOST.perspective was not found!\e[39m"
	rqt
else
	echo "[ INFO][/rqt_base->rqt_base.sh] Loading perspective for $HOST..."
	rqt --perspective-file "$DIRP"
# 	gdb -ex run --args /usr/bin/python "$NIMBRO_ROOT/devel/bin/rqt" --perspective-file "$DIRP" # Uncomment this line to run rqt with gdb...
fi
# EOF