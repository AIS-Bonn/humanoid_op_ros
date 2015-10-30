#!/bin/bash

# Save the directory this script is located in
SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Find the parent directory that corresponds to the NimbRo-OP root
NIMBRO_ROOT="$SCRIPTS_DIR"
while [[ ! -e "$NIMBRO_ROOT/src/CMakeLists.txt" ]] && [[ ! -h "$NIMBRO_ROOT/src/CMakeLists.txt" ]]; do
	NIMBRO_ROOT=$(dirname "$NIMBRO_ROOT")
	if [[ "$NIMBRO_ROOT" == "/" ]]; then
		echo 'Warning: Could not find ROS catkin workspace!'
		return
	fi
done
export NIMBRO_ROOT

# Initialise other variables
INSTALLPATH="/nimbro"
BOT="xs4.local"

# Set up environment variables for catkin and ROS
source "$NIMBRO_ROOT/devel/setup.bash"

# Set additional model path for gazebo simulation
NIMBRO_MODEL_PATHS="$NIMBRO_ROOT/src/nimbro/hardware/nimbro_op_gazebo/models"
[[ "$GAZEBO_MODEL_PATH" != *"$NIMBRO_MODEL_PATHS"* ]] && export GAZEBO_MODEL_PATH="$NIMBRO_MODEL_PATHS:$GAZEBO_MODEL_PATH"

function gitpull() {
	if ! git pull --rebase; then
		echo "---"
		read -p "Did the pull get refused because of unstaged changes [y/N]? " response
		echo
		if [[ $response == "y" || $response == "Y" ]]; then
			echo "Yes: Ok, I'm temporarily stashing away those changes..."
			git status
			git stash save "Changes stashed automatically to allow a rebase" && {
				git pull --rebase
				git stash pop || echo "Couldn't pop the stashed changes - Please check 'git stash list'..."
				echo 'The stashed changes have been reapplied to the working directory!'
			}
		else
			echo "No: Ok, then please resolve the problem and try again."
		fi
	fi
}

function gitpush() {
	local LIGHT_GREEN="$(echo -e "\E[1;32m")"
	local NO_COLOUR="$(echo -e "\E[0m")"
	git --no-pager log @{upstream}..HEAD
	counts=($(git rev-list --left-right --count @{upstream}..HEAD))
	[[ "${counts[1]}" -gt 0 ]] && echo
	if [[ "${counts[0]}" -eq 0 ]]; then
		if [[ "${counts[1]}" -eq 0 ]]; then
			echo $LIGHT_GREEN"No commits to push."$NO_COLOUR
		else
			echo $LIGHT_GREEN"Ahead ${counts[1]} commits."$NO_COLOUR
		fi
	else
		echo $LIGHT_GREEN"Ahead ${counts[1]} commits, but behind ${counts[0]} commits."$NO_COLOUR
	fi
	[[ "${counts[1]}" -eq 0 ]] && return
	read -p "Git push [Y/n]? " response
	response="${response:0:1}"
	response="${response,,}"
	if [[ $response == "n" ]]; then
		echo "No: Ok, not pushing anything."
	else
		if ! git push "$@"; then
			echo "Git push was not successful."
		fi
	fi
}

function _fadetorque() {
	rostopic pub -1 /robotcontrol/fade_torque/goal robotcontrol/FadeTorqueActionGoal "{header: {stamp: now}, goal: {torque: $1}}"
}

function _headcontrol() {
	rostopic pub --once /robotcontrol/headcontrol/target head_control/LookAtTarget "{enabled: $1, is_angular_data: $2, is_relative: $3, vec: {x: $4, y: $5, z: $6}, pitchEffort: $7, yawEffort: $8}"
}

function _gaitcmd() {
	re1='^[-+]?[0-9]+(\.[0-9]*)?$'
	re2='^(true|false)$'
	if [[ "$1" =~ $re1 ]] && [[ "$2" =~ $re1 ]] && [[ "$3" =~ $re1 ]] && [[ "$4" =~ $re2 ]]; then
		rostopic pub -1 /gaitCommand gait_msgs/GaitCommand "{gcvX: $1, gcvY: $2, gcvZ: $3, walk: $4, motion: 0}"
	else
		echo "Walk parameter not recognised."
		echo "Usage: nimbro {control|ctrl} halt"
		echo "       nimbro {control|ctrl} walk gcvX gcvY gcvZ [true|false]"
	fi
}

function _opendoc() { # Pass the file path to open as the first and only parameter
	if [[ -f "$1" ]]; then
		if which xdg-open > /dev/null; then
			xdg-open "$1"
		elif which kde-open > /dev/null; then
			kde-open "$1"
		elif which gnome-open > /dev/null; then
			gnome-open "$1"
		else
			echo "Could not detect the web browser to open the documentation with."
		fi
	fi
}

function _gendoc() { # Pass the file path of the generate script as the first parameter, add "-v" as the second parameter to make the generation verbose
	if [[ -x "$1" ]]; then
		if [[ "$2" == "-v" ]]; then
			"$1"
		else
			"$1" | grep warning || true
		fi
	fi
}

function _makefirmware() { # Pass the device name as the first parameter
	echo "$PATH" | grep -q "/opt/gcc-arm/bin" || PATH="$PATH:/opt/gcc-arm/bin"
	make clean
	failed="false"
	if [[ -z "$1" ]]; then
		make || failed="true"
	else
		make "DEVICE=$1" || failed="true"
	fi
	if [[ "$failed" != "false" ]]; then
		echo 'Error occurred while trying to build the CM730/CM740 firmware!'
		echo "If the required arm tool binary is not located in /opt/gcc-arm/bin,"
		echo "then please add its actual location to the PATH variable."
		return 1
	fi
	echo
	return 0
}

function _callservice() { # Example: _callservice /robotcontrol/nopInterface/attEstCalibrate robotcontrol
	if rosservice list &>/dev/null; then
		echo "Calling service: $1"
		rosservice call "$1" "${@:3}" || echo "Service call failed! Is the $2 node running?"
	else
		echo "Could not list the available ROS services, is a roscore running?"
	fi
}

function nimbro() {
	local LIGHT_CYAN="$(echo -e "\E[1;36m")"
	local NO_COLOUR="$(echo -e "\E[0m")"
	cd "$NIMBRO_ROOT"
	case "$1" in
		"")
			;;
		make)
			catkin_make "${@:2}" -DCMAKE_INSTALL_PREFIX="$INSTALLPATH" -DCMAKE_BUILD_TYPE=RelWithDebInfo
			;;
		make-doc | make-docv)
			if [[ "$1" == "make-docv" ]]; then
				VFLAG="-v"
			else
				VFLAG=
			fi
			case "$2" in
				"" | "nim" | "nimbro")
					_gendoc "$NIMBRO_ROOT/src/nimbro/doc/generate.sh" $VFLAG
					cd "$NIMBRO_ROOT/src/nimbro"
					if [[ "$3" == "open" ]]; then
						_opendoc "$NIMBRO_ROOT/src/nimbro/doc/out/html/index.html"
					fi
					;;
				"vis" | "visualization")
					_gendoc "$NIMBRO_ROOT/src/nimbro_vis/doc/generate.sh" $VFLAG
					cd "$NIMBRO_ROOT/src/nimbro_vis"
					if [[ "$3" == "open" ]]; then
						_opendoc "$NIMBRO_ROOT/src/nimbro_vis/doc/out/html/index.html"
					fi
					;;
				"rob" | "robot" | "robotcontrol")
					_gendoc "$NIMBRO_ROOT/src/nimbro_robotcontrol/doc/generate.sh" $VFLAG
					cd "$NIMBRO_ROOT/src/nimbro_robotcontrol"
					if [[ "$3" == "open" ]]; then
						_opendoc "$NIMBRO_ROOT/src/nimbro_robotcontrol/doc/out/html/index.html"
					fi
					;;
				"con" | "config" | "config_server")
					_gendoc "$NIMBRO_ROOT/src/nimbro_config_server/doc/generate.sh" $VFLAG
					cd "$NIMBRO_ROOT/src/nimbro_config_server"
					if [[ "$3" == "open" ]]; then
						_opendoc "$NIMBRO_ROOT/src/nimbro_config_server/doc/out/html/index.html"
					fi
					;;
				"all")
					_gendoc "$NIMBRO_ROOT/src/nimbro/doc/generate.sh" $VFLAG
					_gendoc "$NIMBRO_ROOT/src/nimbro_vis/doc/generate.sh" $VFLAG
					_gendoc "$NIMBRO_ROOT/src/nimbro_robotcontrol/doc/generate.sh" $VFLAG
					_gendoc "$NIMBRO_ROOT/src/nimbro_config_server/doc/generate.sh" $VFLAG
					cd "$NIMBRO_ROOT/src"
					if [[ "$3" == "open" ]]; then
						_opendoc "$NIMBRO_ROOT/src/nimbro/doc/out/html/index.html"
						_opendoc "$NIMBRO_ROOT/src/nimbro_vis/doc/out/html/index.html"
						_opendoc "$NIMBRO_ROOT/src/nimbro_robotcontrol/doc/out/html/index.html"
						_opendoc "$NIMBRO_ROOT/src/nimbro_config_server/doc/out/html/index.html"
					fi
					;;
				"open")
					_gendoc "$NIMBRO_ROOT/src/nimbro/doc/generate.sh" $VFLAG
					cd "$NIMBRO_ROOT/src/nimbro"
					_opendoc "$NIMBRO_ROOT/src/nimbro/doc/out/html/index.html"
					;;
				*)
					if [[ "$2" != "-h" && "$2" != "--help" ]]; then
						echo "Unknown parameter '$2'"'!'
					fi
					echo
					echo "Usage: nimbro make-doc[v] [REPO] [open]"
					echo "Generates the documentation of a NimbRo repository."
					echo
					echo "make-doc   Generate the documentation showing only Doxygen warnings/errors"
					echo "make-docv  Generate the documentation with full Doxygen verbosity"
					echo "REPO       The repository to generate the documentation for (default: nim)"
					echo "           {<empty>|nim|nimbro|vis|visualization|rob|robot|robotcontrol|"
					echo "            con|config|config_server|all}"
					echo "open       Open the generated documentation main pages"
					cd "$NIMBRO_ROOT/src"
					;;
			esac
			;;
		doc)
			case "$2" in
				"" | "nim" | "nimbro")
					_opendoc "$NIMBRO_ROOT/src/nimbro/doc/out/html/index.html"
					cd "$NIMBRO_ROOT/src/nimbro"
					;;
				"vis" | "visualization")
					_opendoc "$NIMBRO_ROOT/src/nimbro_vis/doc/out/html/index.html"
					cd "$NIMBRO_ROOT/src/nimbro_vis"
					;;
				"rob" | "robot" | "robotcontrol")
					_opendoc "$NIMBRO_ROOT/src/nimbro_robotcontrol/doc/out/html/index.html"
					cd "$NIMBRO_ROOT/src/nimbro_robotcontrol"
					;;
				"con" | "config" | "config_server")
					_opendoc "$NIMBRO_ROOT/src/nimbro_config_server/doc/out/html/index.html"
					cd "$NIMBRO_ROOT/src/nimbro_config_server"
					;;
				"all")
					_opendoc "$NIMBRO_ROOT/src/nimbro/doc/out/html/index.html"
					_opendoc "$NIMBRO_ROOT/src/nimbro_vis/doc/out/html/index.html"
					_opendoc "$NIMBRO_ROOT/src/nimbro_robotcontrol/doc/out/html/index.html"
					_opendoc "$NIMBRO_ROOT/src/nimbro_config_server/doc/out/html/index.html"
					cd "$NIMBRO_ROOT/src"
					;;
				*)
					if [[ "$2" != "-h" && "$2" != "--help" ]]; then
						echo "Unknown parameter '$2'"'!'
					fi
					echo
					echo "Usage: nimbro doc [REPO]"
					echo "Opens the documentation of a NimbRo repository."
					echo
					echo "REPO  The repository to open the documentation for (default: nim)"
					echo "      {<empty>|nim|nimbro|vis|visualization|rob|robot|robotcontrol|"
					echo "       con|config|config_server|all}"
					cd "$NIMBRO_ROOT/src"
					;;
			esac
			;;
		deploy)
			if [[ "$2" == "clean" ]]; then
				echo "Removing all contents of the /nimbro folder..."
				find /nimbro -mindepth 1 -name "*" -delete || { echo "Something went wrong."; return; }
				echo "Done."
				return
			fi
			if nimbro make install; then
				find /nimbro -mindepth 1 -name "*~" -type f -printf "Removing file %p\n" -delete
				if [ -z "$2" ];then
					rsync -avz --delete /nimbro/ nimbro@$BOT:/nimbro
				else
					target="$2"
					rsync -avz --delete /nimbro/ nimbro@$target:/nimbro
				fi
				if [ $? -ne 0 ]; then
					echo $LIGHT_CYAN"############################################################"$NO_COLOUR
					echo $LIGHT_CYAN"Careful: The rsync failed so the code has NOT been deployed"'!'$NO_COLOUR
					echo $LIGHT_CYAN"############################################################"$NO_COLOUR
				fi
			else
				echo $LIGHT_CYAN"###########################################################"$NO_COLOUR
				echo $LIGHT_CYAN"Careful: The make install failed so no rsync was performed"'!'$NO_COLOUR
				echo $LIGHT_CYAN"###########################################################"$NO_COLOUR
			fi
			;;
		flash)
			cd "$NIMBRO_ROOT/src/nimbro/hardware/cm730/firmware"
			echo "Firmware in directory: $(pwd)"
			if [[ "$2" == "clean" ]]; then
				echo "Removing all CM730 firmware build products other than the output *.hex file..."
				make cleanbuild
			elif [[ "$2" == "compile" ]]; then
				echo "Remaking the CM730 firmware..."
				device="$3"
				_makefirmware "$device" || return 1
				echo "Output hex file located at:"
				echo "$(pwd)/CM730.hex"
				echo
			elif [[ -z "$2" ]] || [[ "$2" == "robot" ]]; then
				if [[ -n "$3" ]]; then
					localbot="$3"
				else
					localbot="$BOT"
				fi
				localbot="${localbot%% *}"
				echo "Remaking the CM730 firmware and flashing it onto the robot $localbot..."
				modelnumber=
				[[ "$4" == "CM730" ]] && modelnumber="0x7301"
				[[ "$4" == "CM740" ]] && modelnumber="0x7401"
				if [[ -z "$modelnumber" ]]; then
					echo
					echo "Getting board model number using dynatool over ssh..."
					modelnumber="$(ssh nimbro@$localbot 'bash -ic '"'"'dynatool --device=/dev/cm730 "--exec=dev --pure 200;read --pure 0"'"'"'' |& grep -v 'bash:')"
					echo "Dynatool says the board is: $modelnumber"
					echo
				fi
				if [[ "$modelnumber" == "0x7300" ]] || [[ "$modelnumber" == "0x7301" ]]; then
					_makefirmware "CM730" || return 1
				elif [[ "$modelnumber" == "0x7400" ]] || [[ "$modelnumber" == "0x7401" ]]; then
					_makefirmware "CM740" || return 1
				else
					echo "Unrecognised model number '$modelnumber'! Don't know how to flash this device."
					read -p "Flash anyway with CM730 firmware (y/N)? " response 2>&1
					response="${response:0:1}"
					if [[ "${response,,}" != "y" ]]; then
						echo "No => Stopping here then."
						return 0
					fi
					_makefirmware "CM730" || return 1
				fi
				echo "Copying CM730 hex file onto the robot..."
				echo "Destination file: /home/nimbro/CM730.hex"
				scp CM730.hex nimbro@$localbot:/home/nimbro/ && {
					echo "Executing firmware installer on robot over ssh..."
					ssh nimbro@$localbot '/home/nimbro/firmware_installer -c CM730.hex'
					echo 'Done!'
				}
				echo
			elif [[ "$2" == "direct" ]]; then
				CMDEVICE="/dev/ttyUSB0"
				if [[ -n "$3" ]]; then
					if [[ -c "$3" ]]; then
						CMDEVICE="$3"
					else
						echo "Specified device '$3' not found - Ignoring this..."
					fi
				fi
				echo "Remaking the CM730 firmware and flashing it onto the microcontroller..."
				modelnumber=
				[[ "$4" == "CM730" ]] && modelnumber="0x7301"
				[[ "$4" == "CM740" ]] && modelnumber="0x7401"
				if [[ -z "$modelnumber" ]]; then
					echo
					echo "Getting board model number using dynatool..."
					modelnumber="$(dynatool "--device=$CMDEVICE" "--exec=dev --pure 200;read --pure 0")"
					echo "Dynatool says the board is: $modelnumber"
					echo
				fi
				if [[ "$modelnumber" == "0x7300" ]] || [[ "$modelnumber" == "0x7301" ]]; then
					_makefirmware "CM730" || return 1
				elif [[ "$modelnumber" == "0x7400" ]] || [[ "$modelnumber" == "0x7401" ]]; then
					_makefirmware "CM740" || return 1
				else
					echo "Unrecognised model number '$modelnumber'! Don't know how to flash this device."
					read -p "Flash anyway with CM730 firmware (y/N)? " response 2>&1
					response="${response:0:1}"
					if [[ "${response,,}" != "y" ]]; then
						echo "No => Stopping here then."
						return 0
					fi
					_makefirmware "CM730" || return 1
				fi
				FIRM_INST="$NIMBRO_ROOT/tools/firmware_installer/firmware_installer"
				if [[ -x "$FIRM_INST" ]]; then
					echo "Flashing to device: $CMDEVICE"
					echo
					if [[ -c "$CMDEVICE" ]]; then
						"$FIRM_INST" -c CM730.hex -d "$CMDEVICE"
					else
						echo "Could not detect the CM730 device on ttyUSB0 of the local computer. Are you sure it's connected?"
						return 1
					fi
				else
					echo
					echo "Error: Could not locate and/or execute the firmware installer binary at the following location:"
					echo "       $FIRM_INST"
					return 1
				fi
				echo
			else
				echo "Unrecognised flash subcommand '$2'"'!'
				echo "Usage: nimbro flash [clean|compile|direct|robot]"
			fi
			;;
		clean)
			echo "Working directory: $(pwd)"
			echo "Clearing out build products..."
			rm -rf "$NIMBRO_ROOT/build" "$NIMBRO_ROOT/devel" || echo "Something went wrong. Do the folders even exist?"
			echo "Clearing out temporary files..."
			find . -name "*~" -type f -printf "Removing file %p\n" -delete
			echo "Clearing out .directory files..."
			find . -name ".directory" -type f -printf "Removing file %p\n" -delete
			echo "Done"
			;;
		remake-all)
			echo "Removing build/ and devel/ folders from nimbro project root..."
			rm -rf "$NIMBRO_ROOT/build" "$NIMBRO_ROOT/devel"
			echo "Running catkin_make..."
			catkin_make "${@:2}" -DCMAKE_INSTALL_PREFIX="$INSTALLPATH" -DCMAKE_BUILD_TYPE=RelWithDebInfo
			;;
		source | src)
			case "$2" in
				"src" | "source")
					cd "$NIMBRO_ROOT/src"
					;;
				"" | "nim" | "nimbro")
					cd "$NIMBRO_ROOT/src/nimbro"
					;;
				"vis" | "visualization")
					cd "$NIMBRO_ROOT/src/nimbro_vis"
					;;
				"rob" | "robot" | "robotcontrol")
					cd "$NIMBRO_ROOT/src/nimbro_robotcontrol"
					;;
				"con" | "config" | "config_server")
					cd "$NIMBRO_ROOT/src/nimbro_config_server"
					;;
				"dyn" | "dynalib")
					cd "$NIMBRO_ROOT/src/dynalib"
					;;
				"mon" | "rosmon")
					cd "$NIMBRO_ROOT/src/rosmon"
					;;
				"mis" | "misc")
					cd "$NIMBRO_ROOT/misc"
					;;
				"nop" | "nopnotes")
					cd "$NIMBRO_ROOT/misc/NOPNotes"
					;;
				"rgen" | "releasegen")
					cd "$NIMBRO_ROOT/releasegen"
					;;
				"rel" | "release" | "nimbro-op-ros")
					cd "$NIMBRO_ROOT/nimbro-op-ros"
					;;
				*)
					echo "Unknown parameter '$2': Going to main nimbro repository"'!'
					echo "Usage: nimbro source [repository]"
					echo "[repository] can be:"
					echo "src folder:           src, source"
					echo "nimbro:               <empty>, nim, nimbro"
					echo "nimbro_vis:           vis, visualization"
					echo "nimbro_robotcontrol:  rob, robot, robotcontrol"
					echo "nimbro_config_server: con, config, config_server"
					echo "dynalib:              dyn, dynalib"
					echo "rosmon:               mon, rosmon"
					echo "misc:                 mis, misc"
					echo "NOPNotes:             nop, nopnotes"
					echo "releasegen:           rgen, releasegen"
					echo "nimbro-op-ros:        rel, release, nimbro-op-ros"
					cd "$NIMBRO_ROOT/src/nimbro"
					;;
			esac
			;;
		status)
			echo "Printing the git status of all the repositories..."
			cd "$NIMBRO_ROOT/src/nimbro"
			echo
			echo $LIGHT_CYAN"*** nimbro repository ***"$NO_COLOUR
			git status
			cd "$NIMBRO_ROOT/src/nimbro_vis"
			echo
			echo $LIGHT_CYAN"*** nimbro_vis repository ***"$NO_COLOUR
			git status
			cd "$NIMBRO_ROOT/src/nimbro_robotcontrol"
			echo
			echo $LIGHT_CYAN"*** nimbro_robotcontrol repository ***"$NO_COLOUR
			git status
			cd "$NIMBRO_ROOT/src/nimbro_config_server"
			echo
			echo $LIGHT_CYAN"*** nimbro_config_server repository ***"$NO_COLOUR
			git status
			cd "$NIMBRO_ROOT/src/dynalib"
			echo
			echo $LIGHT_CYAN"*** dynalib repository ***"$NO_COLOUR
			git status
			if [[ -d "$NIMBRO_ROOT/src/rosmon" ]]; then
				cd "$NIMBRO_ROOT/src/rosmon"
				echo
				echo $LIGHT_CYAN"*** rosmon repository ***"$NO_COLOUR
				git status
			fi
			if [[ -d "$NIMBRO_ROOT/misc" ]]; then
				cd "$NIMBRO_ROOT/misc"
				echo
				echo $LIGHT_CYAN"*** misc repository ***"$NO_COLOUR
				git status
			fi
			if [[ -d "$NIMBRO_ROOT/misc/NOPNotes" ]]; then
				cd "$NIMBRO_ROOT/misc/NOPNotes"
				echo
				echo $LIGHT_CYAN"*** NOPNotes repository ***"$NO_COLOUR
				git status
			fi
			if [[ -d "$NIMBRO_ROOT/releasegen" ]]; then
				cd "$NIMBRO_ROOT/releasegen"
				echo
				echo $LIGHT_CYAN"*** releasegen repository ***"$NO_COLOUR
				git status
			fi
			if [[ -d "$NIMBRO_ROOT/nimbro-op-ros" ]]; then
				cd "$NIMBRO_ROOT/nimbro-op-ros"
				echo
				echo $LIGHT_CYAN"*** nimbro-op-ros repository ***"$NO_COLOUR
				git status
			fi
			echo
			cd "$NIMBRO_ROOT/src/nimbro"
			;;
		gui)
			echo "Opening git gui for repositories with working tree changes..."
			cd "$NIMBRO_ROOT/src/nimbro"
			echo
			echo $LIGHT_CYAN"*** nimbro repository ***"$NO_COLOUR
			[[ -n "$(git status --porcelain --untracked-files=normal)" ]] && git gui
			cd "$NIMBRO_ROOT/src/nimbro_vis"
			echo
			echo $LIGHT_CYAN"*** nimbro_vis repository ***"$NO_COLOUR
			[[ -n "$(git status --porcelain --untracked-files=normal)" ]] && git gui
			cd "$NIMBRO_ROOT/src/nimbro_robotcontrol"
			echo
			echo $LIGHT_CYAN"*** nimbro_robotcontrol repository ***"$NO_COLOUR
			[[ -n "$(git status --porcelain --untracked-files=normal)" ]] && git gui
			cd "$NIMBRO_ROOT/src/nimbro_config_server"
			echo
			echo $LIGHT_CYAN"*** nimbro_config_server repository ***"$NO_COLOUR
			[[ -n "$(git status --porcelain --untracked-files=normal)" ]] && git gui
			cd "$NIMBRO_ROOT/src/dynalib"
			echo
			echo $LIGHT_CYAN"*** dynalib repository ***"$NO_COLOUR
			[[ -n "$(git status --porcelain --untracked-files=normal)" ]] && git gui
			if [[ -d "$NIMBRO_ROOT/src/rosmon" ]]; then
				cd "$NIMBRO_ROOT/src/rosmon"
				echo
				echo $LIGHT_CYAN"*** rosmon repository ***"$NO_COLOUR
				[[ -n "$(git status --porcelain --untracked-files=normal)" ]] && git gui
			fi
			if [[ -d "$NIMBRO_ROOT/misc" ]]; then
				cd "$NIMBRO_ROOT/misc"
				echo
				echo $LIGHT_CYAN"*** misc repository ***"$NO_COLOUR
				[[ -n "$(git status --porcelain --untracked-files=normal)" ]] && git gui
			fi
			if [[ -d "$NIMBRO_ROOT/misc/NOPNotes" ]]; then
				cd "$NIMBRO_ROOT/misc/NOPNotes"
				echo
				echo $LIGHT_CYAN"*** NOPNotes repository ***"$NO_COLOUR
				[[ -n "$(git status --porcelain --untracked-files=normal)" ]] && git gui
			fi
			if [[ -d "$NIMBRO_ROOT/releasegen" ]]; then
				cd "$NIMBRO_ROOT/releasegen"
				echo
				echo $LIGHT_CYAN"*** releasegen repository ***"$NO_COLOUR
				[[ -n "$(git status --porcelain --untracked-files=normal)" ]] && git gui
			fi
			if [[ -d "$NIMBRO_ROOT/nimbro-op-ros" ]]; then
				cd "$NIMBRO_ROOT/nimbro-op-ros"
				echo
				echo $LIGHT_CYAN"*** nimbro-op-ros repository ***"$NO_COLOUR
				[[ -n "$(git status --porcelain --untracked-files=normal)" ]] && git gui
			fi
			echo
			cd "$NIMBRO_ROOT/src/nimbro"
			;;
		pull)
			cd "$NIMBRO_ROOT/src/nimbro"
			echo
			echo $LIGHT_CYAN"*** Pulling nimbro repository ***"$NO_COLOUR
			gitpull
			cd "$NIMBRO_ROOT/src/nimbro_vis"
			echo
			echo $LIGHT_CYAN"*** Pulling nimbro_vis repository ***"$NO_COLOUR
			gitpull
			cd "$NIMBRO_ROOT/src/nimbro_robotcontrol"
			echo
			echo $LIGHT_CYAN"*** Pulling nimbro_robotcontrol repository ***"$NO_COLOUR
			gitpull
			cd "$NIMBRO_ROOT/src/nimbro_config_server"
			echo
			echo $LIGHT_CYAN"*** Pulling nimbro_config_server repository ***"$NO_COLOUR
			gitpull
			cd "$NIMBRO_ROOT/src/dynalib"
			echo
			echo $LIGHT_CYAN"*** Pulling dynalib repository ***"$NO_COLOUR
			gitpull
			if [[ -d "$NIMBRO_ROOT/src/rosmon" ]]; then
				cd "$NIMBRO_ROOT/src/rosmon"
				echo
				echo $LIGHT_CYAN"*** Pulling rosmon repository ***"$NO_COLOUR
				gitpull
			fi
			if [[ -d "$NIMBRO_ROOT/misc" ]]; then
				cd "$NIMBRO_ROOT/misc"
				echo
				echo $LIGHT_CYAN"*** Pulling misc repository ***"$NO_COLOUR
				gitpull
			fi
			if [[ -d "$NIMBRO_ROOT/misc/NOPNotes" ]]; then
				cd "$NIMBRO_ROOT/misc/NOPNotes"
				echo
				echo $LIGHT_CYAN"*** Pulling NOPNotes repository ***"$NO_COLOUR
				gitpull
			fi
			if [[ -d "$NIMBRO_ROOT/releasegen" ]]; then
				cd "$NIMBRO_ROOT/releasegen"
				echo
				echo $LIGHT_CYAN"*** Pulling releasegen repository ***"$NO_COLOUR
				gitpull
			fi
			if [[ -d "$NIMBRO_ROOT/nimbro-op-ros" ]]; then
				cd "$NIMBRO_ROOT/nimbro-op-ros"
				echo
				echo $LIGHT_CYAN"*** Pulling nimbro-op-ros repository ***"$NO_COLOUR
				gitpull
			fi
			echo
			cd "$NIMBRO_ROOT/src/nimbro"
			;;
		push)
			cd "$NIMBRO_ROOT/src/nimbro"
			echo
			echo $LIGHT_CYAN"*** Pushing nimbro repository ***"$NO_COLOUR
			gitpush "${@:2}"
			cd "$NIMBRO_ROOT/src/nimbro_vis"
			echo
			echo $LIGHT_CYAN"*** Pushing nimbro_vis repository ***"$NO_COLOUR
			gitpush "${@:2}"
			cd "$NIMBRO_ROOT/src/nimbro_robotcontrol"
			echo
			echo $LIGHT_CYAN"*** Pushing nimbro_robotcontrol repository ***"$NO_COLOUR
			gitpush "${@:2}"
			cd "$NIMBRO_ROOT/src/nimbro_config_server"
			echo
			echo $LIGHT_CYAN"*** Pushing nimbro_config_server repository ***"$NO_COLOUR
			gitpush "${@:2}"
			cd "$NIMBRO_ROOT/src/dynalib"
			echo
			echo $LIGHT_CYAN"*** Pushing dynalib repository ***"$NO_COLOUR
			gitpush "${@:2}"
			if [[ -d "$NIMBRO_ROOT/src/rosmon" ]]; then
				cd "$NIMBRO_ROOT/src/rosmon"
				echo
				echo $LIGHT_CYAN"*** Pushing rosmon repository ***"$NO_COLOUR
				gitpush "${@:2}"
			fi
			if [[ -d "$NIMBRO_ROOT/misc" ]]; then
				cd "$NIMBRO_ROOT/misc"
				echo
				echo $LIGHT_CYAN"*** Pushing misc repository ***"$NO_COLOUR
				gitpush "${@:2}"
			fi
			if [[ -d "$NIMBRO_ROOT/misc/NOPNotes" ]]; then
				cd "$NIMBRO_ROOT/misc/NOPNotes"
				echo
				echo $LIGHT_CYAN"*** Pushing NOPNotes repository ***"$NO_COLOUR
				gitpush "${@:2}"
			fi
			if [[ -d "$NIMBRO_ROOT/releasegen" ]]; then
				cd "$NIMBRO_ROOT/releasegen"
				echo
				echo $LIGHT_CYAN"*** Pushing releasegen repository ***"$NO_COLOUR
				gitpush "${@:2}"
			fi
			if [[ -d "$NIMBRO_ROOT/nimbro-op-ros" ]]; then
				cd "$NIMBRO_ROOT/nimbro-op-ros"
				echo
				echo $LIGHT_CYAN"*** Pushing nimbro-op-ros repository ***"$NO_COLOUR
				gitpush "${@:2}"
			fi
			echo
			cd "$NIMBRO_ROOT/src/nimbro"
			;;
		set)
			if [[ -z "$2" ]]; then
				echo "Currently set nimbro robot is:"
				echo "NIMBRO_ROBOT_TYPE: $NIMBRO_ROBOT_TYPE"
				echo "NIMBRO_ROBOT_NAME: $NIMBRO_ROBOT_NAME"
				echo "NIMBRO_ROBOT_VARIANT: $NIMBRO_ROBOT_VARIANT"
				echo "These can be modified using: nimbro set TYPE [NAME] [VARIANT]"
				return
			fi
			export NIMBRO_ROBOT_TYPE="$2"
			export NIMBRO_ROBOT_NAME="$3"
			if [[ -n "$4" ]]; then
				export NIMBRO_ROBOT_VARIANT="$4"
			fi
			;;
		robot | bot)
			if [[ -z "$2" ]]; then
				echo "Currently selected robot is '$BOT'."
				return
			fi
			if ! ping -c1 "$2" > /dev/null; then
				echo "Could not resolve host name '$2'."
				echo "Selecting '$2' as host anyway..."
			fi
			BOT="$2"
			echo "Robot '$BOT' selected."
			;;
		ctrl | control)
			case "$2" in
				"fade")
					case "$3" in
						"in")
							_fadetorque 1.0
						;;
						"out")
							_fadetorque 0.0
						;;
						*)
							echo "Unrecognised fade command."
							echo "Usage: nimbro {control|ctrl} fade {in|out}"
						;;
					esac
				;;
				"halt")
					_gaitcmd 0.0 0.0 0.0 false
				;;
				"head")
					if [[ -z "$3" ]]; then
						echo "Need to specify a head command."
						echo "Usage: nimbro {control|ctrl} head {disable|movetoangle|movetovector} [...]"
					elif [[ "$3" == "disable" ]]; then
						_headcontrol false false false 0.0 0.0 0.0 0.0 0.0
					elif [[ "$3" == "enable" ]]; then
						_headcontrol true true false 0.0 0.0 0.0 0.0 0.0
					elif [[ "$3" == "relax" ]]; then
						_headcontrol true true false 0.0 0.0 0.0 -1 -1
					elif [[ "$3" == "movetoangle" ]]; then
						local zangle="$4"
						local yangle="$5"
						local yeffort="$6"
						local peffort="$7"
						[[ -z "$zangle" ]] && zangle=0.0
						[[ -z "$yangle" ]] && yangle=0.0
						[[ -z "$yeffort" ]] && yeffort=0.0
						[[ -z "$peffort" ]] && peffort=0.0
						_headcontrol true true false 0.0 "$yangle" "$zangle" "$peffort" "$yeffort"
					elif [[ "$3" == "movetovector" ]]; then
						local xcoord="$4"
						local ycoord="$5"
						local zcoord="$6"
						local yeffort="$7"
						local peffort="$8"
						[[ -z "$xcoord" ]] && xcoord=0.0
						[[ -z "$ycoord" ]] && ycoord=0.0
						[[ -z "$zcoord" ]] && zcoord=1.0
						[[ -z "$yeffort" ]] && yeffort=0.0
						[[ -z "$peffort" ]] && peffort=0.0
						_headcontrol true false false "$xcoord" "$ycoord" "$zcoord" "$peffort" "$yeffort"
					fi
				;;
				"walk")
					local gcvx="$3"
					local gcvy="$4"
					local gcvz="$5"
					local walk="$6"
					[[ -z "$gcvx" ]] && gcvx=0
					[[ -z "$gcvy" ]] && gcvy=0
					[[ -z "$gcvz" ]] && gcvz=0
					[[ -z "$walk" ]] && walk=true
					_gaitcmd "$gcvx" "$gcvy" "$gcvz" "$walk"
				;;
				*)
					echo "Unrecognised control command"
					echo "Usage: nimbro {control|ctrl} {fade|walk|halt} [...]"
				;;
			esac
			;;
		calib)
			case "$2" in
				"heading")
					( # Start a subshell to have only a temporary change of the ROS master
						if [[ -n "$3" ]]; then
							ROS_MASTER_URI=http://"$3":11311
							ROS_HOSTNAME=$(hostname).local
						fi
						local curhost="${ROS_MASTER_URI#http://}"
						curhost="${curhost%:11311}"
						echo "Calibration of the heading via the attitude estimator magnetometer calibration vector:"
						echo "Put the robot on the field exactly upright and facing the positive goal..."
						read -p "Continue with heading calibration of $curhost (Y/n)? " response 2>&1
						response="${response:0:1}"
						[[ "${response,,}" == "n" ]] && echo "No => Stopping here then." || _callservice /robotcontrol/nopInterface/attEstCalibrate robotcontrol
					)
				;;
				"magnetometer")
					( # Start a subshell to have only a temporary change of the ROS master
						[[ "$3" == "warpAdd" ]] && hostvar="$5" || hostvar="$4"
						if [[ -n "$hostvar" ]]; then
							ROS_MASTER_URI=http://"$hostvar":11311
							ROS_HOSTNAME=$(hostname).local
						fi
						local curhost="${ROS_MASTER_URI#http://}"
						curhost="${curhost%:11311}"
						if [[ "$3" == "start" ]]; then
							echo "Start 2D/3D calibration of the magnetometer:"
							echo "2D => Yaw the robot in all directions but ensure it remains perfectly upright at all times"
							echo "3D => Rotate the robot in every direction imaginable"
							read -p "Start magnetometer calibration of $curhost (Y/n)? " response 2>&1
							response="${response:0:1}"
							[[ "${response,,}" == "n" ]] && echo "No => Stopping here then." || _callservice /robotcontrol/nopInterface/magFilter/startCalibration robotcontrol
						elif [[ "$3" == "stop2D" ]]; then
							echo "Stopping the 2D magnetometer calibration of $curhost..."
							_callservice /robotcontrol/nopInterface/magFilter/stopCalibration2D robotcontrol
						elif [[ "$3" == "stop3D" ]]; then
							echo "Stopping the 3D magnetometer calibration of $curhost..."
							_callservice /robotcontrol/nopInterface/magFilter/stopCalibration3D robotcontrol
						elif [[ "$3" == "warpClear" ]]; then
							echo "Clearing the magnetometer warp parameter string, resulting in an identity yaw warp transform..."
							_callservice /robotcontrol/nopInterface/magFilter/warpClearPoints robotcontrol
						elif [[ "$3" == "warpAdd" ]]; then
							echo "Adding reference point that robot is currently at a true yaw (CCW from +ve goal) of $4 degrees..."
							_callservice /robotcontrol/nopInterface/magFilter/warpAddPoint robotcontrol "$4"
						else
							echo "Usage: nimbro calib magnetometer {start|stop2D|stop3D|warpClear|warpAdd} [value] [robot]"
						fi
					)
				;;
				*)
					echo "Unrecognised calib command"
					echo "Usage: nimbro calib {heading|magnetometer} [...]"
				;;
			esac
			;;
		config)
			case "$2" in
				"list")
					proctemp="$(rostopic echo -n 1 /config_server/parameter_list | grep name | sort)"
					if [[ -z "$proctemp" ]]; then
						echo "Unable to retrieve parameter list from config server"'!'
						return
					fi
					configlist=()
					while IFS= read -r -d $'\n' line; do
						[[ -z "$line" ]] && continue
						configlist+=("/${line#*/}")
					done <<< "$proctemp"
					echo "$(IFS=$'\n'; echo "${configlist[*]}")"
				;;
				"get")
					paramname="$3"
					if [[ -z "$paramname" ]] || [[ "${paramname:0:1}" != "/" ]]; then
						echo "Config parameter name '$paramname' must be non-null and start with '/'"'!'
					else
						paramvalue="$(rosservice call /config_server/get_parameter "name: '$paramname'")"
						paramvalue="${paramvalue#value: }"
						echo "$paramname ==> $paramvalue"
					fi
				;;
				"reset" | "load")
					confname="$(rosparam get /config_server/robot_name)"
					if [[ -n "$confname" ]]; then
						confname="config_$confname"
						echo "Reloading configuration parameters from yaml file '$confname.yaml'."
						confpath="$(rosparam get /config_server/config_path)"
						[[ -n "$confpath" ]] && echo "In directory: $confpath"
						rosservice call /config_server/load "filename: '$confname'"
					else
						echo "Could not get robot name from ROS param '/config_server/robot_name'"'!'
					fi
				;;
				"save")
					confname="$(rosparam get /config_server/robot_name)"
					if [[ -n "$confname" ]]; then
						confname="config_$confname"
						echo "Saving current state of configuration parameters into '$confname.yaml'."
						confpath="$(rosparam get /config_server/config_path)"
						[[ -n "$confpath" ]] && echo "In directory: $confpath"
						rosservice call /config_server/save "filename: '$confname'"
					else
						echo "Could not get robot name from ROS param '/config_server/robot_name'"'!'
					fi
				;;
				"set")
					paramname="$3"
					paramvalue="$4"
					if [[ -z "$paramname" ]] || [[ "${paramname:0:1}" != "/" ]]; then
						echo "Config parameter name '$paramname' must be non-null and start with '/'"'!'
					elif [[ -z "$paramvalue" ]]; then
						echo "No value to set the config parameter to was provided"'!'
					else
						[[ "$paramvalue" == "true" ]] && paramvalue="1"
						[[ "$paramvalue" == "false" ]] && paramvalue="0"
						[[ "$paramvalue" == "on" ]] && paramvalue="1"
						[[ "$paramvalue" == "off" ]] && paramvalue="0"
						echo "Setting parameter '$paramname' to '$paramvalue'..."
						rosservice call /config_server/set_parameter "{name: '$paramname', value: '$paramvalue', no_notify: '0'}"
					fi
				;;
				"showdead")
					configPath="$3"
					echo "Showing dead config parameters in '$configPath'... (see robotcontrol console)"
					rosservice call /config_server/show_dead_vars "{configPath: '$configPath'}"
				;;
				"cleanyaml")
					local LAUNCH="$(rospack find launch)"
					if [[ -z "$3" ]]; then
						echo "Please specify the robot to clean the config file for as an argument.";
					elif [[ "$3" == "all" ]] || [[ "$4" == "all" ]] || [[ "$5" == "all" ]] || [[ "$6" == "all" ]]; then
						python "$LAUNCH/config/cleanYaml.py" "$LAUNCH/config/"config_*.yaml
					else
						[[ -n "$3" ]] && python "$LAUNCH/config/cleanYaml.py" "$LAUNCH/config/config_$3.yaml"
						[[ -n "$4" ]] && python "$LAUNCH/config/cleanYaml.py" "$LAUNCH/config/config_$4.yaml"
						[[ -n "$5" ]] && python "$LAUNCH/config/cleanYaml.py" "$LAUNCH/config/config_$5.yaml"
						[[ -n "$6" ]] && python "$LAUNCH/config/cleanYaml.py" "$LAUNCH/config/config_$6.yaml"
					fi
				;;
				"cpyaml")
					local LAUNCH="$(rospack find launch)"
					python "$LAUNCH/config/copyConfig.py" "-f$3" "-t$4" "-p$5" "-b$LAUNCH/config/"
				;;
				"retrieve")
					local target="$BOT"
					[[ -n "$3" ]] && target="$3"
					local LAUNCH="$(rospack find launch)"
					scp "nimbro@$target:/nimbro/share/launch/config/config*.yaml" "$LAUNCH/config/"
					scp "nimbro@$target:/nimbro/share/launch/config/vision/*" "$LAUNCH/config/vision/"
					scp "nimbro@$target:/nimbro/share/camera_v4l2/launch/cam_settings.yaml" "$(rospack find camera_v4l2)/launch/"
				;;
				*)
					echo "Unrecognised config command"
					echo "Usage: nimbro config {list|reset|load|save|get|set|showdead|cleanyaml|cpyaml|retrieve} [...]"
				;;
			esac
			;;
		sim)
			case "$2" in
				"poke")
					$SCRIPTS_DIR/gazebo_apply_force.py "${@:3}"
				;;
				*)
					echo "Unrecognised sim command"
					echo "Usage: nimbro sim {poke} [...]"
				;;
			esac
			;;
		host)
			host="$2"
			if [[ "$host" == "reset" ]]; then
				export ROS_HOSTNAME=localhost
				export ROS_MASTER_URI=http://localhost:11311
				return 0
			fi
			export ROS_MASTER_URI=http://$host:11311
			if ! ping -c1 $host > /dev/null; then
				echo "Could not resolve host name '$host'."
				return 1
			fi
			if ping -c1 $(hostname).local > /dev/null; then
				export ROS_HOSTNAME=$(hostname).local
			fi
			if ! rostopic list > /dev/null; then
				echo "Could not connect to ROS master running at '$host'"
				echo "Setting it as master anyway."
				BOT="$2"
				return 0
			fi
			echo "Connected."
			BOT="$2"
			;;
		ssh)
			if [[ -z "$2" ]]; then
				while ! ping -c1 "$BOT" 1>/dev/null; do sleep 0.5s; done
			    ssh nimbro@"$BOT"
			else
				local target="$2"
				while ! ping -c1 "$target" 1>/dev/null; do sleep 0.5s; done
				ssh nimbro@"$target"
			fi
			;;
		help|-h|--help)
			cat <<EOS
Usage: nimbro [command]

Commands:
  bot         Alias for the robot command
  clean       Removes any build products and temporary files
  config      Perform actions such as saving the config parameters
  control     Send specified command to a running robotcontrol instance
  ctrl        Alias for 'control'
  deploy      Make and deploy binaries to the robot
  doc         Opens the specified nimbro html documentation
  flash       Compiles and flashes the CM730 firmware onto a connected microcontroller
  getconfig   Get config.yaml from robot
  cpconfig  Copy config.yaml from one robot to another
  gui         Open the git gui for repositories that have changes in the working tree
  help        Display this help message
  host        Use HOST as the ROS master, e.g. nimbro host xs2.local
  make        Run catkin_make with correct arguments in the correct directory
  make-doc    Compile the doxygen documentation (use 'make-doc open' to automatically open the html)
  make-docv   Verbose compilation of the doxygen documentation (see make-doc)
  pull        Pull and rebase the latest commits for each of the source repositories
  push        Git push in each of the source repositories (asks for confirmation)
  remake-all  Hard clean build and devel folders then run catkin_make to remake entire project
  robot       Select a given robot, e.g. nimbro robot xs2.local
  set         Set the environment variables for a particular robot
  sim         Commands to control the robot Gazebo simulation
  source      Change to the nimbro source directory
  src         Alias for 'source'
  ssh         Open an SSH connection to the robot
  status      Display the git status of the internal nimbro repositories

The default command just changes into the NimbRo-OP catkin workspace.
EOS
			;;
		*)
			echo 'Unrecognised nimbro command!'
			echo "Try: nimbro help"
			;;
	esac
}

function _nimbro()
{
	local cur="${COMP_WORDS[COMP_CWORD]}"
	local cmd="${COMP_WORDS[1]}"
	local subcmd="${COMP_WORDS[2]}"
	local subsubcmd="${COMP_WORDS[3]}"
	
	local robotlist="xs4 xs4.local xs5 xs5.local xs6 xs6.local xs7 xs7.local"
	local P1list="xs4 xs5 xs6 xs7"
	
	COMPREPLY=""
	case "${COMP_CWORD}" in
		1)
			COMPREPLY=($(compgen -W "bot calib clean config control ctrl deploy doc flash gui help host make make-doc make-docv pull push remake-all robot set sim source src ssh status" -- "$cur"))
			;;
		2)
			case "$cmd" in
				calib)
					COMPREPLY=($(compgen -W "heading magnetometer" -- "$cur"))
					;;
				config)
					COMPREPLY=($(compgen -W "cleanyaml cpyaml get list load reset retrieve save set showdead" -- "$cur"))
					;;
				ctrl | control)
					COMPREPLY=($(compgen -W "fade halt head walk" -- "$cur"))
					;;
				deploy)
					COMPREPLY=($(compgen -W "clean $robotlist" -- "$cur"))
					;;
				doc)
					COMPREPLY=($(compgen -W "nimbro visualization robotcontrol config_server all" -- "$cur"))
					;;
				flash)
					COMPREPLY=($(compgen -W "clean compile direct robot" -- "$cur"))
					;;
				host)
					COMPREPLY=($(compgen -W "$robotlist" -- "$cur"))
					;;
				make-doc | make-docv)
					COMPREPLY=($(compgen -W "nimbro visualization robotcontrol config_server all open" -- "$cur"))
					;;
				robot | bot)
					COMPREPLY=($(compgen -W "$robotlist" -- "$cur"))
					;;
				set)
					COMPREPLY=($(compgen -W "P1" -- "$cur"))
					;;
				sim)
					COMPREPLY=($(compgen -W "poke" -- "$cur"))
					;;
				source | src)
					local sources="src nimbro visualization robotcontrol config_server dynalib"
					[[ -d "$NIMBRO_ROOT/src/rosmon" ]] && sources+=" mon rosmon"
					[[ -d "$NIMBRO_ROOT/misc" ]] && sources+=" misc"
					[[ -d "$NIMBRO_ROOT/misc/NOPNotes" ]] && sources+=" nopnotes"
					[[ -d "$NIMBRO_ROOT/releasegen" ]] && sources+=" rgen releasegen"
					[[ -d "$NIMBRO_ROOT/nimbro-op-ros" ]] && sources+=" release nimbro-op-ros"
					COMPREPLY=($(compgen -W "$sources" -- "$cur"))
					;;
				ssh)
					COMPREPLY=($(compgen -W "$robotlist" -- "$cur"))
					;;
			esac
			;;
		3)
			case "$cmd" in
				calib)
					[[ "$subcmd" == "heading" ]] && COMPREPLY=($(compgen -W "localhost $robotlist" -- "$cur"))
					[[ "$subcmd" == "magnetometer" ]] && COMPREPLY=($(compgen -W "start stop2D stop3D warpAdd warpClear" -- "$cur"))
					;;
				config)
					[[ "$subcmd" == "cleanyaml" ]] && COMPREPLY=($(compgen -W "all $P1list" -- "$cur"))
					[[ "$subcmd" == "cpyaml" ]] && COMPREPLY=($(compgen -W "$P1list" -- "$cur"))
					[[ "$subcmd" == "retrieve" ]] && COMPREPLY=($(compgen -W "$robotlist" -- "$cur"))
					if [[ "$subcmd" == "get" ]] || [[ "$subcmd" == "set" ]] || [[ "$subcmd" == "showdead" ]]; then
						proctemp="$(rostopic echo -n 1 /config_server/parameter_list 2>/dev/null | grep name | sort)"
						if [[ -n "$proctemp" ]]; then
							local configlist=()
							while IFS= read -r -d $'\n' line; do
								[[ -z "$line" ]] && continue
								configlist+=("/${line#*/}")
							done <<< "$proctemp"
							COMPREPLY=($(compgen -W "${configlist[*]}" -- "$cur"))
						fi
					fi
					;;
				ctrl | control)
					[[ "$subcmd" == "fade" ]] && COMPREPLY=($(compgen -W "in out" -- "$cur"))
					[[ "$subcmd" == "walk" ]] && COMPREPLY=($(compgen -W "0.0" -- "$cur"))
					[[ "$subcmd" == "head" ]] && COMPREPLY=($(compgen -W "disable enable movetoangle movetovector relax" -- "$cur"))
					;;
				flash)
					[[ "$subcmd" == "compile" ]] && COMPREPLY=($(compgen -W "CM730 CM740" -- "$cur"))
					[[ "$subcmd" == "direct" ]] && COMPREPLY=($(compgen -G "/dev/ttyUSB*" -- "$cur"))
					[[ "$subcmd" == "robot" ]] && COMPREPLY=($(compgen -W "$robotlist" -- "$cur"))
					;;
				make-doc | make-docv)
					[[ "$subcmd" != "open" ]] && COMPREPLY=($(compgen -W "open" -- "$cur"))
					;;
				set)
					[[ "$subcmd" == "P1" ]] && COMPREPLY=($(compgen -W "$P1list" -- "$cur"))
					;;
			esac
			;;
		4)
			case "$cmd" in
				calib)
					if [[ "$subcmd" == "magnetometer" ]]; then
						if [[ "$subsubcmd" == "warpAdd" ]]; then
							COMPREPLY=($(compgen -W "0 45 90 180 225 270 315" -- "$cur"))
						else
							COMPREPLY=($(compgen -W "localhost $robotlist" -- "$cur"))
						fi
					fi
					;;
				config)
					[[ "$subcmd" == "cleanyaml" ]] && COMPREPLY=($(compgen -W "all $P1list" -- "$cur"))
					[[ "$subcmd" == "cpyaml" ]] && COMPREPLY=($(compgen -W "$P1list" -- "$cur"))
					;;
				ctrl | control)
					[[ "$subcmd" == "walk" ]] && COMPREPLY=($(compgen -W "0.0" -- "$cur"))
					[[ "$subcmd" == "head" ]] && [[ "${subsubcmd:0:6}" == "moveto" ]] && COMPREPLY=($(compgen -W "0.0" -- "$cur"))
					;;
				flash)
					[[ "$subcmd" == "direct" ]] && COMPREPLY=($(compgen -W "CM730 CM740" -- "$cur"))
					[[ "$subcmd" == "robot" ]] && COMPREPLY=($(compgen -W "CM730 CM740" -- "$cur"))
					;;
				set)
					[[ "$subcmd" == "P1" ]] && COMPREPLY=($(compgen -W "nimbro_op nimbro_op_hull" -- "$cur"))
					;;
			esac
			;;
		5)
			case "$cmd" in
				calib)
					[[ "$subcmd" == "magnetometer" ]] && [[ "$subsubcmd" == "warpAdd" ]] && COMPREPLY=($(compgen -W "localhost $robotlist" -- "$cur"))
					;;
				config)
					[[ "$subcmd" == "cleanyaml" ]] && COMPREPLY=($(compgen -W "all $P1list" -- "$cur"))
					if [[ "$subcmd" == "cpyaml" ]]; then
						local configdir="$(rospack find launch)/config"
						local configfile="$configdir/config_$subsubcmd.yaml"
						local depth="$(grep -o "/" <<< "$cur" | wc -l)"
						[[ "$depth" == "0" ]] && depth="1"
						proctemp="$(python "$(rospack find launch)/config/listYaml.py" "$configfile" "$depth" | sort)"
						if [[ -n "$proctemp" ]]; then
							local configlist=("all")
							while IFS= read -r -d $'\n' line; do
								[[ -z "$line" ]] && continue
								configlist+=("$line")
							done <<< "$proctemp"
							COMPREPLY=($(compgen -W "${configlist[*]}" -- "$cur"))
						fi
					fi
					;;
				ctrl | control)
					[[ "$subcmd" == "walk" ]] && COMPREPLY=($(compgen -W "0.0" -- "$cur"))
					[[ "$subcmd" == "head" ]] && [[ "${subsubcmd:0:6}" == "moveto" ]] && COMPREPLY=($(compgen -W "0.0" -- "$cur"))
					;;
			esac
			;;
		6)
			case "$cmd" in
				config)
					[[ "$subcmd" == "cleanyaml" ]] && COMPREPLY=($(compgen -W "all $P1list" -- "$cur"))
					;;
				ctrl | control)
					[[ "$subcmd" == "walk" ]] && COMPREPLY=($(compgen -W "true" -- "$cur"))
					[[ "$subcmd" == "head" ]] && [[ "$subsubcmd" == "movetoangle" ]] && COMPREPLY=($(compgen -W "0.25" -- "$cur"))
					[[ "$subcmd" == "head" ]] && [[ "$subsubcmd" == "movetovector" ]] && COMPREPLY=($(compgen -W "1.0" -- "$cur"))
					;;
			esac
			;;
		7)
			case "$cmd" in
				ctrl | control)
					[[ "$subcmd" == "head" ]] && [[ "${subsubcmd:0:6}" == "moveto" ]] && COMPREPLY=($(compgen -W "0.25" -- "$cur"))
					;;
			esac
			;;
		8)
			case "$cmd" in
				ctrl | control)
					[[ "$subcmd" == "head" ]] && [[ "$subsubcmd" == "movetovector" ]] && COMPREPLY=($(compgen -W "0.25" -- "$cur"))
					;;
			esac
			;;
	esac
	[[ -z "$COMPREPLY" ]] && COMPREPLY=($(compgen -o nospace -W "" -- "$cur"))
}

complete -F _nimbro nimbro
# EOF
