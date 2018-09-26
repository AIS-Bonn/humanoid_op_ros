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
BOT="xs0.local"

# Set up environment variables for catkin and ROS
source "$NIMBRO_ROOT/devel/setup.bash"

# Set additional model path for gazebo simulation
NIMBRO_MODEL_PATHS="$NIMBRO_ROOT/src/nimbro/hardware/nimbro_op_gazebo/models"
[[ "$GAZEBO_MODEL_PATH" != *"$NIMBRO_MODEL_PATHS"* ]] && export GAZEBO_MODEL_PATH="$NIMBRO_MODEL_PATHS:$GAZEBO_MODEL_PATH"

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

function _makefirmware() { # Pass the device name as the first parameter, and the servo type as the second parameter (e.g. _makefirmware CM740 MX)
	echo "$PATH" | grep -q "/opt/gcc-arm/bin" || PATH="$PATH:/opt/gcc-arm/bin"
	make clean
	failed="false"
	local device servo
	[[ -n "$1" ]] && device="DEVICE=$1" || device=""
	[[ -n "$2" ]] && servo="SERVOS=$2" || servo=""
	make $device $servo || failed="true"
	if [[ "$failed" != "false" ]]; then
		echo 'Error occurred while trying to build the CM730/CM740 firmware!'
		echo "If the required arm tool binary is not located in /opt/gcc-arm/bin,"
		echo "then please add its actual location to the PATH variable."
		return 1
	fi
	echo
	return 0
}

function _callservice() { # Example: _callservice /nimbro_op_interface/attEstCalibrate robotcontrol
	if rosservice list &>/dev/null; then
		echo "Calling service: $1"
		rosservice call "$1" "${@:3}" || echo "Service call failed! Is the $2 node running?"
	else
		echo "Could not list the available ROS services, is a roscore running?"
	fi
}

function _setGCIP() { # Example: _setGCIP OR _setGCIP xs4.local
	( # Start a subshell to have only a temporary change of the ROS master
		local target="$1"
		if [[ "$target" == "xs0" ]] || [[ "$target" == "xs0.local" ]]; then
			target="localhost"
		fi
		if [[ -n "$target" ]]; then
			ROS_MASTER_URI=http://"$target":11311
			ROS_HOSTNAME=$(hostname).local
		fi
		local curhost="${ROS_MASTER_URI#http://}"
		curhost="${curhost%:11311}"
		echo "Setting the server IP of the game controller..."
		local serverIP="$(rosservice call /config_server/get_parameter "name: '/game_controller/serverIP'")"
		serverIP="${serverIP#value: }"
		echo "Server IP was:    $serverIP"
		rosservice call /config_server/set_parameter "{name: '/game_controller/useLastServerIP', value: '1', no_notify: '0'}" >/dev/null
		serverIP="$(rosservice call /config_server/get_parameter "name: '/game_controller/serverIP'")"
		serverIP="${serverIP#value: }"
		echo "Server IP is now: $serverIP"
	)
}

function nimbro() {
	local LIGHT_RED="$(echo -e "\E[1;31m")"
	local LIGHT_CYAN="$(echo -e "\E[1;36m")"
	local NO_COLOUR="$(echo -e "\E[0m")"
	local oldPath="$(pwd)"
	cd "$NIMBRO_ROOT"
	case "$1" in
		"")
			;;
		make)
			time catkin_make "${@:2}" -DCMAKE_INSTALL_PREFIX="$INSTALLPATH" -DCMAKE_BUILD_TYPE=RelWithDebInfo
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
			local target="$2"
			if [[ "$target" == "xs0" ]] || [[ "$target" == "xs0.local" ]]; then
				target="localhost"
			fi
			if nimbro make install; then
				find /nimbro -mindepth 1 -name "*~" -type f -printf "Removing file %p\n" -delete
				if [[ "$target" != "localhost" ]]; then
					[[ -z "$target" ]] && target="$BOT"
					rsync -avz --delete --exclude='share/launch/config/vision/samples/*' /nimbro/ nimbro@"$target":/nimbro
					if [[ $? -ne 0 ]]; then
						echo $LIGHT_CYAN"############################################################"$NO_COLOUR
						echo $LIGHT_CYAN"Careful: The rsync failed so the code has NOT been deployed"'!'$NO_COLOUR
						echo $LIGHT_CYAN"############################################################"$NO_COLOUR
					fi
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
				servo="$4"
				_makefirmware "$device" "$servo" || return 1
				echo "Output hex file located at:"
				echo "$(pwd)/CM730.hex"
				echo
			elif [[ -z "$2" ]] || [[ "$2" == "robot" ]]; then
				if [[ -n "$3" ]]; then
					localbot="$3"
				else
					localbot="$BOT"
				fi
				if [[ "$localbot" == "localhost" ]] || [[ "$localbot" == "xs0" ]] || [[ "$localbot" == "xs0.local" ]]; then
					echo "You are trying to flash your own localhost microcontroller as a robot, use nimbro flash direct instead..."
					return 1
				fi
				localbot="${localbot%% *}"
				echo "Remaking the CM730 firmware and flashing it onto the robot $localbot..."
				modelnumberH=
				[[ "$4" == "CM730" ]] && modelnumberH="0x73"
				[[ "$4" == "CM740" ]] && modelnumberH="0x74"
				modelnumberL=
				[[ "$5" == "MX" ]] && modelnumberL="01"
				[[ "$5" == "X" ]] && modelnumberL="02"
				modelnumber="$modelnumberH$modelnumberL"
				if [[ -z "$modelnumberH" ]] || [[ -z "$modelnumberL" ]]; then
					echo
					echo "Getting board model number using dynatool over ssh..."
					modelnumber="$(ssh nimbro@$localbot 'bash -ic '"'"'dynatool --device=/dev/cm730 "--exec=dev --pure 200;read --pure 0"'"'"'' |& grep -v 'bash:')"
					echo "Dynatool says the board is: $modelnumber"
					echo
				fi
				if [[ "$modelnumber" == "0x7300" ]] || [[ "$modelnumber" == "0x7301" ]]; then
					_makefirmware "CM730" "MX" || return 1
				elif [[ "$modelnumber" == "0x7302" ]]; then
					_makefirmware "CM730" "X" || return 1
				elif [[ "$modelnumber" == "0x7400" ]] || [[ "$modelnumber" == "0x7401" ]]; then
					_makefirmware "CM740" "MX" || return 1
				elif [[ "$modelnumber" == "0x7402" ]]; then
					_makefirmware "CM740" "X" || return 1
				else
					echo "Unrecognised model number '$modelnumber'! Don't know how to flash this device."
					read -p "Flash anyway with CM740 MX firmware (y/N)? " response 2>&1
					response="${response:0:1}"
					if [[ "${response,,}" != "y" ]]; then
						echo "No => Stopping here then."
						return 0
					fi
					_makefirmware "CM740" "MX" || return 1
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
				modelnumberH=
				[[ "$4" == "CM730" ]] && modelnumberH="0x73"
				[[ "$4" == "CM740" ]] && modelnumberH="0x74"
				modelnumberL=
				[[ "$5" == "MX" ]] && modelnumberL="01"
				[[ "$5" == "X" ]] && modelnumberL="02"
				modelnumber="$modelnumberH$modelnumberL"
				if [[ -z "$modelnumberH" ]] || [[ -z "$modelnumberL" ]]; then
					echo
					echo "Getting board model number using dynatool..."
					modelnumber="$(dynatool "--device=$CMDEVICE" "--exec=dev --pure 200;read --pure 0")"
					echo "Dynatool says the board is: $modelnumber"
					echo
				fi
				if [[ "$modelnumber" == "0x7300" ]] || [[ "$modelnumber" == "0x7301" ]]; then
					_makefirmware "CM730" "MX" || return 1
				elif [[ "$modelnumber" == "0x7302" ]]; then
					_makefirmware "CM730" "X" || return 1
				elif [[ "$modelnumber" == "0x7400" ]] || [[ "$modelnumber" == "0x7401" ]]; then
					_makefirmware "CM740" "MX" || return 1
				elif [[ "$modelnumber" == "0x7402" ]]; then
					_makefirmware "CM740" "X" || return 1
				else
					echo "Unrecognised model number '$modelnumber'! Don't know how to flash this device."
					read -p "Flash anyway with CM740 MX firmware (y/N)? " response 2>&1
					response="${response:0:1}"
					if [[ "${response,,}" != "y" ]]; then
						echo "No => Stopping here then."
						return 0
					fi
					_makefirmware "CM740" "MX" || return 1
				fi
				FIRM_INST="$NIMBRO_ROOT/misc/tools/firmware_installer/firmware_installer"
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
			safePlace="$(mktemp -d /tmp/remake-all-XXXXXX)" || { echo "Error: Failed to create temporary directory with mktemp"'!'; return 1; }
			cp -r "$NIMBRO_ROOT/build/"{.project,.pydevproject,.cproject} "$safePlace" 2>/dev/null
			rm -rf "$NIMBRO_ROOT/build" "$NIMBRO_ROOT/devel"
			time catkin_make "${@:2}" -DCMAKE_INSTALL_PREFIX="$INSTALLPATH" -DCMAKE_BUILD_TYPE=RelWithDebInfo
			cp -r "$safePlace/"{.project,.pydevproject,.cproject} "$NIMBRO_ROOT/build" 2>/dev/null
			rm -rf "$safePlace"
			;;
		eclipse)
			echo "Generate eclipse project ..."
			catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"
			awk -f $(rospack find mk)/eclipse.awk build/.project > build/.project_with_env && mv build/.project_with_env build/.project
			mkdir -p build/.settings
			printf '<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n<project>\n<configuration id="org.eclipse.cdt.core.default.config.1" name="Configuration">\n		<extension point="org.eclipse.cdt.core.LanguageSettingsProvider">\n			<provider copy-of="extension" id="org.eclipse.cdt.ui.UserLanguageSettingsProvider"/>\n			<provider-reference id="org.eclipse.cdt.core.ReferencedProjectsLanguageSettingsProvider" ref="shared-provider"/>\n			<provider-reference id="org.eclipse.cdt.core.PathEntryScannerInfoLanguageSettingsProvider" ref="shared-provider"/>\n			<provider class="org.eclipse.cdt.managedbuilder.language.settings.providers.GCCBuiltinSpecsDetector" console="false" env-hash="-1547573120553910201" id="org.eclipse.cdt.managedbuilder.core.GCCBuiltinSpecsDetector" keep-relative-paths="false" name="CDT GCC Built-in Compiler Settings" parameter="${COMMAND} ${FLAGS} -E -P -v -dD &quot;${INPUTS}&quot;">\n				<language-scope id="org.eclipse.cdt.core.gcc"/>\n				<language-scope id="org.eclipse.cdt.core.g++"/>\n			</provider>\n		</extension>\n	</configuration>\n</project>\n' > build/.settings/language.settings.xml
			echo "Your eclipse project is in build/.project"
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
				"net" | "nimbro_network")
					cd "$NIMBRO_ROOT/src/nimbro_network"
					;;
				"cat" | "catch_ros")
					cd "$NIMBRO_ROOT/src/catch_ros"
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
					echo "nimbro_network:       net, nimbro_network"
					echo "catch_ros:            cat, catch_ros"
					echo "dynalib:              dyn, dynalib"
					echo "rosmon:               mon, rosmon"
					cd "$NIMBRO_ROOT/src/nimbro"
					;;
			esac
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
			local target="$2"
			if [[ "$target" == "xs0" ]] || [[ "$target" == "xs0.local" ]]; then
				target="localhost"
			fi
			if [[ -z "$target" ]]; then
				echo "Currently selected robot is '$BOT'."
				return 0
			fi
			if ! ping -c1 "$target" &>/dev/null; then
				echo "Could not resolve host name '$target'."
				echo "Selecting '$target' as host anyway..."
			fi
			BOT="$target"
			echo "Robot '$BOT' selected."
			;;
		behaviour)
			case "$2" in
				"startstop")
					_callservice /config_server/set_parameter config_server "{name: '/nimbro_op_interface/button/pressButton1', value: '1', no_notify: '0'}"
				;;
				"setGCIP")
					_setGCIP "$3"
				;;
				"visualiseClear")
					_callservice /walk_and_kick/visualiseClear walk_and_kick "{}"
				;;
				"visualiseDBH")
					local field="$4"
					[[ "$field" != "bonn" ]] && field="teensize"
					local side="$5"
					local sidesign=
					if [[ "$side" == "blue" ]]; then
						side="false"
						sidesign="-"
					else
						side="true"
					fi
					local heading="$6"
					[[ -z "$heading" ]] && heading="0.0"
					case "$3" in
						"clear")
							_callservice /walk_and_kick/visualiseDBH walk_and_kick "{type: 0, playOnYellow: $side, ballXMin: 0.0, ballXMax: 0.0, ballXNum: 0, ballYMin: 0.0, ballYMax: 0.0, ballYNum: 0, compassHeading: $heading}"
						;;
						"compass")
							[[ "$field" == "bonn" ]] && _callservice /walk_and_kick/visualiseDBH walk_and_kick "{type: 2, playOnYellow: $side, ballXMin: -3.0, ballXMax: 3.0, ballXNum: 70, ballYMin: -2.3, ballYMax: 2.3, ballYNum: 54, compassHeading: $heading}"
							[[ "$field" == "teensize" ]] && _callservice /walk_and_kick/visualiseDBH walk_and_kick "{type: 2, playOnYellow: $side, ballXMin: -4.8, ballXMax: 4.8, ballXNum: 70, ballYMin: -3.2, ballYMax: 3.2, ballYNum: 47, compassHeading: $heading}"
						;;
						"corner")
							[[ "$field" == "bonn" ]] && _callservice /walk_and_kick/visualiseDBH walk_and_kick "{type: 1, playOnYellow: $side, ballXMin: $sidesign""1.5, ballXMax: $sidesign""3.0, ballXNum: 78, ballYMin: 0.0, ballYMax: 2.3, ballYNum: 120, compassHeading: $heading}"
							[[ "$field" == "teensize" ]] && _callservice /walk_and_kick/visualiseDBH walk_and_kick "{type: 1, playOnYellow: $side, ballXMin: $sidesign""2.4, ballXMax: $sidesign""4.8, ballXNum: 90, ballYMin: 0.0, ballYMax: 3.2, ballYNum: 120, compassHeading: $heading}"
						;;
						"posts")
							[[ "$field" == "bonn" ]] && _callservice /walk_and_kick/visualiseDBH walk_and_kick "{type: 1, playOnYellow: $side, ballXMin: $sidesign""1.5, ballXMax: $sidesign""3.0, ballXNum: 39, ballYMin: -2.3, ballYMax: 2.3, ballYNum: 120, compassHeading: $heading}"
							[[ "$field" == "teensize" ]] && _callservice /walk_and_kick/visualiseDBH walk_and_kick "{type: 1, playOnYellow: $side, ballXMin: $sidesign""2.4, ballXMax: $sidesign""4.8, ballXNum: 45, ballYMin: -3.2, ballYMax: 3.2, ballYNum: 120, compassHeading: $heading}"
						;;
						"half")
							[[ "$field" == "bonn" ]] && _callservice /walk_and_kick/visualiseDBH walk_and_kick "{type: 1, playOnYellow: $side, ballXMin: 0.0, ballXMax: $sidesign""3.0, ballXNum: 47, ballYMin: -2.3, ballYMax: 2.3, ballYNum: 70, compassHeading: $heading}"
							[[ "$field" == "teensize" ]] && _callservice /walk_and_kick/visualiseDBH walk_and_kick "{type: 1, playOnYellow: $side, ballXMin: 0.0, ballXMax: $sidesign""4.8, ballXNum: 53, ballYMin: -3.2, ballYMax: 3.2, ballYNum: 70, compassHeading: $heading}"
						;;
						"centre")
							[[ "$field" == "bonn" ]] && _callservice /walk_and_kick/visualiseDBH walk_and_kick "{type: 1, playOnYellow: $side, ballXMin: -1.5, ballXMax: 1.5, ballXNum: 47, ballYMin: -2.3, ballYMax: 2.3, ballYNum: 70, compassHeading: $heading}"
							[[ "$field" == "teensize" ]] && _callservice /walk_and_kick/visualiseDBH walk_and_kick "{type: 1, playOnYellow: $side, ballXMin: -2.4, ballXMax: 2.4, ballXNum: 53, ballYMin: -3.2, ballYMax: 3.2, ballYNum: 70, compassHeading: $heading}"
						;;
						"whole" | "")
							[[ "$field" == "bonn" ]] && _callservice /walk_and_kick/visualiseDBH walk_and_kick "{type: 1, playOnYellow: $side, ballXMin: -3.0, ballXMax: 3.0, ballXNum: 70, ballYMin: -2.3, ballYMax: 2.3, ballYNum: 54, compassHeading: $heading}"
							[[ "$field" == "teensize" ]] && _callservice /walk_and_kick/visualiseDBH walk_and_kick "{type: 1, playOnYellow: $side, ballXMin: -4.8, ballXMax: 4.8, ballXNum: 70, ballYMin: -3.2, ballYMax: 3.2, ballYNum: 47, compassHeading: $heading}"
						;;
						*)
							echo "Unrecognised visualiseDBH command."
							echo "Usage: nimbro behaviour visualiseDBH [clear|compass|corner|posts|half|centre|whole] [teensize|bonn] [yellow|blue] [HEADING]"
						;;
					esac
				;;
				"visualiseDbApp")
					local robotangle="$3"
					local rightfoot="$4"
					[[ -z "$robotangle" ]] && robotangle="0.0"
					[[ "$rightfoot" != "leftFoot" ]] && rightfoot="true" || rightfoot="false"
					_callservice /walk_and_kick/visualiseDbApp walk_and_kick "{useRightFoot: $rightfoot, robotAngle: $robotangle, numX: 50, numY: 25, gridSize: 0.04}"
				;;
				"visualiseGcvXY")
					_callservice /walk_and_kick/visualiseGcvXY walk_and_kick "{}"
				;;
				*)
					echo "Unrecognised behaviour command."
					echo "Usage: nimbro behaviour {startstop|setGCIP|visualiseClear|visualiseDBH|visualiseDbApp|visualiseGcvXY}"
				;;
			esac
			;;
		ctrl | control)
			case "$2" in
				"facetracking")
					( # Start a subshell to have only a temporary change of the ROS master
						hostvar="$4"
						if [[ "$hostvar" == "xs0" ]] || [[ "$hostvar" == "xs0.local" ]]; then
							hostvar="localhost"
						fi
						if [[ -n "$hostvar" ]]; then
							ROS_MASTER_URI=http://"$hostvar":11311
							ROS_HOSTNAME=$(hostname).local
						fi
						local curhost="${ROS_MASTER_URI#http://}"
						curhost="${curhost%:11311}"
						if [[ "$3" == "disable" ]]; then
							echo "Disabling face tracking..."
							rostopic pub /vision/face_tracker std_msgs/Bool "data: false" --once
						elif [[ "$3" == "enable" ]]; then
							echo "Enabling face tracking..."
							rostopic pub /vision/face_tracker std_msgs/Bool "data: true" --once
						else
							echo "Usage: nimbro ctrl facetracking {disable|enable} [robot]"
						fi
					)
				;;
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
				"press")
					case "$3" in
						"fade")
							_callservice /config_server/set_parameter config_server "{name: '/nimbro_op_interface/button/pressButton0', value: '1', no_notify: '0'}"
						;;
						"mode")
							_callservice /config_server/set_parameter config_server "{name: '/nimbro_op_interface/button/pressButton1', value: '1', no_notify: '0'}"
						;;
						"reset")
							_callservice /config_server/set_parameter config_server "{name: '/nimbro_op_interface/button/pressButton2', value: '1', no_notify: '0'}"
						;;
						*)
							echo "Unrecognised button to press."
							echo "Usage: nimbro {control|ctrl} press {fade|mode|reset}"
						;;
					esac
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
					echo "Usage: nimbro {control|ctrl} {facetracking|fade|halt|head|press|walk} [...]"
				;;
			esac
			;;
		calib)
			case "$2" in
				"heading")
					( # Start a subshell to have only a temporary change of the ROS master
						local target="$3"
						if [[ "$target" == "xs0" ]] || [[ "$target" == "xs0.local" ]]; then
							target="localhost"
						fi
						if [[ -n "$target" ]]; then
							ROS_MASTER_URI=http://"$target":11311
							ROS_HOSTNAME=$(hostname).local
						fi
						local curhost="${ROS_MASTER_URI#http://}"
						curhost="${curhost%:11311}"
						echo "Calibration of the heading via the attitude estimator magnetometer calibration vector:"
						echo "Put the robot on the field exactly upright and facing the positive goal..."
						read -p "Continue with heading calibration of $curhost (Y/n)? " response 2>&1
						response="${response:0:1}"
						[[ "${response,,}" == "n" ]] && echo "No => Stopping here then." || _callservice /nimbro_op_interface/attEstCalibrate robotcontrol
					)
				;;
				"magnetometer")
					( # Start a subshell to have only a temporary change of the ROS master
						[[ "$3" == "warpAdd" ]] && hostvar="$5" || hostvar="$4"
						if [[ "$hostvar" == "xs0" ]] || [[ "$hostvar" == "xs0.local" ]]; then
							hostvar="localhost"
						fi
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
							[[ "${response,,}" == "n" ]] && echo "No => Stopping here then." || _callservice /nimbro_op_interface/magFilter/startCalibration robotcontrol
						elif [[ "$3" == "stop2D" ]]; then
							echo "Stopping the 2D magnetometer calibration of $curhost..."
							_callservice /nimbro_op_interface/magFilter/stopCalibration2D robotcontrol
						elif [[ "$3" == "stop3D" ]]; then
							echo "Stopping the 3D magnetometer calibration of $curhost..."
							_callservice /nimbro_op_interface/magFilter/stopCalibration3D robotcontrol
						elif [[ "$3" == "abort" ]]; then
							echo "Aborting the magnetometer calibration of $curhost..."
							_callservice /nimbro_op_interface/magFilter/abortCalibration robotcontrol
						elif [[ "$3" == "show" ]]; then
							echo "Showing the current magnetometer calibration of $curhost..."
							_callservice /nimbro_op_interface/magFilter/showCalibration robotcontrol
						elif [[ "$3" == "warpClear" ]]; then
							echo "Clearing the magnetometer warp parameter string, resulting in an identity yaw warp transform..."
							_callservice /nimbro_op_interface/magFilter/warpClearPoints robotcontrol
						elif [[ "$3" == "warpAdd" ]]; then
							echo "Adding reference point that robot is currently at a true yaw (CCW from +ve goal) of $4 degrees..."
							_callservice /nimbro_op_interface/magFilter/warpAddPoint robotcontrol "$4"
						else
							echo "Usage: nimbro calib magnetometer {abort|show|start|stop2D|stop3D|warpClear|warpAdd} [value] [robot]"
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
			roscd launch/config
			case "$2" in
				"")
				;;
				"list")
					proctemp="$(rostopic echo -n 1 /config_server/parameter_list | grep name | sort)"
					if [[ -z "$proctemp" ]]; then
						echo "Unable to retrieve parameter list from config server"'!'
						return 1
					fi
					configlist=()
					while IFS= read -r -d $'\n' line; do
						[[ -z "$line" ]] && continue
						configlist+=("/${line#*/}")
					done <<< "$proctemp"
					echo "$(IFS=$'\n'; echo "${configlist[*]}")"
				;;
				"open")
					local LAUNCH="$(rospack find launch)"
					local robotname="$3"
					[[ -z "$robotname" ]] && robotname="xs0"
					robotsuffix=".${robotname#*.}"
					[[ "$robotsuffix" == ".$robotname" ]] && robotsuffix= || robotname="${robotname%$robotsuffix}"
					local filepath="$LAUNCH/config/config_$robotname.yaml$robotsuffix"
					echo "Opening config file: $filepath..."
					xdg-open "$filepath"
				;;
				"find")
					proctemp="$(rostopic echo -n 1 /config_server/parameter_list | grep name | sort)"
					if [[ -z "$proctemp" ]]; then
						echo "Unable to retrieve parameter list from config server"'!'
						return 1
					fi
					configlist=()
					while IFS= read -r -d $'\n' line; do
						[[ -z "$line" ]] && continue
						configlist+=("/${line#*/}")
					done <<< "$proctemp"
					if [[ -n "$3" ]]; then
						echo "$(IFS=$'\n'; echo "${configlist[*]}")" | grep "$3"
					else
						echo "$(IFS=$'\n'; echo "${configlist[*]}")"
					fi
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
					echo "Showing dead config parameters in '$configPath'... (see config server console)"
					rosservice call /config_server/show_dead_vars "{configPath: '$configPath'}"
				;;
				"cleanyaml")
					local LAUNCH="$(rospack find launch)"
					if [[ -z "$3" ]]; then
						echo "Please specify the robot(s) to clean the config file(s) for as arguments.";
					elif [[ "$3" == "all" ]] || [[ "$4" == "all" ]] || [[ "$5" == "all" ]] || [[ "$6" == "all" ]]; then
						compgen -G "$LAUNCH/config/config_*.yaml" >/dev/null && python "$LAUNCH/config/cleanYaml.py" "$LAUNCH/config/"config_*.yaml
						compgen -G "$LAUNCH/config/config_*.yaml.*" >/dev/null && python "$LAUNCH/config/cleanYaml.py" "$LAUNCH/config/"config_*.yaml.*
					else
						local filelist=()
						while [[ -n "$3" ]]; do
							local src="$3"
							local srcbase="${src%%.*}"
							src="config_$srcbase.yaml${src#$srcbase}"
							filelist+=("$LAUNCH/config/$src")
							shift
						done
						python "$LAUNCH/config/cleanYaml.py" "${filelist[@]}"
					fi
				;;
				"listyaml")
					local LAUNCH="$(rospack find launch)"
					if [[ -z "$3" ]]; then
						echo "Please specify the robot to list the config file for as an argument."
					else
						local depth="$4"
						python "$LAUNCH/config/listYaml.py" "$LAUNCH/config/config_$3.yaml" "$depth"
					fi
				;;
				"cpyaml")
					local LIGHT_RED="$(echo -e "\E[1;31m")"
					local NO_COLOUR="$(echo -e "\E[0m")"
					local LAUNCH="$(rospack find launch)"
					local src="$3"
					local dst="$4"
					local element="$5"
					if [[ -z "$src" ]] || [[ -z "$dst" ]]; then
						echo $LIGHT_RED"You need to specify source and destination config files"'!'$NO_COLOUR
						return 1
					fi
					if [[ -z "$element" ]]; then
						echo $LIGHT_RED"No config parameter element specified => Use 'all' if you want to copy everything"'!'$NO_COLOUR
						return 1
					fi
					if [[ "$src" == "all" ]]; then
						echo $LIGHT_RED"The source config file cannot be 'all'"'!'$NO_COLOUR
						return 1
					fi
					local srcbase="${src%%.*}"
					src="config_$srcbase.yaml${src#$srcbase}"
					if [[ "$dst" == "all" ]]; then
						for yamlfile in "$LAUNCH/config/"config_*.yaml; do
							local filename="$(basename "$yamlfile")"
							[[ "$filename" == "config_*.yaml" ]] && continue
							if [[ "$src" != "$filename" ]]; then
								python "$LAUNCH/config/copyConfig.py" -b "$LAUNCH/config" -s "$src" -d "$filename" -e "$element"
							fi
						done
						for yamlfile in "$LAUNCH/config/"config_*.yaml.*; do
							local filename="$(basename "$yamlfile")"
							[[ "$filename" == "config_*.yaml.*" ]] && continue
							if [[ "$src" != "$filename" ]]; then
								python "$LAUNCH/config/copyConfig.py" -b "$LAUNCH/config" -s "$src" -d "$filename" -e "$element"
							fi
						done
					else
						local dstbase="${dst%%.*}"
						dst="config_$dstbase.yaml${dst#$dstbase}"
						if [[ "$src" == "$dst" ]]; then
							echo $LIGHT_RED"Are you trying to copy from and to the same yaml...?"'!'$NO_COLOUR
							return 1
						fi
						python "$LAUNCH/config/copyConfig.py" -b "$LAUNCH/config" -s "$src" -d "$dst" -e "$element"
					fi
				;;
				"rmyaml")
					local LIGHT_RED="$(echo -e "\E[1;31m")"
					local NO_COLOUR="$(echo -e "\E[0m")"
					local LAUNCH="$(rospack find launch)"
					local src="$3"
					local element="$4"
					if [[ -z "$src" ]]; then
						echo $LIGHT_RED"You need to specify a source config file"'!'$NO_COLOUR
						return 1
					fi
					if [[ -z "$element" ]]; then
						echo $LIGHT_RED"No config parameter element specified => Use 'all' if you want to remove everything"'!'$NO_COLOUR
						return 1
					fi
					if [[ "$src" == "all" ]]; then
						for yamlfile in "$LAUNCH/config/"config_*.yaml; do
							local filename="$(basename "$yamlfile")"
							[[ "$filename" == "config_*.yaml" ]] && continue
							python "$LAUNCH/config/removeConfig.py" -b "$LAUNCH/config" -s "$filename" -e "$element"
						done
						for yamlfile in "$LAUNCH/config/"config_*.yaml.*; do
							local filename="$(basename "$yamlfile")"
							[[ "$filename" == "config_*.yaml.*" ]] && continue
							python "$LAUNCH/config/removeConfig.py" -b "$LAUNCH/config" -s "$filename" -e "$element"
						done
					else
						local srcbase="${src%%.*}"
						src="config_$srcbase.yaml${src#$srcbase}"
						python "$LAUNCH/config/removeConfig.py" -b "$LAUNCH/config" -s "$src" -e "$element"
					fi
				;;
				"compare")
					local LIGHT_RED="$(echo -e "\E[1;31m")"
					local NO_COLOUR="$(echo -e "\E[0m")"
					local LAUNCH="$(rospack find launch)"
					local src="$3"
					local dst="$4"
					local element="$5"
					if [[ -z "$src" ]] || [[ -z "$dst" ]]; then
						echo $LIGHT_RED"You need to specify source and destination config files"'!'$NO_COLOUR
						return 1
					fi
					local srcbase="${src%%.*}"
					src="config_$srcbase.yaml${src#$srcbase}"
					local dstbase="${dst%%.*}"
					dst="config_$dstbase.yaml${dst#$dstbase}"
					python "$LAUNCH/config/compareConfig.py" -b "$LAUNCH/config" -s "$src" -d "$dst" -e "$element"
				;;
				"kompare")
					local LAUNCH="$(rospack find launch)"
					"$LAUNCH/config/kompareConfig.sh" "$3" "$4"
				;;
				"retrieve")
					local target="$BOT"
					[[ -n "$3" ]] && target="$3"
					if [[ "$target" == "localhost" ]] || [[ "$target" == "xs0" ]] || [[ "$target" == "xs0.local" ]]; then
						echo "You are trying to retrieve configs from yourself...?"
						return 1
					fi
					while ! ping -c1 "$target" &>/dev/null; do sleep 0.5s; done
					local targetname="$(ssh "nimbro@$target" 'hostname')" 2>/dev/null
					targetname="${targetname%%\\*}"
					targetname="${targetname%%/*}"
					local LAUNCH="$(rospack find launch)"
					local LIGHT_RED="$(echo -e "\E[1;31m")"
					local LIGHT_GREEN="$(echo -e "\E[1;32m")"
					local NO_COLOUR="$(echo -e "\E[0m")"
					local robots=(xs0 xs1 xs2 xs3 xs4 xs5 xs6 xs7 xs8 xs9)
					if [[ -z "$4" ]]; then
						echo $LIGHT_GREEN"Retrieving config.yaml's..."$NO_COLOUR
						local proctemp="$(ssh "nimbro@$target" 'find -P /nimbro/share/launch/config -ignore_readdir_race -noleaf -nowarn -type f -name "*.yaml" -print | sort')" 2>/dev/null
						local filelist=()
						while IFS= read -r -d $'\n' line; do
							[[ -z "$line" ]] && continue
							for robot in "${robots[@]}"; do
								[[ "$robot" == "$targetname" ]] && continue
								[[ "$line" =~ ^.*_"$robot"\..*$ ]] && continue 2
							done
							filelist+=("nimbro@$target:$line")
						done <<< "$proctemp"
						if [[ "${#filelist[@]}" -gt 0 ]]; then
							rsync -avz --checksum --progress --human-readable "${filelist[@]}" "$LAUNCH/config/"
						else
							echo "Nothing to retrieve here from $target..."
						fi
						echo
						echo $LIGHT_GREEN"Retrieving vision configs..."$NO_COLOUR
						local proctemp="$(ssh "nimbro@$target" 'find -P /nimbro/share/launch/config/vision -ignore_readdir_race -noleaf -nowarn -maxdepth 1 -type f -print | sort')" 2>/dev/null
						local filelist=("nimbro@$target:/nimbro/share/launch/config/vision/samples")
						while IFS= read -r -d $'\n' line; do
							[[ -z "$line" ]] && continue
							for robot in "${robots[@]}"; do
								[[ "$robot" == "$targetname" ]] && continue
								[[ "$line" =~ ^.*_"$robot"\..*$ ]] && continue 2
							done
							filelist+=("nimbro@$target:$line")
						done <<< "$proctemp"
						if [[ "${#filelist[@]}" -gt 0 ]]; then
							rsync -avz --checksum --progress --human-readable "${filelist[@]}" "$LAUNCH/config/vision/"
						else
							echo "Nothing to retrieve here from $target..."
						fi
					elif [[ "$4" == "backup" ]]; then
						local LOG_DIR="/var/log/nimbro"
						local BACKUP_DIR="$LAUNCH/config/backup"
						echo
						echo $LIGHT_GREEN"Retrieving latest config.yaml backup to $BACKUP_DIR..."$NO_COLOUR
						local latestconfig="$(ssh "nimbro@$target" 'find -P "'"$LOG_DIR/config_server"'" -ignore_readdir_race -noleaf -nowarn -type f -name "'"config_$targetname_*.yaml"'" -print 2>/dev/null | sort | tail -n 1')" 2>/dev/null
						if [[ -n "$latestconfig" ]]; then
							mkdir -p "$BACKUP_DIR" && rsync -avz --progress --human-readable "nimbro@$target:$latestconfig" "$BACKUP_DIR/"
						else
							echo $LIGHT_RED"No config backup files were found on $target"'!'$NO_COLOUR
						fi
					else
						echo "Unknown specification '$4' of what to retrieve"
						echo "Usage: nimbro config retrieve [TARGET] [{backup}]"
					fi
				;;
				"setGCIP")
					_setGCIP "$3"
				;;
				*)
					echo "Unrecognised config command"
					echo "Usage: nimbro config {list|reset|load|save|open|compare|kompare|get|set|showdead|cleanyaml|cpyaml|listyaml|rmyaml|retrieve|setGCIP} [...]"
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
			if [[ -n "$2" ]]; then
				local host="$2"
				if [[ "$host" == "reset" ]] || [[ "$host" == "xs0" ]] || [[ "$host" == "xs0.local" ]]; then
					host="localhost"
				fi
				if ! ping -c1 "$host" &>/dev/null; then
					echo "Could not resolve host name '$host'."
					return 1
				fi
				export ROS_HOSTNAME="$(hostname).local"
				export ROS_MASTER_URI="http://$host:11311"
				if rostopic list > /dev/null; then
					echo "Connected."
				else
					echo "Could not connect to ROS master running at '$host'"
					echo "Setting it as master anyway."
				fi
				BOT="$host"
				if [[ "$host" == "localhost" ]]; then # To avoid localhost password request: cat ~/.ssh/id_rsa.pub >> ~/.ssh/authorized_keys && chmod og-wx ~/.ssh/authorized_keys
					hostRobotSet="$(ssh "localhost" 'bash -ic '"'"'echo "$NIMBRO_ROBOT_TYPE" && echo "$NIMBRO_ROBOT_NAME" && echo "$NIMBRO_ROBOT_VARIANT"'"'"' 2>/dev/null')"
				else
					hostRobotSet="$(ssh "nimbro@$host" 'bash -ic '"'"'echo "$NIMBRO_ROBOT_TYPE" && echo "$NIMBRO_ROBOT_NAME" && echo "$NIMBRO_ROBOT_VARIANT"'"'"' 2>/dev/null')"
				fi
				local hostRobotType="$(sed -n 1p <<< "$hostRobotSet")"
				local hostRobotName="$(sed -n 2p <<< "$hostRobotSet")"
				local hostRobotVariant="$(sed -n 3p <<< "$hostRobotSet")"
				[[ -n "$hostRobotType" ]] && export NIMBRO_ROBOT_TYPE="$hostRobotType"
				[[ -n "$hostRobotName" ]] && export NIMBRO_ROBOT_NAME="$hostRobotName"
				[[ -n "$hostRobotVariant" ]] && export NIMBRO_ROBOT_VARIANT="$hostRobotVariant"
			fi
			echo "Current ROS hostname: $ROS_HOSTNAME"
			echo "Current ROS master:   $ROS_MASTER_URI"
			echo "Current BOT:          $BOT"
			echo "Current NIMBRO_ROBOT_TYPE:    $NIMBRO_ROBOT_TYPE"
			echo "Current NIMBRO_ROBOT_NAME:    $NIMBRO_ROBOT_NAME"
			echo "Current NIMBRO_ROBOT_VARIANT: $NIMBRO_ROBOT_VARIANT"
			;;
		ssh)
			local bott="$2"
			if [[ "$bott" == "localhost" ]] || [[ "$bott" == "xs0" ]] || [[ "$bott" == "xs0.local" ]]; then
				echo "You are trying to ssh into yourself...?"
				return 1
			fi
			if [[ -z "$bott" ]]; then
				while ! ping -c1 "$BOT" &>/dev/null; do sleep 0.5s; done
			    ssh nimbro@"$BOT"
			else
				local target="$bott"
				while ! ping -c1 "$target" &>/dev/null; do sleep 0.5s; done
				ssh nimbro@"$target"
			fi
			;;
		servos)
			cat "$(roscd dynalib/robots && pwd)/NOP.robot"
			;;
		env)
			xdg-open "${BASH_SOURCE[0]}"
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
  help        Display this help message
  host        Use HOST as the ROS master, e.g. nimbro host xs4.local
  make        Run catkin_make with correct arguments in the correct directory
  make-doc    Compile the doxygen documentation (use 'make-doc open' to automatically open the html)
  make-docv   Verbose compilation of the doxygen documentation (see make-doc)
  remake-all  Hard clean build and devel folders then run catkin_make to remake entire project
  eclipse     Make eclipse project for you
  robot       Select a given robot, e.g. nimbro robot xs4.local
  set         Set the environment variables for a particular robot
  sim         Commands to control the robot Gazebo simulation
  source      Change to the nimbro source directory
  src         Alias for 'source'
  ssh         Open an SSH connection to the robot

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
	
	local hostlist="xs0.local xs1.local"
	local hostlistloc="localhost $hostlist"
	local robotlist="xs0 xs1"
	
	COMPREPLY=""
	case "${COMP_CWORD}" in
		1)
			COMPREPLY=($(compgen -W "bot behaviour calib clean config control ctrl deploy doc env flash help host make make-doc make-docv remake-all eclipse robot servos set sim source src ssh" -- "$cur"))
			;;
		2)
			case "$cmd" in
				behaviour)
					COMPREPLY=($(compgen -W "setGCIP startstop visualiseClear visualiseDBH visualiseDbApp visualiseGcvXY" -- "$cur"))
					;;
				calib)
					COMPREPLY=($(compgen -W "heading magnetometer" -- "$cur"))
					;;
				config)
					COMPREPLY=($(compgen -W "cleanyaml compare cpyaml find get kompare list listyaml load open reset retrieve rmyaml save set setGCIP showdead" -- "$cur"))
					;;
				ctrl | control)
					COMPREPLY=($(compgen -W "facetracking fade halt head press walk" -- "$cur"))
					;;
				deploy)
					COMPREPLY=($(compgen -W "clean $hostlistloc" -- "$cur"))
					;;
				doc)
					COMPREPLY=($(compgen -W "nimbro visualization robotcontrol config_server all" -- "$cur"))
					;;
				flash)
					COMPREPLY=($(compgen -W "clean compile direct robot" -- "$cur"))
					;;
				host)
					COMPREPLY=($(compgen -W "$hostlistloc" -- "$cur"))
					;;
				make)
					COMPREPLY=($(compgen -W "--force-cmake clean install run_tests test tests" -- "$cur"))
					;;
				make-doc | make-docv)
					COMPREPLY=($(compgen -W "nimbro visualization robotcontrol config_server all open" -- "$cur"))
					;;
				robot | bot)
					COMPREPLY=($(compgen -W "$hostlistloc" -- "$cur"))
					;;
				set)
					COMPREPLY=($(compgen -W "P1 A1" -- "$cur"))
					;;
				sim)
					COMPREPLY=($(compgen -W "poke" -- "$cur"))
					;;
				source | src)
					local sources="src nimbro visualization robotcontrol config_server dynalib"
					[[ -d "$NIMBRO_ROOT/src/rosmon" ]] && sources+=" mon rosmon"
					[[ -d "$NIMBRO_ROOT/src/nimbro_network" ]] && sources+=" net network"
					[[ -d "$NIMBRO_ROOT/src/catch_ros" ]] && sources+=" cat catch_ros"
					COMPREPLY=($(compgen -W "$sources" -- "$cur"))
					;;
				ssh)
					COMPREPLY=($(compgen -W "$hostlist" -- "$cur"))
					;;
			esac
			;;
		3)
			case "$cmd" in
				behaviour)
					[[ "$subcmd" == "setGCIP" ]] && COMPREPLY=($(compgen -W "$hostlist" -- "$cur"))
					[[ "$subcmd" == "visualiseDBH" ]] && COMPREPLY=($(compgen -W "clear compass corner posts half centre whole" -- "$cur"))
					[[ "$subcmd" == "visualiseDbApp" ]] && COMPREPLY=($(compgen -W "0.0 0.5 1.0 1.5 -0.5 -1.0 -1.5" -- "$cur"))
					;;
				calib)
					[[ "$subcmd" == "heading" ]] && COMPREPLY=($(compgen -W "$hostlistloc" -- "$cur"))
					[[ "$subcmd" == "magnetometer" ]] && COMPREPLY=($(compgen -W "abort show start stop2D stop3D warpAdd warpClear" -- "$cur"))
					;;
				config)
					[[ "$subcmd" == "compare" ]] && COMPREPLY=($(compgen -W "$robotlist" -- "$cur"))
					[[ "$subcmd" == "cpyaml" ]] && COMPREPLY=($(compgen -W "$robotlist" -- "$cur"))
					[[ "$subcmd" == "listyaml" ]] && COMPREPLY=($(compgen -W "$robotlist" -- "$cur"))
					[[ "$subcmd" == "rmyaml" ]] && COMPREPLY=($(compgen -W "all $robotlist" -- "$cur"))
					[[ "$subcmd" == "kompare" ]] && COMPREPLY=($(compgen -W "$robotlist" -- "$cur"))
					[[ "$subcmd" == "open" ]] && COMPREPLY=($(compgen -W "$robotlist" -- "$cur"))
					[[ "$subcmd" == "retrieve" ]] && COMPREPLY=($(compgen -W "$hostlist" -- "$cur"))
					[[ "$subcmd" == "setGCIP" ]] && COMPREPLY=($(compgen -W "$hostlist" -- "$cur"))
					if [[ "$subcmd" == "get" ]] || [[ "$subcmd" == "set" ]] || [[ "$subcmd" == "showdead" ]] || [[ "$subcmd" == "find" ]]; then
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
					[[ "$subcmd" == "facetracking" ]] && COMPREPLY=($(compgen -W "enable disable" -- "$cur"))
					[[ "$subcmd" == "fade" ]] && COMPREPLY=($(compgen -W "in out" -- "$cur"))
					[[ "$subcmd" == "press" ]] && COMPREPLY=($(compgen -W "fade mode reset" -- "$cur"))
					[[ "$subcmd" == "walk" ]] && COMPREPLY=($(compgen -W "0.0" -- "$cur"))
					[[ "$subcmd" == "head" ]] && COMPREPLY=($(compgen -W "disable enable movetoangle movetovector relax" -- "$cur"))
					;;
				flash)
					[[ "$subcmd" == "compile" ]] && COMPREPLY=($(compgen -W "CM730 CM740" -- "$cur"))
					if [[ "$subcmd" == "direct" ]]; then
						local usblist=(/dev/ttyUSB*)
						[[ "${usblist[*]}" == "/dev/ttyUSB*" ]] && usblist=()
						COMPREPLY=($(compgen -W "${usblist[*]}" -- "$cur"))
					fi
					[[ "$subcmd" == "robot" ]] && COMPREPLY=($(compgen -W "$hostlist" -- "$cur"))
					;;
				make-doc | make-docv)
					[[ "$subcmd" != "open" ]] && COMPREPLY=($(compgen -W "open" -- "$cur"))
					;;
				set)
					[[ "$subcmd" == "P1" ]] && COMPREPLY=($(compgen -W "$robotlist" -- "$cur"))
					[[ "$subcmd" == "A1" ]] && COMPREPLY=($(compgen -W "xs2 xs4" -- "$cur"))
					;;
			esac
			;;
		4)
			case "$cmd" in
				behaviour)
					[[ "$subcmd" == "visualiseDBH" ]] && [[ "$subsubcmd" != "clear" ]] && COMPREPLY=($(compgen -W "teensize bonn" -- "$cur"))
					[[ "$subcmd" == "visualiseDbApp" ]] && COMPREPLY=($(compgen -W "leftFoot rightFoot" -- "$cur"))
					;;
				calib)
					if [[ "$subcmd" == "magnetometer" ]]; then
						if [[ "$subsubcmd" == "warpAdd" ]]; then
							COMPREPLY=($(compgen -W "0 45 90 180 225 270 315" -- "$cur"))
						else
							COMPREPLY=($(compgen -W "$hostlistloc" -- "$cur"))
						fi
					fi
					;;
				config)
					[[ "$subcmd" == "compare" ]] && COMPREPLY=($(compgen -W "$robotlist" -- "$cur"))
					[[ "$subcmd" == "cpyaml" ]] && COMPREPLY=($(compgen -W "all $robotlist" -- "$cur"))
					[[ "$subcmd" == "listyaml" ]] && COMPREPLY=($(compgen -W "1 2 3 4 5" -- "$cur"))
					[[ "$subcmd" == "kompare" ]] && COMPREPLY=($(compgen -W "$robotlist" -- "$cur"))
					[[ "$subcmd" == "retrieve" ]] && COMPREPLY=($(compgen -W "backup" -- "$cur"))
					if [[ "$subcmd" == "rmyaml" ]]; then
						local configdir="$(rospack find launch)/config"
						local configfile="$configdir/config_$subsubcmd.yaml"
						if [[ "$subsubcmd" == "all" ]]; then
							configfile="$configdir/$(ls -1 "$configdir" | egrep "config_.*.yaml" | sort | head -n 1)"
						fi
						local depth="$(grep -o "/" <<< "$cur" | wc -l)"
						[[ "$depth" == "0" ]] && depth="1"
						local proctemp="$(python "$(rospack find launch)/config/listYaml.py" "$configfile" "$depth" 2>/dev/null | sort)"
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
					[[ "$subcmd" == "facetracking" ]] && COMPREPLY=($(compgen -W "$hostlistloc" -- "$cur"))
					[[ "$subcmd" == "walk" ]] && COMPREPLY=($(compgen -W "0.0" -- "$cur"))
					[[ "$subcmd" == "head" ]] && [[ "${subsubcmd:0:6}" == "moveto" ]] && COMPREPLY=($(compgen -W "0.0" -- "$cur"))
					;;
				flash)
					[[ "$subcmd" == "compile" ]] && COMPREPLY=($(compgen -W "MX X" -- "$cur"))
					[[ "$subcmd" == "direct" ]] && COMPREPLY=($(compgen -W "CM730 CM740" -- "$cur"))
					[[ "$subcmd" == "robot" ]] && COMPREPLY=($(compgen -W "CM730 CM740" -- "$cur"))
					;;
				set)
					[[ "$subcmd" == "P1" ]] && COMPREPLY=($(compgen -W "nimbro_op nimbro_op_hull igus_op igus_op_hull" -- "$cur"))
					[[ "$subcmd" == "A1" ]] && COMPREPLY=($(compgen -W "nimbro_adult nimbro_adult_hull nimbro_adult_sim nimbro_op2x nimbro_op2x_hull nimbro_op2x_sim" -- "$cur"))
					;;
			esac
			;;
		5)
			case "$cmd" in
				behaviour)
					[[ "$subcmd" == "visualiseDBH" ]] && [[ "$subsubcmd" != "clear" ]] && COMPREPLY=($(compgen -W "yellow blue" -- "$cur"))
					;;
				calib)
					[[ "$subcmd" == "magnetometer" ]] && [[ "$subsubcmd" == "warpAdd" ]] && COMPREPLY=($(compgen -W "$hostlistloc" -- "$cur"))
					;;
				config)
					if [[ "$subcmd" == "cpyaml" ]] || [[ "$subcmd" == "compare" ]]; then
						local configdir="$(rospack find launch)/config"
						local configfile="$configdir/config_$subsubcmd.yaml"
						local depth="$(grep -o "/" <<< "$cur" | wc -l)"
						[[ "$depth" == "0" ]] && depth="1"
						local proctemp="$(python "$(rospack find launch)/config/listYaml.py" "$configfile" "$depth" | sort)"
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
				flash)
					[[ "$subcmd" == "direct" ]] && COMPREPLY=($(compgen -W "MX X" -- "$cur"))
					[[ "$subcmd" == "robot" ]] && COMPREPLY=($(compgen -W "MX X" -- "$cur"))
					;;
			esac
			;;
		6)
			case "$cmd" in
				behaviour)
					[[ "$subcmd" == "visualiseDBH" ]] && [[ "$subsubcmd" == "compass" ]] && COMPREPLY=($(compgen -W "0.0 0.5 1.0 1.5 -0.5 -1.0 -1.5" -- "$cur"))
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
	if [[ "$cmd" == "config" ]] && [[ "$subcmd" == "cleanyaml" ]] && [[ "${COMP_CWORD}" -ge 3 ]]; then
		COMPREPLY=($(compgen -W "all $robotlist" -- "$cur"))
	fi
	[[ -z "$COMPREPLY" ]] && COMPREPLY=($(compgen -o nospace -W "" -- "$cur"))
}

complete -F _nimbro nimbro
# EOF
