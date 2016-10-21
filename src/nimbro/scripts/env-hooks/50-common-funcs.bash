#Authors:
#		Hafez Farazi <farazi@ais.uni-bonn.de>
#		Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

# If you change this file, please increment the version number in
# CMakeLists.txt to trigger a CMake update. Alternatively you can
# do nimbro make --force-cmake to quickly test the changes.

# Common environment variables
export ROS_HOSTNAME="$(hostname).local"
export VIS_HOSTNAME="$(hostname)"
export ROSCONSOLE_FORMAT='[${severity}][${node}->${function}]: ${message}'

# Aliases
alias op='xdg-open'
alias src='source ~/.bashrc'
alias qgit='qgit --all'
alias diff='colordiff -u'
alias mirror='rsync -avz --delete --delete-excluded --progress --stats --human-readable'
alias BV='vim ~/.bashrc'
alias BS='source ~/.bashrc'
alias BC='cat ~/.bashrc'
alias BT='tail ~/.bashrc'
alias BG='xdg-open ~/.bashrc'
alias CL='clear'
alias RS='reset'
alias PB='rosbag play -f /vision/takenImg -l --clock'
alias LVM='roslaunch launch vision_module.launch'
alias LRS='roslaunch launch robot_standing_cap.launch'
alias LRSC='roslaunch launch robot_standing_cap.launch'
alias LDRS='roslaunch launch dummy_robot_standing_cap.launch'
alias LDRSC='roslaunch launch dummy_robot_standing_cap.launch'
alias LRG='roslaunch launch robot_game.launch'
alias LDRG='roslaunch launch dummy_robot_game.launch'
alias LC='roslaunch launch communication.launch'
alias LVB='roslaunch launch visualization_bag.launch'
alias LVV='roslaunch launch visualization_bench.launch'
alias LVS='roslaunch launch visualization_standalone.launch'
alias LB='roslaunch launch behaviour.launch'
alias LSD='roslaunch sitting_demo sitting_demo.launch'
alias LCS='roslaunch launch config_server.launch'
alias GenerateEclipseDebug='cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug .'
alias GenerateEclipseRelease='cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Release .'
alias GenerateEclipseNimbroDebug='pushd .;cd $NIMBRO_ROOT;catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles";awk -f $(rospack find mk)/eclipse.awk build/.project > build/.project_with_env && mv build/.project_with_env build/.project;cd build/;cmake ../src -DCMAKE_BUILD_TYPE=Debug;popd'
alias GenerateEclipseNimbroRelease='pushd .;cd $NIMBRO_ROOT;catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles";awk -f $(rospack find mk)/eclipse.awk build/.project > build/.project_with_env && mv build/.project_with_env build/.project;cd build/;popd'
alias SourceThis='cd devel;source setup.bash;cd ..'

function introject_essentials()
{
	if [[ "$HOSTNAME" == "xs"*  ]]; then
    	echo "These files are now being replaced: rc.local, roscore.conf"
		sudo cp /nimbro/lib/scripts/rc.local.d/rc.local /etc/rc.local
		sudo cp /nimbro/lib/scripts/rc.local.d/roscore.conf /etc/init/roscore.conf
		echo "You should restart the robot now"'!'
	else
	    echo "This function is for running on the robot."
	fi
}

function dynarecover()
{
	DC_DIR="$(catkin_find scripts dynarecover.py)"
	python "$DC_DIR" "$@"
}

function changeRobot()
{
	CR_DIR="$(catkin_find scripts changeRobot.py)"
	sudo "$CR_DIR" "$@"
}

function setCamera()
{
	CR_DIR="$(catkin_find scripts set_camera.py)"
	sudo "$CR_DIR" "$@"
}

function installOpenFEC()
{
	CR_DIR="$(catkin_find scripts install_openfec.sh)"
	"$CR_DIR" "$@"
}

function setCm7X0()
{
	CR_DIR="$(catkin_find scripts set_cm7X0.py)"
	sudo "$CR_DIR" "$@"
}

function setNetworkUdev()
{
	CR_DIR="$(catkin_find scripts set_netUdev.py)"
	sudo "$CR_DIR" "$@"
}

function fixSSHSpeed()
{
	CR_DIR="$(catkin_find scripts fix_ssh_speed.py)"
	sudo "$CR_DIR" "$@"
}

_robotList() 
{
    local cur prev

    cur=${COMP_WORDS[COMP_CWORD]}
    prev=${COMP_WORDS[COMP_CWORD-1]}

    case ${COMP_CWORD} in
    	1)
            COMPREPLY=($(compgen -W "xs0 xs1 xs4 xs5 xs6 xs7 xs8" ${cur}))
            ;;
        *)
            COMPREPLY=()
            ;;
    esac
}

_camList() 
{
    local cur prev

    cur=${COMP_WORDS[COMP_CWORD]}
    prev=${COMP_WORDS[COMP_CWORD-1]}

    case ${COMP_CWORD} in
    	1)
            COMPREPLY=($(compgen -W "video0 video1 video2 video3" ${cur}))
            ;;
        *)
            COMPREPLY=()
            ;;
    esac
}

function LoadCamParam()
{
	local RED_COLOR="$(echo -e "\E[1;31m")"
	local NO_COLOR="$(echo -e "\E[0m")"
    if [[ "$1" != "video0" ]] && [[ "$1" != "video1" ]] && [[ "$1" != "video2" ]] && [[ "$1" != "video3" ]]; then
        echo $RED_COLOR"Video device number should be >=0 && <=3"$NO_COLOR
        return
    fi
    v4l2ctrl -d "/dev/$1" -l ~/NimbRo-OP/src/nimbro/launch/config/vision/logitechConfig.txt
}
complete -F _camList LoadCamParam

function SaveCamParam()
{
	local RED_COLOR="$(echo -e "\E[1;31m")"
	local NO_COLOR="$(echo -e "\E[0m")"
    if [[ "$1" != "video0" ]] && [[ "$1" != "video1" ]] && [[ "$1" != "video2" ]] && [[ "$1" != "video3" ]]; then
        echo $RED_COLOR"Video device number should be >=0 && <=3"$NO_COLOR
        return
    fi
    v4l2ctrl -d "/dev/$1" -s ~/NimbRo-OP/src/nimbro/launch/config/vision/logitechConfig.txt
}
complete -F _camList SaveCamParam

function ShowGuiCamParam()
{
	local RED_COLOR="$(echo -e "\E[1;31m")"
	local NO_COLOR="$(echo -e "\E[0m")"
    if [[ "$1" != "video0" ]] && [[ "$1" != "video1" ]] && [[ "$1" != "video2" ]] && [[ "$1" != "video3" ]]; then
        echo $RED_COLOR"Video device number should be >=0 && <=3"$NO_COLOR
        return
    fi
    v4l2ucp -d "/dev/$1"
}
complete -F _camList ShowGuiCamParam

function RemoveSSHPass()
{
	local RED_COLOR="$(echo -e "\E[1;31m")"
	local GREEN_COLOR="$(echo -e "\E[1;32m")"
	local NO_COLOR="$(echo -e "\E[0m")"
	if [[ "$1" == "xs"* ]] || [[ "$1" == "ws"* ]]  ; then
		if [[ "$1" == "xs0" ]]; then
			cat ~/.ssh/id_rsa.pub >> ~/.ssh/authorized_keys && chmod og-wx ~/.ssh/authorized_keys
		else
			cat ~/.ssh/id_rsa.pub | ssh nimbro@"$1".local 'cat - >> ~/.ssh/authorized_keys'
		fi
		echo $GREEN_COLOR"Done."$NO_COLOR
	else
		echo $RED_COLOR"Please specify the robot name."$NO_COLOR
	fi
}
complete -F _robotList RemoveSSHPass

function UpdateSSH()
{
	local RED_COLOR="$(echo -e "\E[1;31m")"
	local GREEN_COLOR="$(echo -e "\E[1;32m")"
	local NO_COLOR="$(echo -e "\E[0m")"
	if [[ "$1" == "xs"* ]] || [[ "$1" == "ws"* ]]  ; then
		ssh-keygen -R "$1"
		echo $GREEN_COLOR"Done."$NO_COLOR
	else
		echo $RED_COLOR"Please specify the robot name."$NO_COLOR
	fi
}
complete -F _robotList RemoveSSHPass

function LT()
{
	local RED_COLOR="$(echo -e "\E[1;31m")"
	local GREEN_COLOR="$(echo -e "\E[1;32m")"
	local NO_COLOR="$(echo -e "\E[0m")"
	if [[ "$1" == "xs"* ]]  ; then
		if [[ "$1" == "xs1" ]] ; then
			echo $GREEN_COLOR"Going to run : nimbro host $1"$NO_COLOR
	    	nimbro host "$1"
			echo $GREEN_COLOR"Going to run : roslaunch launch trajectory_editor.launch"$NO_COLOR
			roslaunch launch trajectory_editor.launch
		else
			echo $GREEN_COLOR"Going to run : nimbro host $1"$NO_COLOR
	    	nimbro host "$1"
			echo $GREEN_COLOR"Going to run : roslaunch launch trajectory_editor.launch"$NO_COLOR
			roslaunch launch trajectory_editor.launch
		fi
	elif [ "$#" -eq  "0" ] ; then
		echo $GREEN_COLOR"Going to run : roslaunch launch trajectory_editor.launch"$NO_COLOR
		roslaunch launch trajectory_editor.launch
	else
		echo $RED_COLOR"Please specify the robot name correctly."$NO_COLOR
	fi
}
complete -F _robotList LT

function LV()
{
	local RED_COLOR="$(echo -e "\E[1;31m")"
	local GREEN_COLOR="$(echo -e "\E[1;32m")"
	local NO_COLOR="$(echo -e "\E[0m")"
	if [[ "$1" == "xs"* ]]  ; then
		if [[ "$1" == "xs1" ]] ; then
			echo $GREEN_COLOR"Going to run : nimbro host $1"$NO_COLOR
	    	nimbro host "$1"
			echo $GREEN_COLOR"Going to run : roslaunch launch visualization.launch"$NO_COLOR
			roslaunch launch visualization.launch
		else
			echo $GREEN_COLOR"Going to run : nimbro host $1"$NO_COLOR
	    	nimbro host "$1"
			echo $GREEN_COLOR"Going to run : roslaunch launch visualization.launch"$NO_COLOR
			roslaunch launch visualization.launch
		fi
	elif [ "$#" -eq  "0" ] ; then
		echo $GREEN_COLOR"Going to run : roslaunch launch visualization.launch"$NO_COLOR
		roslaunch launch visualization.launch
	else
		echo $RED_COLOR"Please specify the robot name correctly."$NO_COLOR
	fi
}
complete -F _robotList LV

function LPV()
{
	local RED_COLOR="$(echo -e "\E[1;31m")"
	local GREEN_COLOR="$(echo -e "\E[1;32m")"
	local NO_COLOR="$(echo -e "\E[0m")"
	if [[ "$1" == "xs"* ]]  ; then
		if [[ "$1" == "xs1" ]] ; then
			echo $GREEN_COLOR"Going to run : nimbro set D1 $1 dynaped"$NO_COLOR
	    	nimbro set D1 "$1" dynaped
			echo $GREEN_COLOR"Going to run : roslaunch launch visualization_plotter.launch"$NO_COLOR
			roslaunch launch visualization_plotter.launch
		else
			echo $GREEN_COLOR"Going to run : nimbro set P1 $1 nimbro_op_hull"$NO_COLOR
	    	nimbro set P1 "$1" nimbro_op_hull
			echo $GREEN_COLOR"Going to run : roslaunch launch visualization_plotter.launch"$NO_COLOR
			roslaunch launch visualization_plotter.launch
		fi
	elif [ "$#" -eq  "0" ] ; then
		echo $GREEN_COLOR"Going to run : roslaunch launch visualization_plotter.launch"$NO_COLOR
		roslaunch launch visualization_plotter.launch
	else
		echo $RED_COLOR"Please specify the robot name correctly."$NO_COLOR
	fi
}
complete -F _robotList LPV

function LDV()
{
	local RED_COLOR="$(echo -e "\E[1;31m")"
	local GREEN_COLOR="$(echo -e "\E[1;32m")"
	local NO_COLOR="$(echo -e "\E[0m")"
	if [[ "$1" == "xs"* ]]  ; then
		if [[ "$1" == "xs1" ]] ; then
			echo $GREEN_COLOR"Going to run : nimbro set D1 $1 dynaped"$NO_COLOR
	    	nimbro set D1 "$1" dynaped
			echo $GREEN_COLOR"Going to run : roslaunch launch visualization_bag.launch"$NO_COLOR
			roslaunch launch visualization_bag.launch
		else
			echo $GREEN_COLOR"Going to run : nimbro set P1 $1 nimbro_op_hull"$NO_COLOR
	    	nimbro set P1 "$1" nimbro_op_hull
			echo $GREEN_COLOR"Going to run : roslaunch launch visualization_bag.launch"$NO_COLOR
			roslaunch launch visualization_bag.launch
		fi
	elif [ "$#" -eq  "0" ] ; then
		echo $GREEN_COLOR"Going to run : roslaunch launch visualization_bag.launch"$NO_COLOR
		roslaunch launch visualization_bag.launch
	else
		echo $RED_COLOR"Please specify the robot name correctly."$NO_COLOR
	fi
}
complete -F _robotList LDV

function BagItVision()
{
	local RED_COLOR="$(echo -e "\E[1;31m")"
	local GREEN_COLOR="$(echo -e "\E[1;32m")"
	local NO_COLOR="$(echo -e "\E[0m")"
	mkdir -p /var/log/nimbro/bags
	if [[ "$1" == "xs0" ]] ; then
		nimbro config set /vision/debug/imgPublishTime 0
		cd /var/log/nimbro/bags
		rosbag record -o "newBag_$1_" -a -x '.*/compressedDepth|/vis/.*|/robot_tracker/.*|/rosout|/clock|/dive_predictor/.*|/config_server/.*|/vision/(?!takenImg$).*'
	elif [[ "$1" == "xs"* ]] ; then
		nimbro host "$1"
		nimbro config set /vision/debug/imgPublishTime 0
		cd /var/log/nimbro/bags
		rosbag record -o "newBag_$1_" -a -x '.*/compressedDepth|/vis/.*|/robot_tracker/.*|/rosout|/clock|/dive_predictor/.*|/config_server/.*|/vision/(?!takenImg$).*'
	else
		echo $RED_COLOR"Please specify the robot name."$NO_COLOR
	fi
}
complete -F _robotList BagItVision

# Check for running ROS processes
function checkros()
{
	proctemp="$(catkin_find --lib)"$'\n'"$(catkin_find --bin)"
	while IFS= read -r -d $'\n' libdir; do
		procs="$(ps ax -o "pid args" | grep "$libdir")"
		while IFS= read -r -d $'\n' procline; do
			if [[ "$procline" =~ ^" "*[0-9]{1,5}" "+("/usr/bin/python "+)?"$libdir".*$ ]]; then # "  15462  /usr/bin/python  /opt/ros/indigo/bin/roscore -w 3"
				tmpA="${procline%%[^ ]*}" # "  "
				tmpB="${procline#$tmpA}"  # "15462  /usr/bin/python  /opt/ros/indigo/bin/roscore -w 3"
				pid="${tmpB%%[^0-9]*}"    # "15462"
				cmd="${tmpB#$pid}"        # "  /usr/bin/python  /opt/ros/indigo/bin/roscore -w 3"
				tmpA="${cmd%%[^ ]*}"      # "  "
				cmd="${cmd#$tmpA}"        # "/usr/bin/python  /opt/ros/indigo/bin/roscore -w 3"
				bin="${cmd%% *}"          # "/usr/bin/python"
				if [[ "$bin" == "/usr/bin/python" ]]; then
					tcmd="${cmd#$bin}"    # "  /opt/ros/indigo/bin/roscore -w 3"
					tmpA="${tcmd%%[^ ]*}" # "  "
					bin="${tcmd#$tmpA}"   # "/opt/ros/indigo/bin/roscore -w 3"
					bin="${bin%% *}"      # "/opt/ros/indigo/bin/roscore"
				fi
				locbin="${bin#$libdir/}"  # "roscore"
				if [[ "$locbin" == "roscore" ]] || [[ "$locbin" == "rosmaster" ]] || [[ "$locbin" == "rosout/rosout" ]] || [[ "$locbin" == "catkin_make" ]]; then
					echo "$(echo -e "\E[1;32m")Running:$(echo -e "\E[0m") $cmd"
				else
					echo "$(echo -e "\E[1;31m")Running:$(echo -e "\E[0m") $cmd"
				fi
			fi
		done <<< "$procs"
	done <<< "$proctemp"
}

# Kill running ROS processes
function killros()
{
	# Kill all common ROS stuff
	echo "Killing launch files..."
	sudo killall -q -v rosmon
	sudo killall -q -v roslaunch
	
	# Wait some time
	echo "<< Waiting 3s >>"
	sleep 3s
	
	# Kill all run nodes
	echo "Killing ROS nodes..."
	proctemp="$(catkin_find --lib)"$'\n'"$(catkin_find --bin)"
	while IFS= read -r -d $'\n' libdir; do
		procs="$(ps ax -o "pid args" | grep "$libdir")"
		while IFS= read -r -d $'\n' procline; do
			if [[ "$procline" =~ ^" "*[0-9]{1,5}" "+("/usr/bin/python "+)?"$libdir".*$ ]]; then # "  15462  /usr/bin/python  /opt/ros/indigo/bin/roscore -w 3"
				tmpA="${procline%%[^ ]*}" # "  "
				tmpB="${procline#$tmpA}"  # "15462  /usr/bin/python  /opt/ros/indigo/bin/roscore -w 3"
				pid="${tmpB%%[^0-9]*}"    # "15462"
				cmd="${tmpB#$pid}"        # "  /usr/bin/python  /opt/ros/indigo/bin/roscore -w 3"
				tmpA="${cmd%%[^ ]*}"      # "  "
				cmd="${cmd#$tmpA}"        # "/usr/bin/python  /opt/ros/indigo/bin/roscore -w 3"
				bin="${cmd%% *}"          # "/usr/bin/python"
				if [[ "$bin" == "/usr/bin/python" ]]; then
					tcmd="${cmd#$bin}"    # "  /opt/ros/indigo/bin/roscore -w 3"
					tmpA="${tcmd%%[^ ]*}" # "  "
					bin="${tcmd#$tmpA}"   # "/opt/ros/indigo/bin/roscore -w 3"
					bin="${bin%% *}"      # "/opt/ros/indigo/bin/roscore"
				fi
				locbin="${bin#$libdir/}"  # "roscore"
				if [[ "$locbin" == "roscore" ]] || [[ "$locbin" == "rosmaster" ]] || [[ "$locbin" == "rosout/rosout" ]] || [[ "$locbin" == "catkin_make" ]]; then
					continue
				fi
				sudo kill "$pid"
				echo "$(echo -e "\E[1;31m")Killed$(echo -e "\E[0m") $(echo -e "\E[1;32m")node($pid)$(echo -e "\E[0m") with signal 15: $cmd"
			fi
		done <<< "$procs"
	done <<< "$proctemp"
	
	# Wait some time
	echo "<< Waiting 1s >>"
	sleep 1s
	
	# Kill the roscore last to avoid nodes possibly hanging
	echo "Killing roscore..."
	sudo killall -q -v roscore
}

# Recursively remove temporary files from a directory
function rmtmp()
{
	case "$1" in
	"d"|"dr"|"dry"|"dryr"|"dryru"|"dryrun")
		find . -name "*~" -type f -printf "[dryrun] Will remove %p\n"
		;;
	"q")
		find . -name "*~" -type f -delete
		;;
	"")
		find . -name "*~" -type f -printf "Removing file %p\n" -delete
		;;
	*)
		echo "Unsupported option \`$1'. Did you mean \`dryrun' or \`d' (or \`q' for quiet mode)?"
		;;
	esac
}

# Recursively remove .directory files from a directory
function rmtmpdir()
{
    case "$1" in
    "-d"|"-dr"|"-dry"|"-dryr"|"-dryru"|"-dryrun")
        find . -name ".directory" -type f -printf "[dryrun] Will remove %p\n"
        ;;
    "-q")
        find . -name ".directory" -type f -delete
        ;;
    "")
        find . -name ".directory" -type f -printf "Removing file %p\n" -delete
        ;;
    *)
        echo "Unsupported option '$1'. Did you mean '-dryrun' or '-d' (or '-q' for quiet mode)?"
        ;;
    esac
}

function gitall()
{
	# Colours
	local LIGHT_CYAN="$(echo -e "\E[1;36m")"
	local NO_COLOUR="$(echo -e "\E[0m")"
	
	# Parse options to gitall
	haveexclude=
	excludeopts=()
	while true; do
		param="$1"
		[[ "${param:0:1}" != "-" ]] && break
		case "$param" in
			-exclude)
				exdir="$2"
				exdir="${exdir#./}"
				exdir="${exdir%/}"
				echo "Excluding: $exdir"
				if [[ -z "$haveexclude" ]]; then
					excludeopts+=(\( -path "./$exdir")
				else
					excludeopts+=(-o -path "./$exdir")
				fi
				haveexclude="true"
				shift
			;;
			*)
				echo "Error: Unknown option '$param' for gitall!"
				return 1
			;;
		esac
		shift
	done
	[[ -n "$haveexclude" ]] && excludeopts+=(\) -prune -o)
	
	# Handle case with no options
	if [[ $# -eq 0 ]]; then
		echo "Warning: No git action was passed to gitall!"
		return 0
	fi

	# Find each git repository and execute the required command
	echo "Finding all git repos in current directory..."
	echo
	echo "find -P . -ignore_readdir_race -noleaf -nowarn -type d ${excludeopts[@]} -name \".git\" -print"
	proctemp="$(find -P . -ignore_readdir_race -noleaf -nowarn -type d "${excludeopts[@]}" -name ".git" -print)" 2>/dev/null
	echo
	while IFS= read -r -d $'\n' gitfolder; do
		[[ -z "$gitfolder" ]] && continue
		gitfolder="$(dirname "$gitfolder")"
		echo $LIGHT_CYAN"Git repo: $gitfolder"$NO_COLOUR
		(
			cd "$gitfolder" && {
				git "$@"
			}
		)
		echo
	done <<< "$proctemp"
}
# EOF
