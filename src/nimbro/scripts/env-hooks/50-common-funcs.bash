# If you change this file, please increment the version number in
# CMakeLists.txt to trigger a CMake update.

# Common environment variables
export ROS_HOSTNAME="$(hostname).local"
export ROSCONSOLE_FORMAT='[${severity}][${node}->${function}]: ${message}'

# Aliases
alias diff='colordiff -u'

function wifi()
{
	WIFI_DIR="$(catkin_find scripts wifi.py)"
	sudo $WIFI_DIR "$@"
}

# Check for running ROS processes
function checkros()
{
	ps aux | grep -v -E 'grep.*startup.sh' | grep startup.sh
	ps aux | grep -v -E 'grep.*ros' | grep ros
}

# Kill running ROS processes
function killros()
{
	sudo killall startup.sh 2>/dev/null
	sudo killall rosrun 2>/dev/null
	sudo killall roslaunch 2>/dev/null
	sudo killall roscore 2>/dev/null
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
# EOF
