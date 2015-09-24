# If you change this file, please increment the version number in
# CMakeLists.txt to trigger a CMake update.

function dynatool()
{
	DYNA_ROBOTS_DIR="$(rospack find dynalib)/robots"
	if [[ -d "$DYNA_ROBOTS_DIR" ]]; then
		if [[ -f "$DYNA_ROBOTS_DIR/NOP.robot" ]]; then
			NOP_FILE="$DYNA_ROBOTS_DIR/NOP.robot"
		else
			NOP_FILE="$(ls $DYNA_ROBOTS_DIR/*.robot 2>/dev/null | head -1)"
		fi
	else
			NOP_FILE=""
	fi
	if [[ -f "$NOP_FILE" ]]; then
		rosrun dynalib dynatool --device=/dev/cm730 --robot="$NOP_FILE" "$@"
	else
		rosrun dynalib dynatool --device=/dev/cm730 "$@"
	fi
}
# EOF