#!/bin/bash

set -e

if [[ $UID != 0 ]]; then
	sudo $0 $(whoami)
	exit $?
fi

if [[ $# != 1 ]]; then
	echo "Please run as non-root or specify the user to setup as first argument"
	exit 1
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT="$(cd "$DIR/.." && pwd)"
USER=$1

if [[ ! -e "$ROOT/scripts/system_setup.sh" ]]; then
	echo "Could not determine nimbro root"
	exit 1
fi

echo "NimbRo root is at $ROOT"

echo "Enabling realtime priority for $USER"
cat <<EOS > /etc/security/limits.d/nimbro.conf
*         hard rtprio 0
*         soft rtprio 0
$USER     hard rtprio 20
$USER     soft rtprio 20
EOS

echo "Installing cm730 udev rule to /etc/udev/rules.d"
echo "Adapt serial in 90-cm730.rules if necessary"

cp $ROOT/scripts/90-cm730.rules /etc/udev/rules.d

echo "Updating rosdep cache"
rosdep init &> /dev/null || true
su -c "rosdep update" $USER

echo "Creating /nimbro installation prefix"
mkdir -p /nimbro
chown $USER /nimbro


ENV_SCRIPT="source /nimbro/setup.bash"
if ! grep "$ENV_SCRIPT" ~/.bashrc &> /dev/null; then
	echo "Updating your .bashrc"
	echo "$ENV_SCRIPT" >> ~/.bashrc
fi


