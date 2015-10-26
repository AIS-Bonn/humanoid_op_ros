Installation {#installation}
============

This page documents the installation process required for the igus Humanoid Open Platform ROS software release.

Dependencies
------------

* Ubuntu 14.04 (anything else and you are on your own)

* [ROS Indigo](http://www.ros.org/wiki/indigo/Installation/Ubuntu):
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo apt-get install ros-indigo-desktop-full
sudo apt-get install ros-indigo-rqt-rviz
sudo apt-get install ros-indigo-joy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* The QGLViewer library:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo apt-get install libqglviewer-dev
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* GNU Scientific Library (GSL):
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo apt-get install libgsl0-dev
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Controllers for Gazebo:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo apt-get install ros-indigo-gazebo-ros-control
sudo apt-get install ros-indigo-ros-controllers
sudo apt-get install ros-indigo-controller-manager
sudo apt-get install ros-indigo-rqt-controller-manager
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Vision packages:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo apt-get install v4l-utils
sudo add-apt-repository ppa:pj-assis/ppa
sudo apt-get update
sudo apt-get install guvcview
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Python dependencies:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo apt-get install python-pip python-dev build-essential
sudo pip install --upgrade pip
sudo pip install --upgrade virtualenv
sudo pip install termcolor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Installation
------------

Place the following into your `.bashrc`:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
source /opt/ros/indigo/setup.bash
source /path/to/src/nimbro/scripts/env.sh

export NIMBRO_ROBOT_TYPE=P1
export NIMBRO_ROBOT_NAME=xs4
export NIMBRO_ROBOT_VARIANT=nimbro_op_hull
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Clone our repository:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
cd ~ # Or another path without spaces can be taken
git clone https://github.com/AIS-Bonn/humanoid_op_ros humanoid_op_ros
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

System setup
------------

The robotcontrol node needs realtime permissions to run reliably. Furthermore, you need to setup
udev rules for the devices we use. There is a shell script that does the required system setup,
but it is recommended that you perform the required actions manually yourself and just use the
shell script as a guide.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
nimbro/scripts/system_setup.sh
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You will need to log out and in again to load the security settings.
Alternatively, you can use the following to emulate the login:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo -i -u user
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For everything to work properly you may also need to modify the first two lines of
your `/etc/hosts` to something like this, where `mycomp` is the name of your computer:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
127.0.0.1       localhost
127.0.1.1       mycomp mycomp.local
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The `nimbro` utility
--------------------

The nimbro utility (present once the `env.sh` script is sourced, e.g. in your `.bashrc`) can
help you in your daily work. Type `nimbro help` for usage information. The command source is,
as mentioned, in the `env.sh` script. Start by compiling the source
code using the following command:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
nimbro make
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Here are a few examples of use of the nimbro tool:

	nimbro                  # Go to the nimbro root directory
	nimbro doc              # Open the nimbro source documentation
	nimbro source           # Go to the nimbro source directory
	nimbro src              # Go to the nimbro source directory
	nimbro src nim          # Go to the nimbro source directory
	nimbro src rob          # Go to the nimbro_robotcontrol source directory
	nimbro src vis          # Go to the nimbro_vis source directory
	nimbro make             # Build the framework
	nimbro make tests       # Build the framework unit tests
	nimbro make run_tests   # Build and run the framework unit tests
	nimbro clean            # Clean all build products and temporary files
	nimbro remake-all       # Delete the devel and build folders and recompile the framework
	nimbro make-doc         # Generate the source code documentation
	nimbro make-docv        # Generate the source code documentation (verbose)
	nimbro make-doc open    # Generate the source code documentation and attempt to open it
	nimbro deploy           # Build the framework and copy the installed files to the robot
	nimbro deploy xs4.local # Deploy to the xs4 robot (may not be the set robot)
	nimbro host xs4.local   # Set xs4 as the target robot and the ROS master
	nimbro ssh              # Open a ssh shell to the set robot
	nimbro help             # Display help for the nimbro tool
