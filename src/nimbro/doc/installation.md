Installation {#installation}
============

This page documents the installation process required for the igus Humanoid Open Platform ROS software release.

Dependencies
------------

* Ubuntu 14.04 LTS (anything else and you are on your own)

* Suggested kernel: Xenial Xerus
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo apt-get install linux-generic-lts-xenial
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* [ROS Indigo](http://www.ros.org/wiki/indigo/Installation/Ubuntu):
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full
sudo rosdep init
rosdep update
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
sudo apt-get install v4l-utils v4l2ucp v4l-conf
sudo add-apt-repository ppa:pj-assis/ppa
sudo apt-get update
sudo apt-get install guvcview
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Python dependencies:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo apt-get install sshpass
sudo apt-get install python-pip python-dev build-essential
sudo pip install --upgrade pip
sudo pip install --upgrade virtualenv
sudo pip install --upgrade --no-deps --force-reinstall pexpect
sudo pip install termcolor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* ncurses Library: 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo apt-get install libncurses5-dev
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* x264 Library: 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo apt-get install libx264-dev
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* GCC ARM Compiler: 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo apt-get install gcc-arm-none-eabi
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Doxygen Documentation System:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo apt-get install doxygen
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Utilities (optional, choose relevant ones):
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo apt-get install curl tshark ssh screen
sudo apt-get install git git-gui gitk qgit
sudo apt-get install joe vim nano kwrite kdiff3 colordiff kompare
sudo apt-get install gdb valgrind tree htop
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Installation
------------

Place the following into your `.bashrc`:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
source /opt/ros/indigo/setup.bash
source ~/NimbRo-OP/src/nimbro/scripts/env.sh # Or alternate path

export NIMBRO_ROBOT_TYPE=P1
export NIMBRO_ROBOT_NAME=xs0
export NIMBRO_ROBOT_VARIANT=nimbro_op_hull
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Clone our repository:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
git clone https://github.com/AIS-Bonn/humanoid_op_ros ~/NimbRo-OP
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

System setup
------------

The robotcontrol node needs realtime permissions to run reliably. Create a file
`/etc/security/limits.d/nimbro.conf` with contents (replace USER with your username):
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*    hard rtprio 0
*    soft rtprio 0
USER hard rtprio 20
USER soft rtprio 20
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You will need to log out and in again to load the security settings.
Alternatively, you can use the following to emulate the login:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo -i -u user
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you want to set up udev rules for devices you want to use, then refer to:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
src/nimbro/scripts/set_camera.py
src/nimbro/scripts/set_cm7X0.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For installation and deployment of the required files to the robots, a special installation folder is required:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo mkdir -p /nimbro
sudo chown $USER /nimbro
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A logging and backup directory is also required:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo mkdir -p /var/log/nimbro
sudo chmod 777 /var/log/nimbro
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
