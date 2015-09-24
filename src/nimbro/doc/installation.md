Installation {#installation}
============

Dependencies
------------

* Ubuntu 12.10 or 13.04 (anything else and you are on your own)
* [ROS Groovy](http://www.ros.org/wiki/groovy/Installation/Ubuntu) - Install the packages:
  * ros-groovy-desktop-full
  * ros-groovy-rqt-rviz
  * ros-groovy-joy
* The QGLViewer library - Install the package:
  * libqglviewer-dev
* GNU Scientific Library (GSL) - Install the package:
  * libgsl0-dev

Installation
------------

Load the ROS environment (place this into your .bashrc if you do not want to do this every time you
work on the software):

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
. /opt/ros/groovy/setup.bash
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Now clone our NimbRo repository:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
cd ~            # Or wherever you want to put the repository
git clone [https://github.com/NimbRo/nimbro-op-ros](https://github.com/NimbRo/nimbro-op-ros)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


System setup
------------

The robotcontrol node needs realtime permissions to run reliably. Furthermore, you need to setup
udev rules for the devices we use. To simplify this process, there is a shell script that does
the required system setup:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
nimbro/scripts/system_setup.sh
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You will need to log out and in again to load the security settings.
Alternatively, you can use the following to emulate the login:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo -i -u user
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In order to reduce the computational load of viewing the robot in RViz, you may
also choose to add the following line to your .bashrc, which ensures that a
simplified model of the robot is used for visualisation purposes.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
export NIMBRO_ROBOT_VARIANT=nimbro_op_hull
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For everything to work properly you may also need to modify the first line of
your `/etc/hosts` to something like this, where `mycomp` is the name of your
computer:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
127.0.0.1       localhost mycomp.local
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Robot setup
-----------

Copy the `robot_setup.sh` from `nimbro-op-ros` to the robot and execute it.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
~/robot_setup.sh
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Now follow the the other instructions from the System setup section.

In Order to give the robot access to the CM730 you will have to change the serial number in the `/etc/udev/rules.d/90-cm730.rules` to the serial number of the cm730 in your robot.

The `nimbro` utility
--------------------

The nimbro utility (usable after you run `system_setup.sh`) can help you in your
daily work. Type `nimbro help` for usage information. The command source is in
the `env.sh` script, located in the same folder. Start by compiling the source
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
	nimbro make             # Build the NimbRo framework
	nimbro make tests       # Build the framework unit tests
	nimbro make run_tests   # Build and run the framework unit tests
	nimbro clean            # Clean all build products and temporary files
	nimbro remake-all       # Delete the devel and build folders and recompile the framework
	nimbro make-doc         # Generate the source code documentation
	nimbro make-docv        # Generate the source code documentation (verbose)
	nimbro make-doc open    # Generate the source code documentation and attempt to open it
	nimbro deploy           # Build the framework and copy the installed files to the robot
	nimbro deploy xs2.local # Deploy to the xs2 robot (may not be the set robot)
	nimbro host xs2.local   # Set xs2 as the target robot and the ROS master
	nimbro ssh              # Open a ssh shell to the set robot
	nimbro help             # Display help for the nimbro tool
