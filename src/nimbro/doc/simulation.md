Simulation {#simulation}
============

Using the software with a simulated robot in gazebo
------------
After installation, you can test the software by using it with Gazebo. Only a few steps are necessary in order to simulate a robot. We have simplified the process to launching only two launch-files.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
roslaunch nimbro_op_gazebo nimbro_op.launch 		# starts gazebo, creates the world and spawns the robot.
roslaunch launch gazebo_robot_standing_cap.launch 	# starts the robotcontrol node and connects to the virtual robot.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Before that, please set-up your system variables in each respective terminal by using the nimbro set command:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
nimbro set P1 xs0 igus_op_hull
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
or set it globally by adding these lines to your bashrc:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
export NIMBRO_ROBOT_VARIANT=igus_op_hull 	# to simulate the NimbRo-OP, substitute igus_op_hull with nimbro_op_hull
export NIMBRO_ROBOT_TYPE=P1
export NIMBRO_ROBOT_NAME=xs0			    # using xs0 is necessary to load motions designed for gazebo 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

After these steps, you should be able to see the robot like in the screenshot below:

<table>
<caption id="multi_row"></caption>
<tr><td>![ ](simulation/gazebo1.png)<td>![ ](simulation/gazebo2.png)  
</table>

To control the robot, use a USB joystick. The basic functions are:

* Left stick axis: Walk in commanded direction (if walking is enabled)
* Right stick axis: Rotates the robot(if walking is enabled)
* Button 1: Enable/Disable joystick commands
* Button 2: Enable/Disable walking
* Button 3: Perform a kick with the right foot
* Button 4: Perform a kick with the left foot

As a simple test press button 1, then button 2. The robot should start walking in place. At this point you can control the left and right axis to move the robot. To stop walking, press button 2 again. If the robot falls down, it will get up by playing one of the predesigned getup motions. It is important to note, that <strong>depending on your computers performance, the simulation results may vary</strong>, as gazebo requires a lot of computing resources.

