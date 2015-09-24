Launch files {#launch}
============

Overview of the available launch files in the `launch` package:

<table>
	<tr>
		<th>Name</th>
		<th>Purpose</th>
	</tr>
	<tr>
		<td>`hardware/robot.launch`</td>
		<td>
			Main robot control launch file. Contains RobotControl and other
			important nodes for robot hardware operation.
		</td>
	</tr>
	<tr>
		<td>`hardware/robot_dummy.launch`</td>
		<td>
			Same as `robot.launch`, but uses the dummy
			hardware interface (i.e. needs no physical robot).
		</td>
	</tr>
	<tr>
		<td>`config/server.launch`</td>
		<td>
			Contains the config_server. Included in `hardware/robot.launch`.
		</td>
	<tr>
		<td>`visualization/visualization.launch`</td>
		<td>
			Starts an rqt (ROS Qt GUI) instance for visualization.
		</td>
	</tr>
</table>