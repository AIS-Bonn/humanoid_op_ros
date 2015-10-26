Launch Files {#launchfiles}
============

A summary of the available launch files is provided in the following table:

| Package | Launch file | Description |
|---------|-------------|-------------|
| `behaviour_exercise` | `behaviour_exercise_*` | Launch files for the behaviour exercise. |
| `hand_shaking_demo` | `hand_shaking_demo.launch` | Launches the hand shaking demonstration with the robot interface. |
| `hand_shaking_demo` | `dummy_hand_shaking_demo.launch` | Launches the hand shaking demonstration with the dummy interface. |
| `launch` | `robot_calib.launch` | Launches robotcontrol with the robot interface, zero init position, and no motion modules. This is intended for performing calibrations only. |
| `launch` | `robot_common.launch` | Not intended to be launched directly, but used by every other `robot_*` launch file. |
| `launch` | `robot_demo_*.launch` | Launch files used by demo launch files to launch the robotcontrol node exactly as required for the specific demo. |
| `launch` | `robot_dummy_*.launch` | The same as `robot_*.launch` only with the dummy interface instead of the robot interface. |
| `launch` | `robot_game.launch` | The default launch file for robotcontrol that is intended for playing games. The same as `robot_standing_cap.launch`, only with getup motions. |
| `launch` | `robot_gazebo_indep.launch` | Launches robotcontrol with the gazebo interface and `indep_cpg_gait` gait. |
| `launch` | `robot_motion_test_*.launch` | Launches robotcontrol with the correct state for the purpose of testing particular keyframe motions. |
| `launch` | `robot_standing_cap.launch` | Launches robotcontrol with the robot interface, standing init position, `cap_gait` gait and no getup motions. This is the recommended default. |
| `launch` | `robot_standing_cpg.launch` | Launches robotcontrol with the robot interface, standing init position, `cpg_gait` gait and no getup motions. |
| `launch` | `robot_standing_indep.launch` | Launches robotcontrol with the robot interface, standing init position, `indep_cpg_gait` gait and no getup motions. |
| `launch` | `server.launch` | Launches the configuration server. This is usually not called directly, but instead through a `robot_*` launch file. |
| `launch` | `trajectory_editor_2.launch` | Launches the trajectory editor for editing keyframe motions. |
| `launch` | `visualization.launch` | Launches an instance of RQT for visualization purposes (a robotcontrol instance should be running). |
| `launch` | `visualization_bag.launch` | Launches an instance of RQT for visualization of ROS bag data. |
| `launch` | `visualization_standalone.launch` | Launches an instance of RQT for visualization purposes (if no robotcontrol instance is running). |
| `nimbro_op_gazebo` | `nimbro_op.launch` | Launches gazebo with the `nimbro_op` world. |
| `nimbro_op_model` | `model.launch` | Loads the URDF model to the ROS parameter server. |
| `rrlogger` | `rrlogger_basic.launch` | Launches the logger, usually not called directly, but instead from a `robot_*` launch file. |
| `sitting_demo` | `sitting_demo.launch` | Launches the sitting demonstration with the robot interface. |
| `sitting_demo` | `dummy_sitting_demo.launch` | Launches the sitting demonstration with the dummy interface. |
| `standing_demo` | `standing_demo.launch` | Launches the standing demonstration with the robot interface. |
| `standing_demo` | `dummy_standing_demo.launch` | Launches the standing demonstration with the dummy interface. |

A summary of additional nodes is provided in the following table:

| Package | Node | Description |
|---------|------|-------------|
| `dynalib` | `dynatool` | Launches `dynatool` for testing of arbitrary Dynamixel buses and devices. |
| `walk_and_kick` | `walk_and_kick` | Launches the walk and kick behaviour node. |
| `rcup_game_controller` | `game_controller` | Launch a node that handles communications from the RoboCup game controller. |

Note that in general the function of the three buttons on the back of the robot from left to right is fade in/out,
start/stop, and hardware reset. The two leftmost LEDs display RX/TX activity of the microcontroller to PC connection,
the left RGB LED is green if the microcontroller is connected to the PC by USB and red otherwise, and the right RGB
LED is a custom colour as specified by the currently running behaviour nodes. The other LEDs are also available for
custom use by behaviours.
