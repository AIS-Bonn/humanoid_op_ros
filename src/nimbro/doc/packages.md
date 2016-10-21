Packages and Modules {#packages}
====================

A summary of the provided software packages is given in the following table:

| Repository | Section | Package Name | Description |
|------------|---------|--------------|-------------|
| `catch_ros` | `general` | `catch_ros` | A small ROS wrapper around the Catch unit testing framework. |
| `dynalib` | `general` | `dynalib` | Library and tools (in particular `dynatool`) for communicating with arbitrary Dynamixel devices. |
| `nimbro` | `behaviour` | `behaviour_control` | Generic C++ implementation of a hierarchical behaviour architecture, the Behaviour Control Framework. |
| `nimbro` | `behaviour` | `state_controller` | Generic C++ implementation of a dynamic planning finite state machine architecture, the State Controller Library. |
| `nimbro` | `behaviour` | `walk_and_kick` | A behaviour for playing soccer. |
| `nimbro` | `cv` | `cv_matlab` | Matlab files for computer vision. |
| `nimbro` | `cv` | `vision_module` | Vision module for playing soccer, including localization. |
| `nimbro` | `demonstration` | `demo_msgs` | Common messages for demonstration nodes. |
| `nimbro` | `demonstration` | `hand_shaking_demo` | Hand shaking demonstration. |
| `nimbro` | `demonstration` | `sitting_demo` | Seated robot demonstration. |
| `nimbro` | `demonstration` | `standing_demo` | Standing robot demonstration. |
| `nimbro` | `general` | `launch` | Collection of various launch files for the software framework. |
| `nimbro` | `general` | `scripts` | Collection of various BASH and Python scripts for software framework integration. |
| `nimbro` | `hardware` | `cm730` | Interface classes and firmware for the CM730/CM740 microcontroller boards. |
| `nimbro` | `hardware` | `diagnostics` | Widget for rqt that displays diagnostic information. |
| `nimbro` | `hardware` | `nimbro_op_gazebo` | Package providing simulation of the robot in Gazebo. |
| `nimbro` | `hardware` | `nimbro_op_interface` | Hardware interface for the real robot. |
| `nimbro` | `hardware` | `nimbro_op_model` | URDF model of the robot. |
| `nimbro` | `localization` | `field_model` | Model of the soccer field, for standardisation across the framework. |
| `nimbro` | `localization` | `loc_display` | RViz visualisation plugin to display the soccer field. |
| `nimbro` | `motion` | `cap_gait` | Capture step gait plugin to the `gait` motion module. |
| `nimbro` | `motion` | `fall_protection` | Fall protection motion module. |
| `nimbro` | `motion` | `gait` | Generic gait motion module that uses gait engines via a modular plugin scheme. |
| `nimbro` | `motion` | `gait_msgs` | ROS messages and services used by the gait. |
| `nimbro` | `motion` | `head_control` | Head control motion module. |
| `nimbro` | `motion` | `limb_control` | Limb control motion module for common motions. |
| `nimbro` | `motion` | `motion_player` | Keyframe motion player motion module. |
| `nimbro` | `tools` | `bench_vis` | RQT-based visualisation for soccer robots during games. |
| `nimbro` | `tools` | `control_widget` | Widget for rqt with a number of buttons for common control actions. |
| `nimbro` | `tools` | `hash_library` | Collection of hashing libraries. |
| `nimbro` | `tools` | `led_widget` | Widget for rqt that displays current LED states and allows button presses. |
| `nimbro` | `tools` | `nimbro_relay` | Custom ROS topic relay node. |
| `nimbro` | `tools` | `nimbro_utils` | Utilities for performing common calculations, implementing common filters, and so on. |
| `nimbro` | `tools` | `rcup_game_controller` | Node that listens to the RoboCup game controller. |
| `nimbro` | `tools` | `rosbag` | Improved version of the ROS-native `rosbag` utility. |
| `nimbro` | `tools` | `rqt_brviz` | Customised version of the ROS-native `rqt_rviz` package. |
| `nimbro` | `tools` | `test_utilities` | Utilities for writing unit tests. |
| `nimbro` | `tools` | `tf_tools` | Utilities related to TF transforms and listeners. |
| `nimbro` | `tools` | `trajectory_editor` | Trajectory editor for designing keyframe motions. |
| `nimbro` | `tools` | `walk_control` | Widget for rqt for publishing manual gait commands for test purposes. |
| `nimbro_config_server` | `general` | `config_server` | The configuration server for management and dynamic control of flexible software parameters. |
| `nimbro_config_server` | `general` | `parameter_tuner` | Widget for rqt for manual modification of configuration server parameters. |
| `nimbro_config_server` | `general` | `remote_tuner` | Parameter tuner for the configuration server tweaked for remote connections. |
| `nimbro_config_server` | `general` | `tf_tuner` | Publisher of arbitrary configurable tf transforms for tuning purposes. |
| `nimbro_network` | `general` | `nimbro_cam_transport` | NimbRo network transport of camera images. |
| `nimbro_network` | `general` | `nimbro_log_transport` | NimbRo network transport of ROS logs. |
| `nimbro_network` | `general` | `nimbro_service_transport` | NimbRo network transport of ROS service calls. |
| `nimbro_network` | `general` | `nimbro_topic_transport` | NimbRo network transport of ROS topics. |
| `nimbro_network` | `general` | `tf_throttle` | NimbRo network transport of TF messages, sampled at fixed intervals to reduce bandwidth. |
| `nimbro_robotcontrol` | `contrib` | `rbdl` | Import of the RBDL library with tweaks. |
| `nimbro_robotcontrol` | `hardware` | `robotcontrol` | The main robot control node (includes many utilities). |
| `nimbro_robotcontrol` | `hardware` | `rviz_dynamics` | RViz visualisation plugin for showing dynamics information (e.g. torque display). |
| `nimbro_robotcontrol` | `hardware` | `servomodel` | A feed-forward model to improve control of the servos. |
| `nimbro_robotcontrol` | `hardware` | `timer` | Wrapper classes for high performance timer functionality. |
| `nimbro_robotcontrol` | `util` | `rc_utils` | Generic robotcontrol utilities. |
| `nimbro_robotcontrol` | `util` | `rot_conv` | Rotation representation conversion library. |
| `nimbro_vis` | `contrib` | `rqt` | Import of rqt with tweaks. |
| `nimbro_vis` | `general` | `plot_msgs` | ROS messages and services used for plotting. |
| `nimbro_vis` | `general` | `plotter` | Widget for rqt that allows flexible plotting of data and events. |
| `nimbro_vis` | `general` | `rqt_log_viewer` | Widget for rqt that displays ROS messages from arbitrary nodes. |
| `nimbro_vis` | `general` | `rrlogger` | Configurable ROS topic and plot data background logger. |
| `nimbro_vis` | `general` | `timewarp` | Buffer and time transform node for ROS topics, in particular for the plotter and visualisation. |
| `nimbro_vis` | `general` | `vis_utils` | Visualisation utilities. |
| `rosmon` | `general` | `rosmon` | ROS node launch and monitor daemon. |
