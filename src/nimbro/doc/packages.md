Packages and Modules {#packages}
====================

A summary of the provided software packages is given in the following table:

| Repository | Section | Package Name | Description |
|------------|---------|--------------|-------------|
| `dynalib` | `general` | `dynalib` | Library and tools (in particular `dynatool`) for communicating with arbitrary Dynamixel devices. |
| `nimbro` | `behaviour` | `behaviour_control` | Generic C++ implementation of a hierarchical behaviour architecture, the Behaviour Control Framework. |
| `nimbro` | `behaviour` | `behaviour_exercise` | An exercise for the testing of basic behaviours and behaviour architectures. |
| `nimbro` | `behaviour` | `state_controller` | Generic C++ implementation of a dynamic planning finite state machine architecture, the State Controller Library. |
| `nimbro` | `behaviour` | `walk_and_kick` | A simple behaviour for playing soccer. |
| `nimbro` | `cv` | `bgr2yuyv` | Conversion utility from BGR to YUYV. |
| `nimbro` | `cv` | `calibration` | Colour calibration tool. |
| `nimbro` | `cv` | `camera_v4l2` | V4L2 camera driver. |
| `nimbro` | `cv` | `classificator` | Colour classifier. |
| `nimbro` | `cv` | `display_image` | Image display tools and widget. |
| `nimbro` | `cv` | `downscale` | Image downscaler and throttler for viewing images over wireless connections. |
| `nimbro` | `cv` | `face_detection` | Face detection routines using Haar cascading. |
| `nimbro` | `cv` | `nodelet_gui_manager` | Nodelet GUI manager. |
| `nimbro` | `cv` | `print_fps` | Tool for printing the fps of an image stream. |
| `nimbro` | `cv` | `vision_module` | Vision module for playing soccer, including localization. |
| `nimbro` | `demonstration` | `demo_msgs` | Common messages for demonstration nodes. |
| `nimbro` | `demonstration` | `face_tracker` | Face tracking demonstration. |
| `nimbro` | `demonstration` | `hand_shaking_demo` | Hand shaking demonstration. |
| `nimbro` | `demonstration` | `sitting_demo` | Seated robot demonstration. |
| `nimbro` | `demonstration` | `standing_demo` | Standing robot demonstration. |
| `nimbro` | `general` | `launch` | Collection of various launch files for the software framework. |
| `nimbro` | `general` | `scripts` | Collection of various BASH and Python scripts for software framework integration. |
| `nimbro` | `hardware` | `cm730` | Interface classes and firmware for the CM730/CM740 microcontroller boards. |
| `nimbro` | `hardware` | `diagnostics` | Widget for rqt that displays diagnostic information. |
| `nimbro` | `hardware` | `nimbro_op_gazebo` | Package providing simulation of the robot in Gazebo. |
| `nimbro` | `hardware` | `nimbro_op_interface` | Hardware interface for the real robot. |
| `nimbro` | `hardware` | `nimbro_op_kinematics` | Kinematic model of the robot. |
| `nimbro` | `hardware` | `nimbro_op_model` | URDF model of the robot. |
| `nimbro` | `localization` | `field_model` | Model of the soccer field, for standardisation across the framework. |
| `nimbro` | `localization` | `loc_display` | RViz visualisation plugin to display the soccer field. |
| `nimbro` | `motion` | `cap_gait` | Capture step gait plugin to the `gait` motion module. |
| `nimbro` | `motion` | `cpg_gait` | Central pattern generated gait plugin to the `gait` motion module. |
| `nimbro` | `motion` | `fall_protection` | Fall protection motion module. |
| `nimbro` | `motion` | `gait` | Generic gait motion module that uses gait engines via a modular plugin scheme. |
| `nimbro` | `motion` | `gait_msgs` | ROS messages and services used by the gait. |
| `nimbro` | `motion` | `head_control` | Head control motion module. |
| `nimbro` | `motion` | `indep_cpg_gait` | Independent central pattern generated gait (not a plugin). |
| `nimbro` | `motion` | `limb_control` | Limb control motion module for common motions. |
| `nimbro` | `motion` | `motion_player` | Keyframe motion player motion module. |
| `nimbro` | `tools` | `led_widget` | Widget for rqt that displays current LED states and allows button presses. |
| `nimbro` | `tools` | `nimbro_utils` | Utilities for performing common calculations, implementing common filters, and so on. |
| `nimbro` | `tools` | `rcup_game_controller` | Node that listens to the RoboCup game controller. |
| `nimbro` | `tools` | `test_utilities` | Utilities for writing unit tests. |
| `nimbro` | `tools` | `trajectory_editor_2` | Trajectory editor for designing keyframe motions. |
| `nimbro` | `tools` | `walk_control` | Widget for rqt for publishing manual gait commands for test purposes. |
| `nimbro_config_server` | `general` | `config_server` | The configuration server for management and dynamic control of flexible software parameters. |
| `nimbro_config_server` | `general` | `parameter_tuner` | Widget for rqt for manual modification of configuration server parameters. |
| `nimbro_config_server` | `general` | `remote_tuner` | Parameter tuner for the configuration server tweaked for remote connections. |
| `nimbro_config_server` | `general` | `tf_tuner` | Publisher of arbitrary configurable tf transforms for tuning purposes. |
| `nimbro_robotcontrol` | `contrib` | `rbdl` | Import of the RBDL library with tweaks. |
| `nimbro_robotcontrol` | `hardware` | `robotcontrol` | The main robot control node (includes many utilities). |
| `nimbro_robotcontrol` | `hardware` | `rviz_dynamics` | RViz visualisation plugin for showing dynamics information (e.g. torque display). |
| `nimbro_robotcontrol` | `hardware` | `servomodel` | A feed-forward model to improve control of the servos. |
| `nimbro_robotcontrol` | `hardware` | `timer` | Wrapper classes for high performance timer functionality. |
| `nimbro_robotcontrol` | `util` | `rot_conv` | Rotation representation conversion library. |
| `nimbro_vis` | `contrib` | `rqt` | Import of rqt with tweaks. |
| `nimbro_vis` | `general` | `plot_msgs` | ROS messages and services used for plotting. |
| `nimbro_vis` | `general` | `plotter` | Widget for rqt that allows flexible plotting of data and events. |
| `nimbro_vis` | `general` | `rqt_log_viewer` | Widget for rqt that displays ROS messages from arbitrary nodes. |
| `nimbro_vis` | `general` | `rrlogger` | Configurable ROS topic and plot data background logger. |
| `nimbro_vis` | `general` | `timewarp` | Buffer and time transform node for ROS topics, in particular for the plotter and visualisation. |
| `nimbro_vis` | `general` | `vis_utils` | Visualisation utilities. |
| `rosmon` | `general` | `rosmon` | ROS node launch and monitor daemon. |
