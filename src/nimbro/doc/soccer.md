Playing Soccer {#soccer}
==============

This is a short guide to what is actually needed to adapt the robot to local
conditions and start playing soccer. Before starting with this guide, please
ensure that no ROS software is running on the robot.

1) Color Calibration
--------------------

Color Calibration should be one of the first steps. Please refer to the
[Calibration](@ref CalibrationPackageDoc) page.

2) Magnetometer Calibration
---------------------------

TODO

3) GameController Configuration
-------------------------------

The software needs to know some information about the soccer field. The
convention is to use a right-handed coordinate system with X pointing to the
right goal as seen from the referee table. The Y axis points away from the
referee table. The origin is the center of the field.

@image html soccer_field.png

In particular, these paremeters have to be set on the config server:

| Parameter                | Meaning                                           |
|--------------------------|---------------------------------------------------|
| `/field_type`            | `teensize` or `bonn`, depending on the field. Look at field_model for details. |
| `/field/magneticHeading` | Compass heading if the robot looks to the positive goal, see above.            |
| `/gc/teamNumber`         | Our team number assigned by the game controller.                               |
| `/gc/robotNumber`        | Our robot number assigned by the game controller.                              |
| `/gc/positiveIsYellow`   | If the positive goal corresponds to the old "yellow" goal, this is true.       |
| `/gc/enemyIsRight`       | If the game controller is not reachable, this is used to determine which goal is the enemy goal. |

4) Start
--------

Start the soccer software, either by rebooting or by starting
`roslaunch launch soccer.launch`.

The RGB LED on the back panel displays the state of the soccer control displays
the state of the SoccerManager class, which controls start/stop of the robot:

| Color    | Meaning
|----------|-----------------------|
| Red      | Robot is stopped                                            |
| Magenta  | Robot is waiting for GameController start signal ("armed")  |
| Green    | The soccer behavior is running                              |
| Cyan     | The soccer behavior was started manually and will not stop automatically |

Use the middle button to cycle through the states in the order of the above table.
