#include <trajectory_editor_2/mymarkerdisplay.h>

MyMarkerDisplay::MyMarkerDisplay() : MarkerDisplay()
{

}

void MyMarkerDisplay::update( float wall_dt, float ros_dt )
{
	rviz::MarkerDisplay::update(wall_dt, ros_dt);
	//ROS_INFO("Update MarkerDisplay");
}



