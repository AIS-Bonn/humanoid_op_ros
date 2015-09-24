#ifndef MYMARKERDISPLAY_H
#define MYMARKERDISPLAY_H

#include <rviz/display.h>
#include <rviz/default_plugin/marker_display.h>

#include <ros/package.h>
#include <ros/console.h>


class MyMarkerDisplay : public rviz::MarkerDisplay
{
public:
	MyMarkerDisplay();
	//virtual ~RobotDisplay();
	
	void update( float wall_dt, float ros_dt );
};

#endif
