
#include <ros/ros.h>

#include "testbench.h"

#include <QtGui>
#include <QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	ros::init(argc, argv, "testbench", 0);
	ros::NodeHandle nh;

	ROS_ERROR("main");

	Testbench w;
	ROS_ERROR("main: calling init");
	if(!w.init())
		return 1;

	w.show();
	return a.exec();
}
