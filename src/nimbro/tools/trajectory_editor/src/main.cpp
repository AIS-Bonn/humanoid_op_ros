// Trajectory editor
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <QApplication>
#include <ros/init.h>

#include <trajectory_editor/mainwindow.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "trajectory_editor");
	QApplication app(argc, argv);

	MainWindow win;
	win.show();

	return app.exec();
}
