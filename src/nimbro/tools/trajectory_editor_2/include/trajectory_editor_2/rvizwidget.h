// Incorporates a RViz display
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef RVIZWIDGET_H
#define RVIZWIDGET_H

#include <rviz/visualization_frame.h>
#include <rviz/render_panel.h>
#include <rviz/default_plugin/marker_display.h>
// #include <trajectory_editor_2/mymarkerdisplay.h>

#include <ros/publisher.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>

namespace rviz
{
	class VisualizationManager;
}

class RobotDisplay;

class RVizWidget : public rviz::RenderPanel
{
Q_OBJECT
public:
	explicit RVizWidget(QWidget* parent = 0);
	virtual ~RVizWidget();

	void initialize(ros::NodeHandle* nh);

    inline boost::shared_ptr<urdf::Model> getModel() {return m_model;}
    inline RobotDisplay* getRobot() {return m_robot;}
    
    void test(const visualization_msgs::Marker::ConstPtr& msg);

private:
	rviz::VisualizationManager* m_manager;
    boost::shared_ptr<urdf::Model> m_model;
	RobotDisplay* m_robot;
	
// 	MyMarkerDisplay     *myDisplay;
	rviz::MarkerDisplay *markerDisplay;
	ros::NodeHandle n;
	ros::Subscriber sub;
};

#endif
