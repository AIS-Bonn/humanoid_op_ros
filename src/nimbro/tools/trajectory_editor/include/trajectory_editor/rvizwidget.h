// Incorporates a RViz display
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef RVIZWIDGET_H
#define RVIZWIDGET_H

#include <rviz/visualization_frame.h>
#include <rviz/render_panel.h>
#include <rviz/default_plugin/marker_display.h>
#include <trajectory_editor/mymarkerdisplay.h>

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
	void setCameraPosition();
	
public Q_SLOTS:
	void setModel(std::string path_to_model);
	void setModel(std::string path_to_model, std::vector<std::string> &new_joint_list);
	void updateRobotDisplay(std::vector<std::string> &joint_list, std::vector<double> &joint_positions);
	
private:
	void setAdultParallelJoints(const std::vector<std::string> &joint_list, std::vector<double> &joint_positions, const std::string side);

private:
	rviz::VisualizationManager* m_manager;
    boost::shared_ptr<urdf::Model> m_model;
	std::string m_path_to_model;
	RobotDisplay* m_robot;
	std::vector<boost::shared_ptr<urdf::Joint> > m_mimic_joints; // All mimic joints in the model
	
	MyMarkerDisplay     *myDisplay;
	rviz::MarkerDisplay *markerDisplay;
	ros::NodeHandle n;
	ros::Subscriber sub;
};

#endif
