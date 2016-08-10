// Incorporates a RViz display
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <trajectory_editor_2/rvizwidget.h>
#include <trajectory_editor_2/robotdisplay.h>

#include <rviz/visualization_manager.h>
#include <rviz/robot/robot.h>

#include <sensor_msgs/JointState.h>
#include <OGRE/OgreColourValue.h>

#include <ros/package.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

void RVizWidget::test(const visualization_msgs::Marker::ConstPtr& msg)
{
	ROS_INFO("MSG!");
}

RVizWidget::RVizWidget(QWidget* parent)
 : RenderPanel(parent)
{
	m_manager = new rviz::VisualizationManager(this);

	RenderPanel::initialize(m_manager->getSceneManager(), m_manager);
    RenderPanel::setBackgroundColor(Ogre::ColourValue(0.75f, 0.75f, 0.75f, 1.0f));

	m_manager->initialize();
	m_manager->startUpdate();
}

void RVizWidget::initialize(ros::NodeHandle* nh)
{
	m_model = boost::make_shared<urdf::Model>();

	m_model->initParam("/robot_description");
	m_robot = new RobotDisplay(m_model);

	m_manager->addDisplay(m_robot, true);
	
// 	myDisplay = new MyMarkerDisplay();
// 	myDisplay->setTopic(QString("/foo/bar"), QString("visualization_msgs/Marker"));
// 	m_manager->addDisplay(myDisplay, true);
	
	/*markerDisplay = new rviz::MarkerDisplay();
	markerDisplay->setTopic(QString("/foo/bar"), QString("visualization_msgs/Marker"));
	m_manager->addDisplay(markerDisplay, true);*/
	
	//sub = n.subscribe("visualization_marker", 100, test);
}

RVizWidget::~RVizWidget()
{
	delete m_manager;
}
