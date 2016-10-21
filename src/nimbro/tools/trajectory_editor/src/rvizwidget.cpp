// Incorporates a RViz display
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <trajectory_editor/rvizwidget.h>
#include <trajectory_editor/robotdisplay.h>

#include <rviz/view_controller.h>
#include <rviz/visualization_manager.h>
#include <rviz/robot/robot.h>

#include <sensor_msgs/JointState.h>
#include <OGRE/OgreColourValue.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <std_msgs/String.h>

#include <OGRE/OgreViewport.h>
#include <OGRE/OgreCamera.h>

#include <string>

void RVizWidget::test(const visualization_msgs::Marker::ConstPtr& msg)
{
	ROS_INFO("MSG!");
}

RVizWidget::RVizWidget(QWidget* parent)
 : RenderPanel(parent)
{
	m_model = boost::make_shared<urdf::Model>();
	m_manager = new rviz::VisualizationManager(this);
	m_robot = NULL;

	RenderPanel::initialize(m_manager->getSceneManager(), m_manager);
    RenderPanel::setBackgroundColor(Ogre::ColourValue(0.75f, 0.75f, 0.75f, 1.0f));
	
	m_manager->initialize();
}

void RVizWidget::initialize(ros::NodeHandle* nh)
{
	//m_model->initParam("/robot_description");
	//m_model->initFile("/home/mark/dynaped_model/urdf/dynaped.urdf"); //igus_op
	m_model->initFile(ros::package::getPath("nimbro_op_model")+ "/robots/urdf/igus_op.urdf");
	
	m_robot = new RobotDisplay(m_model);
	m_manager->addDisplay(m_robot, true);
	setCameraPosition();
	
	m_manager->startUpdate();
	
	//myDisplay = new MyMarkerDisplay();
	//myDisplay->setTopic(QString("/foo/bar"), QString("visualization_msgs/Marker"));
	//m_manager->addDisplay(myDisplay, true);
	
	//setup();
	
	/*markerDisplay = new rviz::MarkerDisplay();
	markerDisplay->setTopic(QString("/foo/bar"), QString("visualization_msgs/Marker"));
	m_manager->addDisplay(markerDisplay, true);*/
	
	//sub = n.subscribe("visualization_marker", 100, test);
}

// TODO probably set from perspective file
void RVizWidget::setCameraPosition()
{
	Ogre::Camera *camera = getViewport()->getCamera();

	camera->setPosition(1.36, 0.0, 0.58);
	getViewController()->lookAt(0, 0, 0);
}

// Set model from provided path
void RVizWidget::setModel(std::string path_to_model)
{
	ROS_INFO("Set model %s", path_to_model.c_str());
	if(path_to_model == "P1") // TODO refactor it!
		m_model->initFile(ros::package::getPath("nimbro_op_model")+ "/robots/urdf/igus_op.urdf");
		//m_model->initParam("/robot_description");
	else
		m_model->initFile(path_to_model);

	//m_model->initFile(ros::package::getPath("nimbro_op_model")+ "/robots/urdf/igus_op.urdf");
	m_robot = new RobotDisplay(m_model);
	m_manager->removeAllDisplays();
	m_manager->addDisplay(m_robot, true);
}

// Set model and write new joint list into provided vector
void RVizWidget::setModel(std::string path_to_model, std::vector<std::string> &new_joint_list)
{
	setModel(path_to_model);
	
	boost::shared_ptr<rbdl_parser::URDF_RBDL_Model> rbdl = m_robot->getRBDL();
	
	for (unsigned i = 0; i < rbdl->mJoints.size(); i++)
		new_joint_list.push_back(rbdl->jointName(i));
}

// Update robot pose using provided joint positions
void RVizWidget::updateRobotDisplay(std::vector<std::string> &joint_list, std::vector<double> &joint_positions)
{
	if(m_robot == NULL)
		return;
	
	std::vector<double> positions(m_robot->getRBDL()->dof_count, 0);
	
	for (unsigned i = 0; i < joint_list.size(); i++)
	{
		if (joint_list[i].empty())
			continue;

		int ind = m_robot->getRBDL()->jointIndex(joint_list[i]) - 1;
		positions[ind] = joint_positions[i];
	}
	
	m_robot->update(positions);
}

RVizWidget::~RVizWidget()
{
	delete m_manager;
}
