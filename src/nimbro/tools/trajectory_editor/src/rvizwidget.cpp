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

#include <motion_file/motionfile.h>
#include <string>

void RVizWidget::test(const visualization_msgs::Marker::ConstPtr& msg)
{
	ROS_INFO("MSG!");
}

RVizWidget::RVizWidget(QWidget* parent) : RenderPanel(parent)
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
	m_path_to_model = path_to_model;
	path_to_model = ros::package::getPath("nimbro_op_model") + path_to_model;
	
	if(m_model->initFile(path_to_model))
		ROS_INFO("Successfully loaded model %s", path_to_model.c_str());
	else
		ROS_ERROR("Error ocurred when loading model %s", path_to_model.c_str());

	m_robot = new RobotDisplay(m_model);
	m_manager->removeAllDisplays();
	m_manager->addDisplay(m_robot, true);
	
	// Get mimic joints
	std::vector<boost::shared_ptr<urdf::Link> > links;
	m_model->getLinks(links);
	m_mimic_joints.clear();

	for(size_t i = 0; i < links.size(); i++)
	{
		const boost::shared_ptr<urdf::Joint>& modelJoint = links[i]->parent_joint;

		// Ignore joints we don't need to handle (fixed joints and not-mimic joints)
		if(!modelJoint || (
				   modelJoint->type != urdf::Joint::CONTINUOUS
				&& modelJoint->type != urdf::Joint::REVOLUTE)
				|| modelJoint->mimic == nullptr)
			continue;
		
		m_mimic_joints.push_back(modelJoint);
		//ROS_INFO("Got mimic joint: %s. Anchored to: %s", modelJoint->name.c_str(), modelJoint->mimic->joint_name.c_str());
	}
	
	ROS_INFO("Display initialized.");
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
	
	// HACK  for adult robot   TODO: implement nicely
	if(m_path_to_model == "/robots/nimbro_adult.urdf" || m_path_to_model == "/robots/nimbro_op2x_hull.urdf")
	{
		// Parallel joints
		setAdultParallelJoints(joint_list, joint_positions, "right");
		setAdultParallelJoints(joint_list, joint_positions, "left");
		
		// Mimic joints
		for(size_t i = 0; i < m_mimic_joints.size(); i++)
		{
			int mimic  = motionfile::Motion::nameToIndexPrintError(joint_list, m_mimic_joints.at(i)->name);
			int anchor = motionfile::Motion::nameToIndexPrintError(joint_list, m_mimic_joints.at(i)->mimic->joint_name);
			
			if(mimic == -1 || anchor == -1)
				continue;
			
			joint_positions.at(mimic) = joint_positions.at(anchor) * m_mimic_joints.at(i)->mimic->multiplier + m_mimic_joints.at(i)->mimic->offset;
		}
	}
	
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

// Serial --> parallel for adult robot
void RVizWidget::setAdultParallelJoints(const std::vector<std::string> &joint_list, std::vector<double> &joint_positions, const std::string side)
{
	int tp =  motionfile::Motion::nameToIndexPrintError(joint_list, "parallel_" + side + "_thigh_pitch");		
	int hip = motionfile::Motion::nameToIndexPrintError(joint_list, side + "_hip_pitch");
		
	int sp =  motionfile::Motion::nameToIndexPrintError(joint_list, "parallel_" + side + "_shank_pitch");		
	int knee = motionfile::Motion::nameToIndexPrintError(joint_list, side + "_knee_pitch");
		
	int ankle_roll_p =  motionfile::Motion::nameToIndexPrintError(joint_list, "parallel_" + side + "_ankle_roll");		
	int ankle_roll   = motionfile::Motion::nameToIndexPrintError(joint_list, side + "_ankle_roll");
	
	if(tp == -1 || hip == -1 || sp == -1 || knee == -1 || ankle_roll_p == -1 || ankle_roll == -1)
		return;
	
	joint_positions.at(tp) = joint_positions.at(hip);
	joint_positions.at(sp) = joint_positions.at(hip) + joint_positions.at(knee);
	joint_positions.at(ankle_roll_p) = joint_positions.at(ankle_roll);
}


RVizWidget::~RVizWidget()
{
	delete m_manager;
}
