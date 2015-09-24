// Rviz visualisation of joint torques
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
//         Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rviz_dynamics/torquedisplay.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/properties/float_property.h>
#include <OGRE/OgreSceneManager.h>
#include <pluginlib/class_list_macros.h>

// Namespaces
using namespace Ogre;

//
// TorqueDisplay class
//

TorqueDisplay::TorqueDisplay()
 : m_rootNode(NULL)
 , m_haveJs(false)
 , m_haveJsCmd(false)
{
	ros::NodeHandle nh;

	m_sub_js = nh.subscribe("/vis/joint_states", 1, &TorqueDisplay::handleJointStates, this);
	m_sub_jsCmd = nh.subscribe("/vis/joint_commands", 1, &TorqueDisplay::handleJointCommands, this);

	m_model = boost::make_shared<urdf::Model>();
	if(!m_model->initParam("robot_description"))
		ROS_ERROR("Could not get URDF model");

	m_prop_scale = new rviz::FloatProperty("Scale", 0.2, "The vectors displayed on each joint are scaled by this factor for visualisation purposes", this);
	m_prop_colourRange = new rviz::FloatProperty("Colour Range", 3.0, "A vector of this magnitude or above will have the full green or red colour", this);
	m_prop_dataSource = new rviz::EnumProperty("Data Source", "Torques", "Which joint data to display", this);
	
	m_prop_dataSource->addOptionStd("Positions", DS_POSITIONS);
	m_prop_dataSource->addOptionStd("Velocities", DS_VELOCITIES);
	m_prop_dataSource->addOptionStd("Torques", DS_TORQUES);
	m_prop_dataSource->addOptionStd("Cmd Positions", DS_CMD_POSITIONS);
	m_prop_dataSource->addOptionStd("Cmd Velocities", DS_CMD_VELOCITIES);
	m_prop_dataSource->addOptionStd("Cmd Accelerations", DS_CMD_ACCELERATIONS);
	m_prop_dataSource->addOptionStd("Cmd Raw Positions", DS_CMD_RAW_POSITIONS);
	m_prop_dataSource->addOptionStd("Cmd Torques", DS_CMD_TORQUES);
	m_prop_dataSource->addOptionStd("Cmd Efforts", DS_CMD_EFFORTS);
}

TorqueDisplay::~TorqueDisplay()
{
	onDisable();
	delete m_prop_scale;
	delete m_prop_colourRange;
	delete m_prop_dataSource;
}

void TorqueDisplay::onInitialize()
{
	m_rootNode = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode("torque_root");
}

void TorqueDisplay::onDisable()
{
	rviz::Display::onDisable();
	for(std::map<std::string, rviz::Arrow*>::iterator it = m_arrows.begin(); it != m_arrows.end(); ++it)
		delete it->second;
	m_arrows.clear();
}

void TorqueDisplay::handleJointStates(const sensor_msgs::JointStatePtr& js)
{
	m_js = *js;
	m_haveJs = true;
}

void TorqueDisplay::handleJointCommands(const plot_msgs::JointCommandPtr& jsCmd)
{
	m_jsCmd = *jsCmd;
	m_haveJsCmd = true;
}

void TorqueDisplay::update(float wall_dt, float ros_dt)
{
	if(!context_ || !isEnabled()) return;

	rviz::FrameManager* fmgr = context_->getFrameManager();
	if(!fmgr) return;
	
	int source = m_prop_dataSource->getOptionInt();
	
	const std::vector<std::string>* jointName = NULL;
	if(source >= DS_FIRST && source <= DS_LAST && m_haveJs) jointName = &m_js.name;
	if(source >= DS_CMD_FIRST && source <= DS_CMD_LAST && m_haveJsCmd) jointName = &m_jsCmd.name;
	if(!jointName) return;
	
	const std::vector<double>* jointValue = NULL;
	if(source == DS_POSITIONS) jointValue = &m_js.position;
	if(source == DS_VELOCITIES) jointValue = &m_js.velocity;
	if(source == DS_TORQUES) jointValue = &m_js.effort;
	if(source == DS_CMD_POSITIONS) jointValue = &m_jsCmd.position;
	if(source == DS_CMD_VELOCITIES) jointValue = &m_jsCmd.velocity;
	if(source == DS_CMD_ACCELERATIONS) jointValue = &m_jsCmd.acceleration;
	if(source == DS_CMD_RAW_POSITIONS) jointValue = &m_jsCmd.rawPosition;
	if(source == DS_CMD_TORQUES) jointValue = &m_jsCmd.torque;
	if(source == DS_CMD_EFFORTS) jointValue = &m_jsCmd.effort;
	if(!jointValue) return;

	for(size_t i = 0; i < jointValue->size(); ++i)
	{
		boost::shared_ptr<const urdf::Joint> joint = m_model->getJoint(jointName->at(i));
		if(!joint)
			continue;

		std::map<std::string, rviz::Arrow*>::iterator it = m_arrows.find(jointName->at(i));

		geometry_msgs::Pose pose;
		pose.position.x = pose.position.y = pose.position.z = 0;
		pose.orientation.w = 1;
		pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;

		Ogre::Vector3 pos;
		Ogre::Quaternion rot;
		if(!fmgr->transform(joint->child_link_name, ros::Time(0), pose, pos, rot))
			continue;

		rviz::Arrow* arrow;

		if(it == m_arrows.end())
		{
			Ogre::SceneNode* sn = m_rootNode->createChildSceneNode();

			arrow = new rviz::Arrow(scene_manager_, sn);

			m_arrows[jointName->at(i)] = arrow;
		}
		else
			arrow = it->second;

		arrow->getSceneNode()->getParentSceneNode()->setPosition(pos);
		arrow->getSceneNode()->getParentSceneNode()->setOrientation(rot);

		double scaledValue = jointValue->at(i) * m_prop_scale->getFloat();
		Vector3 jointAxis(joint->axis.x, joint->axis.y, joint->axis.z);
		arrow->setDirection(scaledValue >= 0.0 ? jointAxis : -jointAxis);
		double lambda = 0.5 + 0.5*(jointValue->at(i) / m_prop_colourRange->getFloat());
		if(lambda < 0.0) lambda = 0.0;
		else if(lambda > 1.0) lambda = 1.0;
		arrow->setColor(1.0 - lambda, lambda, 0.0, 1.0);
		arrow->set(fabs(scaledValue), 0.01, 0.03, 0.02);
	}
}

PLUGINLIB_EXPORT_CLASS(TorqueDisplay, rviz::Display);
// EOF