// Rviz visualisation of joint torques
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
//         Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef TORQUEDISPLAY_H
#define TORQUEDISPLAY_H

// Includes
#include <rviz/display.h>
#include <rviz/properties/enum_property.h>
#include <sensor_msgs/JointState.h>
#include <plot_msgs/JointCommand.h>
#include <urdf/model.h>

// Class forward declarations
namespace rviz { class Arrow; class FloatProperty; }
namespace Ogre { class SceneNode; }

// TorqueDisplay class
class TorqueDisplay : public rviz::Display
{
public:
	// Constructor/destructor
	TorqueDisplay();
	virtual ~TorqueDisplay();

	// Virtual function overrides
	virtual void onInitialize();
	virtual void update(float wall_dt, float ros_dt);
	virtual void onDisable();

	// Topic handlers
	void handleJointStates(const sensor_msgs::JointStatePtr& js);
	void handleJointCommands(const plot_msgs::JointCommandPtr& jsCmd);

private:
	// Enumerations
	enum DataSource
	{
		DS_UNKNOWN = 0,
		DS_FIRST,
		DS_POSITIONS = DS_FIRST,
		DS_VELOCITIES,
		DS_TORQUES,
		DS_LAST = DS_TORQUES,
		DS_CMD_FIRST,
		DS_CMD_POSITIONS = DS_CMD_FIRST,
		DS_CMD_VELOCITIES,
		DS_CMD_ACCELERATIONS,
		DS_CMD_RAW_POSITIONS,
		DS_CMD_TORQUES,
		DS_CMD_EFFORTS,
		DS_CMD_LAST = DS_CMD_EFFORTS,
		DS_COUNT
	};
	
	// URDF model
	boost::shared_ptr<urdf::Model> m_model;
	
	// Visualisation
	Ogre::SceneNode* m_rootNode;
	std::map<std::string, rviz::Arrow*> m_arrows;
	
	// ROS subscribers
	ros::Subscriber m_sub_js;
	ros::Subscriber m_sub_jsCmd;
	
	// Latest joint data
	sensor_msgs::JointState m_js;
	plot_msgs::JointCommand m_jsCmd;
	bool m_haveJs;
	bool m_haveJsCmd;

	// Rviz properties
	rviz::FloatProperty* m_prop_scale;
	rviz::FloatProperty* m_prop_colourRange;
	rviz::EnumProperty* m_prop_dataSource;
};

#endif
// EOF