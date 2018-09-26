// Class to load joint perspective from yaml
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <trajectory_editor/jointperspective.h>

#include <stdio.h>

#include <ros/console.h>
#include <ros/common.h>
#include <ros/package.h>

#include <QDir>

namespace joint_perspective
{
	
// Perspective Manager

PerspectiveManager::PerspectiveManager()
{
	// Load all perspectives from /perspectives folder
	QStringList name_filter("*.yaml");
	QString absolute_path = QString::fromStdString(ros::package::getPath("launch") + "/motions/perspectives/");
	
	QDir dir(absolute_path);
	QStringList files = dir.entryList(name_filter);
	
	for(int i = 0; i < files.length(); i++)
	{
		JointPerspective perspective;
		
		if(perspective.load(absolute_path.toStdString() + files.at(i).toStdString()))
			m_perspectives.push_back(perspective);
	}
	
	if(m_perspectives.size() < 1)
		ROS_ERROR("0 perspectives loaded! You can do nothing now:( Perspectives must be in ***/motions/perspectives/ folder");
	else
		ROS_INFO("Perspectives initialized successfully");
}

JointPerspective& PerspectiveManager::getCurrentPerspective()
{
	return m_current_perspective;
}

// Returns true if this joint list corresponds to some new perspective (i.e. not current one)
// Ok is false if there is no such perspective
bool PerspectiveManager::isNewPerspective(std::vector<std::string>& joint_list, bool &ok)
{
	ok = true;
	
	if(m_current_perspective.correspondsTo(joint_list))
		return false;
	
	for(unsigned i = 0; i < m_perspectives.size(); i++)
	{
		if(m_perspectives.at(i).correspondsTo(joint_list))
		{
			m_current_perspective = m_perspectives.at(i); // Set new current perspective
			return true;
		}
	}
	
	ok = false;
	return false;
}

bool PerspectiveManager::isNewPerspective(const std::string perspective, bool &ok)
{
	ok = true;
	
	if(m_current_perspective.m_name == perspective)
		return false;
	
	for(unsigned i = 0; i < m_perspectives.size(); i++)
	{
		if(m_perspectives.at(i).m_name == perspective)
		{
			m_current_perspective = m_perspectives.at(i); // Set new current perspective
			return true;
		}
	}
		
	ok = false;
	return false;
}

const std::vector< JointPerspective >& PerspectiveManager::getPerspectives()
{
	return m_perspectives;
}

PerspectiveManager::~PerspectiveManager()
{

}
	
// Joint Perspective

JointPerspective::JointPerspective()
{

}

// If all joints from JointPerspective are in joint_list
// -> this JointPerspective corresponds to given joint_list
bool JointPerspective::correspondsTo(std::vector< std::string >& joint_list)
{
	if(m_joints.size() < 1) // Perspectives with 0 joints never correspond
		return false;
	
	for(unsigned i = 0; i < m_joints.size(); i++) // For each joint in Perspective
	{
		if(m_joints.at(i).spacer)
			continue;
		
		bool found = false;
		
		// Try to find the same joint in joint_list
		for(unsigned j = 0; j < joint_list.size(); j++)
		{
			if(m_joints.at(i).joint_name == joint_list.at(j))
			{
				found = true;
				break;
			}
		}
		
		if(found == false)
			return false;
	}
	
	return true;
}

bool JointPerspective::load(std::string path_to_file)
{
	m_joints.clear();
	YAML::Node node;
	
	try
	{
		node = YAML::LoadFile(path_to_file);
		nodeToPerspective(node);
	}
	catch (YAML::Exception& e)
	{
		ROS_ERROR("Error when loading joint perspective: '%s' ", e.what());
		return false;
	}
	
	return true;
}

// Parse Perspective from YAML::Node
void JointPerspective::nodeToPerspective(const YAML::Node& node)
{
	// Parse header
	YAML::Node header = node["perspective"];
	
	m_name  = header["name"].as<std::string>();
	m_model = header["model_path"].as<std::string>();
	m_robot_name = header["robot_name"].as<std::string>();
	
	m_joint_space_allowed     = (bool)header["joint_space"].as<int>();
	m_pid_space_allowed       = (bool)header["pid_space"].as<int>();
	m_abstract_space_allowed  = (bool)header["abstract_space"].as<int>();
	m_inverse_space_allowed   = (bool)header["inverse_space"].as<int>();
	
	m_link_length = header["link_length"].as<double>();
	
	// Parse joints
	YAML::Node joints = node["joints"];
	
	for (std::size_t i = 0; i < joints.size(); i++)
	{
		const YAML::Node& parsed_joint = joints[i];
		std::cout << "Name: " << parsed_joint["name"].as<std::string>() << std::endl;
		
		Joint joint;
		joint.joint_name = parsed_joint["name"].as<std::string>();
		
		// If this is a spacer - record that and continue for next joint
		if(joint.joint_name == "spacer")
		{
			joint.spacer = true;
			m_joints.push_back(joint);
			continue;
		}
		
		joint.spacer = false;
		
		// GUI name
		joint.visual_name = parsed_joint["visual_name"].as<std::string>();
		
		// Alignment
		std::string alignment = parsed_joint["alignment"].as<std::string>();
		
		if(alignment == "no_pair")
			joint.alignment = BasicSmallView::NO_PAIR;
		else if(alignment == "left")
			joint.alignment = BasicSmallView::LEFT;
		else
			joint.alignment = BasicSmallView::RIGHT;
		
		// Type
		joint.type = (BasicSmallView::Type)parsed_joint["type"].as<int>();
		
		// Mirror on shift
		joint.mirror_on_shift = (bool)parsed_joint["mirror_on_shift"].as<int>();
		
		m_joints.push_back(joint);
	}
}

JointPerspective::~JointPerspective()
{

}

}
