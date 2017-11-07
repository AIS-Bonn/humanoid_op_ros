// Class to load joint perspective from yaml
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef JOINT_PERSPECTIVE_H
#define JOINT_PERSPECTIVE_H

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>

#include <trajectory_editor/spaces/basicsmallview.h>

namespace joint_perspective
{
	
struct Joint
{
	std::string joint_name;  // The actual name of a joint
	std::string visual_name; // The name you see in GUI
	
	BasicSmallView::Alignment alignment;
	BasicSmallView::Type      type;
	
	bool mirror_on_shift;
	bool spacer;
};

class JointPerspective
{
public:
	JointPerspective();
	~JointPerspective();
	
	bool load(std::string path_to_file);
	bool correspondsTo(std::vector<std::string> &joint_list);
	
public:
	std::string m_name;
	std::string m_model;
	std::string m_robot_name;
	
	bool m_joint_space_allowed;
	bool m_pid_space_allowed;
	bool m_abstract_space_allowed;
	bool m_inverse_space_allowed;
	
	double m_link_length; // Link length for inverse space
	
	std::vector<Joint> m_joints;
	
private:
	void nodeToPerspective(const YAML::Node& node);
};


// Wrapper around vector of perspectives
// TODO move to separate file?
class PerspectiveManager
{
public:
	PerspectiveManager();
	~PerspectiveManager();
	
	bool isNewPerspective(std::vector<std::string> &joint_list, bool &ok);
	bool isNewPerspective(const std::string perspective, bool &ok);
	
	JointPerspective& getCurrentPerspective();
	const std::vector<JointPerspective>& getPerspectives();
	
private:
	std::vector<JointPerspective> m_perspectives;
	JointPerspective m_current_perspective;
};

}

#endif
