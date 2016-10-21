// Class to apply rule parts to keyframe
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef RULEAPPLIER_H
#define RULEAPPLIER_H

#include <boost/shared_ptr.hpp>
#include <gait/util/gait_inverse_pose.h>
#include <gait/util/gait_abstract_pose.h>
#include <Eigen/Geometry>
#include <vector>

namespace motionfile
{

class Keyframe;
class RulePart;

typedef boost::shared_ptr<motionfile::Keyframe> KeyframePtr;
	
class RuleApplier
{
public:
	RuleApplier();
	~RuleApplier();
	
	bool applyRulePart(motionfile::KeyframePtr frame, const std::vector<std::string> &joint_list
	, const motionfile::RulePart &part, double delta, bool apply, bool limit_inverse, double epsilon);

private:
	bool applyJoint(motionfile::KeyframePtr frame, const std::vector<std::string> &joint_list
	, const motionfile::RulePart &part, double delta, bool apply);
	
	bool applyAbstract(motionfile::KeyframePtr frame, const std::vector<std::string> &joint_list
	, const motionfile::RulePart &part, double delta, bool apply);
	
	bool applyInverse(motionfile::KeyframePtr frame, const std::vector<std::string> &joint_list
	, const motionfile::RulePart &part, double delta, bool apply, bool limit_inverse, double epsilon);
	
	double getPos(const gait::AbstractPose& abstract_pose, const std::string& joint_name);
	void setPos(gait::AbstractPose& abstract_pose, const std::string& joint_name, const double value);
	
	double getPos(const gait::InversePose& inverse_pose, const std::string& joint_name);
	void setPos(gait::InversePose& inverse_pose, const std::string& joint_name, const double value);
	
	bool inAbstractLimits(const std::string& joint_name, const double value);
	bool inInverseLimits(const std::string& joint_name, const double value);
	
private:
	gait::InversePose m_inverse_pose;
	gait::InversePose m_temp_inverse_pose;
};

}

#endif
