//Library to manage motion files
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>, Hafez Farazi <farazi@ais.uni-bonn.de>, Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef MOTIONFILE_H
#define MOTIONFILE_H

#include <map>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <ros/common.h>

#include "keyframe_player/Keyframe.h"
#include <motion_file/ruleapplier.h>

namespace motionfile
{

struct FrameJoint
{
	FrameJoint();

	double position;
	double velocity;
	double effort;
	double pGain; double iGain; double dGain;
	double limit;
	kf_player::gainSelectEnum gainSelect;
};

struct Keyframe
{
	int id;
	std::string name;
	std::vector<FrameJoint> joints;
	double duration;
	double roll; double pitch;double yaw;
	std::string support;
};

struct RulePart
{
	std::string type;
	std::string frameName;
	std::string jointName;
	double      scale;
};

struct Rule
{
	std::string name;
	std::vector<RulePart> parts;
};

class Motion
{
public:
	Motion();
	~Motion(){};

	Motion(const Motion& _in) :
		motionName(_in.motionName) ,
		preState(_in.preState) ,
		playState(_in.playState) ,
		postState(_in.postState) ,
		pidEnabled(_in.pidEnabled) ,
		jointList(_in.jointList) ,
		perspective(_in.perspective) ,
		filePath(_in.filePath)
	{
		frames.clear();
		for(size_t i=0;i<_in.frames.size();i++)
		{
			frames.push_back(boost::make_shared<Keyframe>(*_in.frames[i]));
		}
		
		rules.clear();
		for(size_t i=0;i<_in.rules.size();i++)
		{
			rules.push_back(Rule());
			rules.back().name = _in.rules[i].name;
			
			for(size_t j = 0; j < _in.rules[i].parts.size(); j++)
				rules[i].parts.push_back((_in.rules[i].parts[j]));
		}

	} // copy ctor

	Motion& operator=(Motion _in)
	{
		if (this != &_in)
		{
			motionName=(_in.motionName);
			preState=(_in.preState);
			playState=(_in.playState);
			postState=(_in.postState);
			pidEnabled=(_in.pidEnabled);
			jointList=(_in.jointList);
			perspective=(_in.perspective);
			filePath=(_in.filePath);
			
			frames.clear();
			for(size_t i=0;i<_in.frames.size();i++)
			{
				frames.push_back(boost::make_shared<Keyframe>(*_in.frames[i]));
			}
			
			rules.clear();
			for(size_t i=0;i<_in.rules.size();i++)
			{
				rules.push_back(Rule());
				rules.back().name = _in.rules[i].name;
				
				for(size_t j = 0; j < _in.rules[i].parts.size(); j++)
					rules[i].parts.push_back((_in.rules[i].parts[j]));
			}
		}
		return *this;
	}

	bool load(std::string name);
	bool parse(std::string motion);
	bool save(std::string name);
	std::string dump();
	
	int findFrame(const std::string name);
	int findJoint(const std::string name);
	
	bool isRuleValid(int index);
	bool canApplyRule(int index, double delta, bool limit_inverse, double epsilon);
	bool applyRule(int index, double delta, bool limit_inverse, double epsilon);
	
	static int nameToIndex(const std::vector<std::string>& jointList, const std::string& name);
	static int nameToIndexPrintError(const std::vector<std::string>& jointList, const std::string& name);
	
	// Returns true if motions are exactly the same
	static bool isIdentical(motionfile::Motion& motion_1, motionfile::Motion& motion_2);
	
	// Provides detailed output about each not equal variable
	static void detailedComparison(motionfile::Motion& motion_1, motionfile::Motion& motion_2); 

	typedef boost::shared_ptr<Keyframe> KeyframePtr;

//header
	std::string motionName;
	std::string preState;
	std::string playState;
	std::string postState;
	bool pidEnabled;
	
	std::vector<std::string> jointList;
	std::vector<Rule> rules;
	
	std::string perspective; // perspective for trajectory editor
	std::string filePath; // Path to file from which the motion was loaded

//keyframes
	std::vector<KeyframePtr> frames;

private:
	void nodeToMotion(const YAML::Node& node);
	void parseRule(const YAML::Node& node);
	void motionToNode(YAML::Emitter& em);
	
	motionfile::RuleApplier ruleApplier;
};



}

#endif
