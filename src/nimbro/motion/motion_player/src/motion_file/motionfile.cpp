//Library to manage motion files
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>, Hafez Farazi <farazi@ais.uni-bonn.de>, Dmytro Pavlichenko <dm.mark999@gmail.com>

#include "motion_file/motionfile.h"

#include <fstream>

const std::string DEFAULT_KEYFRAME_SUPPORT  = "";
const std::string DEFAULT_PERSPECTIVE = "P1";
const double      DEFAULT_VELOCITY = 0;
const double      DEFAULT_CONTROL_GAIN = 0;
const double      DEFAULT_ANGLE = 0;
const double      DEFAULT_LIMIT = M_PI;
const bool        DEFAULT_PID_ENABLED = false;

using namespace motionfile;

Motion::Motion()
{
	filePath = "";
}

FrameJoint::FrameJoint()
 : position(0)
 , velocity(0)
 , effort(1)
 , pGain(0.0)
 , iGain(0.0)
 , dGain(0.0)
 , gainSelect(kf_player::nonE)
{}


bool Motion::load(std::string name)
{
	//Delete keyframes in case the motion loads more than once
	frames.clear();
	jointList.clear();

	YAML::Node node;
	try
	{
		filePath = name;
		node = YAML::LoadFile(name);
		nodeToMotion(node);
	}
	catch (YAML::Exception& e)
	{
		ROS_ERROR("Error: '%s' ", e.what());
		return false;
	}
	return true;
}

bool Motion::parse(std::string motion)
{
	//see above
	frames.clear();
	jointList.clear();

	YAML::Node node;
	try
	{
		node = YAML::Load(motion);
		nodeToMotion(node);
	}
	catch (YAML::Exception& e)
	{
		ROS_ERROR("Error: '%s' ", e.what());
		return false;
	}
	return true;
}


void Motion::nodeToMotion(const YAML::Node& node)
{
	YAML::Node header    = node["header"];
	motionName = header["name"].as< std::string >();
	preState   = header["preState"].as< std::string >();
	playState  = header["playState"].as< std::string >();
	postState  = header["postState"].as< std::string >();
	
	if(header["pid_enabled"]) 
	{
		if(header["pid_enabled"].as< int >() > 0)
			pidEnabled = true;
		else
			pidEnabled = false;
	}
	else
		pidEnabled = DEFAULT_PID_ENABLED;
	
	if(header["perspective"]) 
		perspective = header["perspective"].as< std::string >();
	else
		perspective = DEFAULT_PERSPECTIVE;
	
	parseRule(node);
	
	YAML::Node keyframes = node["motion"];
	YAML::const_iterator it=keyframes.begin();
	YAML::Node joints = (*it)["joints"];
	
	// Construct joint list, sorted in alphabetical order
	for (YAML::const_iterator jit=joints.begin(); jit!=joints.end(); ++jit)
		jointList.push_back(jit->first.as<std::string>());
	std::sort(jointList.begin(), jointList.end());

	// Parse keyframes
	for (it = keyframes.begin(); it != keyframes.end(); ++it)
	{
		KeyframePtr kf(new Keyframe);
		//per keyframe information
		
		if((*it)["frameName"])
			kf->name = (*it)["frameName"].as<std::string>();
		else
			kf->name = "";
			
		kf->duration = (*it)["duration"].as<double>();
		if ((*it)["support"])
			kf->support = (*it)["support"].as< std::string>();
		else
			kf->support = DEFAULT_KEYFRAME_SUPPORT;

		if ((*it)["roll"])
			kf->roll = (*it)["roll"].as< double>();
		else
			kf->roll = DEFAULT_ANGLE;

		if ((*it)["pitch"])
			kf->pitch = (*it)["pitch"].as< double>();
		else
			kf->pitch = DEFAULT_ANGLE;

		if ((*it)["yaw"])
			kf->yaw = (*it)["yaw"].as< double>();
		else
			kf->yaw = DEFAULT_ANGLE;

		joints = (*it)["joints"];
		
		for (YAML::const_iterator jit=joints.begin(); jit!=joints.end(); ++jit)
		{
			FrameJoint j;

			//per keyframe per joint information
			j.position = jit->second["position"].as<double>();
			j.effort   = jit->second["effort"].as<double>();
			if (jit->second["velocity"])
				j.velocity = jit->second["velocity"].as<double>();
			else
				j.velocity = DEFAULT_VELOCITY;
			//-------------------------------------
			if (jit->second["p_gain"])
				j.pGain = jit->second["p_gain"].as<double>();
			else
				j.pGain = DEFAULT_CONTROL_GAIN;
			//-------------------------------------
			if (jit->second["i_gain"])
				j.iGain = jit->second["i_gain"].as<double>();
			else
				j.iGain = DEFAULT_CONTROL_GAIN;
			//-------------------------------------
			if (jit->second["d_gain"])
				j.dGain = jit->second["d_gain"].as<double>();
			else
				j.dGain = DEFAULT_CONTROL_GAIN;
			//-------------------------------------
			if (jit->second["limit"])
				j.limit = jit->second["limit"].as<double>();
			else
				j.limit = DEFAULT_LIMIT;
			//-------------------------------------
			if (jit->second["gain_select"])
				j.gainSelect = (kf_player::gainSelectEnum)(jit->second["gain_select"].as<int>());
			else
				j.gainSelect = kf_player::nonE;
			//-------------------------------------

			//remember the jointnames for association
			//only motions with the same amount and types of joints are
			//supported.
			std::string jn = jit->first.as<std::string>();
			std::vector<std::string>::iterator it;
			it = std::find(jointList.begin(),jointList.end(),jn);
			
			if (it == jointList.end())
			{
				//if the joint is encountered the first time, safe the name for
				//association
				jointList.push_back(jn);
				kf->joints.push_back(j);
			}
			else
			{
				//already encountered joints are added in the same order as the
				//first time
				kf->joints.resize(jointList.size());
				int index = it - jointList.begin();
				kf->joints[index] = j;
			}

		}
		
		frames.push_back(kf);
	}
}

void Motion::parseRule(const YAML::Node& node)
{
	rules.clear();
	YAML::Node rulesNode = node["rules"];
	YAML::Node partsNode;
	
	if(!rulesNode)
		return;
	
	// Parse rules
	for (std::size_t i = 0; i < rulesNode.size(); i++)
	{
		rules.push_back(Rule());
		
		partsNode = rulesNode[i];
		rules.back().name = partsNode["name"].as<std::string>();
	
		//printf("Rule %d \n\n", (int)i);
		
		// Parse rule parts
		partsNode = partsNode["parts"];
		for (std::size_t j = 0; j < partsNode.size(); j++)
		{
			const YAML::Node& parsedRulePart = partsNode[j];
			RulePart rulePart;
			
			rulePart.type = parsedRulePart["type"].as<std::string>();
			//printf("type %s \n", rulePart.type.c_str());
			rulePart.frameName = parsedRulePart["frameName"].as<std::string>();
			rulePart.jointName = parsedRulePart["jointName"].as<std::string>();
			rulePart.scale = parsedRulePart["scale"].as<double>();
			
			rules.back().parts.push_back(rulePart);
		}
	}
}

bool Motion::isRuleValid(int index)
{
	if(index < 0 || index >= (int)rules.size())
		return false;
	
	Rule rule = rules[index];
	
	for(size_t i = 0; i < rule.parts.size(); i++)
	{
		// Check type
		if(rule.parts[i].type != "joint" && rule.parts[i].type != "abstract" && rule.parts[i].type != "inverse")
		{
			printf("Rule part %d has invalid 'type'\n", (int)i);
			printf("CANT APPLY RULE '%s'\n", rules[index].name.c_str());
			return false;
		}
		
		// Check frame
		if(findFrame(rule.parts[i].frameName) == -1)
		{
			printf("Rule part %d has 'frameName' which does not exist in motion\n", (int)i);
			printf("CANT APPLY RULE '%s'\n", rules[index].name.c_str());
			return false;
		}
	}
	
	return true;
}

bool Motion::canApplyRule(int index, double delta, bool limit_inverse, double epsilon)
{
	if(!isRuleValid(index))
		return false;
	
	for(size_t i = 0; i < rules[index].parts.size(); i++)
	{
		int frame_index = findFrame(rules[index].parts[i].frameName);
		if(!ruleApplier.applyRulePart(frames[frame_index], jointList, rules[index].parts[i], delta, false, limit_inverse, epsilon))
		{
			printf("CANT APPLY RULE '%s'\n", rules[index].name.c_str());
			return false;
		}
	}
	
	return true;
}

bool Motion::applyRule(int index, double delta, bool limit_inverse, double epsilon)
{
	if(!isRuleValid(index))
		return false;
	
	for(size_t i = 0; i < rules[index].parts.size(); i++)
	{
		int frame_index = findFrame(rules[index].parts[i].frameName);
		if(!ruleApplier.applyRulePart(frames[frame_index], jointList, rules[index].parts[i], delta, true, limit_inverse, epsilon))
		{
			printf("CANT APPLY RULE '%s'\n", rules[index].name.c_str());
			return false;
		}
		
		printf("Rule '%s' was successfully applied with delta: %f\n", rules[index].name.c_str(), delta);
	}
	
	return true;
}

bool Motion::save(std::string name)
{

	YAML::Emitter em;

	motionToNode(em);

	std::ofstream out;

	out.open(name.c_str());
	out << em.c_str() << "\n";
	out.close();
	return true;
}

std::string Motion::dump()
{
	YAML::Emitter em;

	motionToNode(em);

	std::stringstream oss;
	oss << em.c_str() << std::endl;


	return oss.str();
}

// Returns index of frame with given name. If frame does not exist - returns -1
int Motion::findFrame(const std::string name)
{
	for(size_t i = 0; i < frames.size(); i++)
	{
		if(frames[i]->name == name)
			return (int)i;
	}
	
	return -1;
}

// Returns index of joint with given name. If joint does not exist - returns -1
int Motion::findJoint(const std::string name)
{
	for(size_t i = 0; i < jointList.size(); i++)
	{
		if(jointList[i] == name)
			return (int)i;
	}
	
	return -1;
}

// Finds joint with given name and returns its index
// Returns -1 if name was not found
int Motion::nameToIndex(const std::vector<std::string>& jointList, const std::string& name)
{
	for (unsigned i = 0; i < jointList.size(); i++)
	{
		if (jointList[i] == name)
			return i;
	}
	return -1;
}

// Prints error message if requested joint was not found
int Motion::nameToIndexPrintError(const std::vector< std::string >& jointList, const std::string& name)
{
	int id = motionfile::Motion::nameToIndex(jointList, name);
	
	if(id == -1)
		ROS_ERROR("Joint '%s' was not found!", name.c_str());
	
	return id;
}

void Motion::motionToNode(YAML::Emitter& em)
{
	em << YAML::BeginMap;

	//write header
	em << YAML::Key << "header";
	em << YAML::Value << YAML::BeginMap;

	em << YAML::Key << "name" << YAML::Value << motionName;
	em << YAML::Key << "preState" << YAML::Value << preState;
	em << YAML::Key << "playState" << YAML::Value << playState;
	em << YAML::Key << "postState" << YAML::Value << postState;
	em << YAML::Key << "pid_enabled" << YAML::Value << (int)pidEnabled;
	em << YAML::Key << "perspective" << YAML::Value << perspective;

	em << YAML::EndMap;
	
	//write rules
	em << YAML::Key << "rules" << YAML::Value << YAML::BeginSeq;
	for (size_t i = 0; i < rules.size(); i++)
	{
		Rule rule = rules[i];
		
		em << YAML::BeginMap;
		em << YAML::Key << "name" << YAML::Value << rule.name;
		em << YAML::Key << "parts" << YAML::Value << YAML::BeginSeq;
		for(size_t j = 0; j < rule.parts.size(); j++)
		{
			em << YAML::BeginMap;
			em << YAML::Key << "type" << YAML::Value << rule.parts[j].type;
			em << YAML::Key << "frameName"<< YAML::Value << rule.parts[j].frameName;
			em << YAML::Key << "jointName" << YAML::Value << rule.parts[j].jointName;
			em << YAML::Key << "scale" << YAML::Value << rule.parts[j].scale;
			em << YAML::EndMap;
		}
		em << YAML::EndSeq;
		em << YAML::EndMap;
	}
	em << YAML::EndSeq;

	//write motion
	em << YAML::Key << "motion" << YAML::Value << YAML::BeginSeq;
	for (int i = 0; i < (int)frames.size(); i++)
	{
		em << YAML::BeginMap;
		em << YAML::Key << "frameName" << YAML::Value << frames[i]->name;
		em << YAML::Key << "duration"<< YAML::Value << frames[i]->duration;
		em << YAML::Key << "support" << YAML::Value << frames[i]->support;
		
		em << YAML::Key << "roll" << YAML::Value << frames[i]->roll;
		em << YAML::Key << "pitch" << YAML::Value << frames[i]->pitch;
		em << YAML::Key << "yaw" << YAML::Value << frames[i]->yaw;
		
		em << YAML::Key << "joints" << YAML::Value << YAML::BeginMap;
		for (int j = 0; j < (int)frames[i]->joints.size(); j++)
		{
			if (jointList[j].empty())
				continue;
			FrameJoint* joint = &(frames[i]->joints[j]);
			em << YAML::Key << jointList[j];
			em << YAML::Value << YAML::BeginMap;
			
			em << YAML::Key << "position" << YAML::Value << joint->position;
			em << YAML::Key << "effort" << YAML::Value << joint->effort;
			em << YAML::Key << "velocity" << YAML::Value << joint->velocity;
			
			em << YAML::Key << "p_gain" << YAML::Value << joint->pGain;
			em << YAML::Key << "i_gain" << YAML::Value << joint->iGain;
			em << YAML::Key << "d_gain" << YAML::Value << joint->dGain;
			em << YAML::Key << "limit"  << YAML::Value << joint->limit;
			em << YAML::Key << "gain_select" << YAML::Value << (int)joint->gainSelect;
			
			em << YAML::EndMap;
		}
		em << YAML::EndMap;

		em << YAML::EndMap;
	}
	em << YAML::EndSeq;

	em << YAML::EndMap;
}

bool Motion::isIdentical(Motion& motion_1, Motion& motion_2)
{
	return motion_1.dump() == motion_2.dump();
}

void Motion::detailedComparison(Motion& motion_1, Motion& motion_2)
{
	// Per motion info
	if(motion_1.motionName != motion_2.motionName)
		printf("Name is not equal: %s and %s\n", motion_1.motionName.c_str(), motion_2.motionName.c_str());
	
	if(motion_1.preState != motion_2.preState)
		printf("PreState is not equal: %s and %s\n", motion_1.preState.c_str(), motion_2.preState.c_str());
	
	if(motion_1.playState != motion_2.playState)
		printf("PlayState is not equal: %s and %s\n", motion_1.playState.c_str(), motion_2.playState.c_str());
	
	if(motion_1.postState != motion_2.postState)
		printf("PostState is not equal: %s and %s\n", motion_1.postState.c_str(), motion_2.postState.c_str());
	
	if(motion_1.pidEnabled != motion_2.pidEnabled)
		printf("PidEnabled is not equal: %d and %d\n", motion_1.pidEnabled, motion_2.pidEnabled);
	
	// Check each frame
	for(unsigned i = 0; i < motion_1.frames.size(); i++)
	{
		if(i >= motion_1.frames.size() || i >= motion_2.frames.size())
			return;
		
		KeyframePtr frame_1 = motion_1.frames.at(i);
		KeyframePtr frame_2 = motion_2.frames.at(i);
		
		printf("___Frame: %s\n", frame_1->name.c_str());
		
		if(frame_1->duration != frame_2->duration)
			printf("Duration is not equal: %f and %f\n", frame_1->duration, frame_2->duration);
		
		if(frame_1->roll != frame_2->roll)
			printf("Roll is not equal: %f and %f\n", frame_1->roll, frame_2->roll);
		
		if(frame_1->pitch != frame_2->pitch)
			printf("Pitch is not equal: %f and %f\n", frame_1->pitch, frame_2->pitch);
		
		if(frame_1->yaw != frame_2->yaw)
			printf("Yaw is not equal: %f and %f\n", frame_1->yaw, frame_2->yaw);
		
		if(frame_1->support != frame_2->support)
			printf("Support is not equal: %s and %s\n", frame_1->support.c_str(), frame_2->support.c_str());
		
		// Check each joint
		for(unsigned i = 0; i < motion_1.jointList.size(); i++)
		{
			printf("___Joint: %s\n", motion_1.jointList.at(i).c_str());
			
			int id1 = motionfile::Motion::nameToIndex(motion_1.jointList, motion_1.jointList.at(i));
			int id2 = motionfile::Motion::nameToIndex(motion_2.jointList, motion_1.jointList.at(i));
			
			if(id1 < 0 || id2 < 0)
			{
				printf("Error\n");
				continue;
			}
			
			// Position
			if(frame_1->joints[id1].position != frame_2->joints[id2].position)
				printf("Position is not equal: %f and %f\n", frame_1->joints[id1].position
															, frame_2->joints[id2].position);
			// Effort
			if(frame_1->joints[id1].effort != frame_2->joints[id2].effort)
				printf("Effort is not equal: %f and %f\n", frame_1->joints[id1].effort
															, frame_2->joints[id2].effort);
			// Velocity
			if(frame_1->joints[id1].velocity != frame_2->joints[id2].velocity)
				printf("Velocity is not equal: %f and %f\n", frame_1->joints[id1].velocity
															, frame_2->joints[id2].velocity);
				
			// P
			if(frame_1->joints[id1].pGain != frame_2->joints[id2].pGain)
				printf("P is not equal: %f and %f\n", frame_1->joints[id1].pGain
															, frame_2->joints[id2].pGain);
			// I
			if(frame_1->joints[id1].iGain != frame_2->joints[id2].iGain)
				printf("I is not equal: %f and %f\n", frame_1->joints[id1].iGain
															, frame_2->joints[id2].iGain);
			// D
			if(frame_1->joints[id1].dGain != frame_2->joints[id2].dGain)
				printf("D is not equal: %f and %f\n", frame_1->joints[id1].dGain
															, frame_2->joints[id2].dGain);
			// PID_SELECT
			if(frame_1->joints[id1].gainSelect != frame_2->joints[id2].gainSelect)
				printf("PID_SELECT is not equal: %d and %d\n", frame_1->joints[id1].gainSelect
															, frame_2->joints[id2].gainSelect);
		}
	}
}

