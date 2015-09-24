//Library to manage motion files
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include "motion_file/motionfile.h"

#include <fstream>

const std::string DEFAULT_KEYFRAME_SUPPORT  = "";
const double      DEFAULT_VELOCITY = 0;

using namespace motionfile;

Motion::Motion()
{
}

FrameJoint::FrameJoint()
 : position(0)
 , velocity(0)
 , effort(1)
{}


bool Motion::load(std::string name)
{
	//Delete keyframes in case the motion loads more than once
	frames.clear();
	jointList.clear();

	YAML::Node node;
	try
	{
		node = YAML::LoadFile(name);
		nodeToMotion(node);
	}
	catch (YAML::Exception& e)
	{
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

	YAML::Node keyframes = node["motion"];
	for (YAML::const_iterator it=keyframes.begin(); it!=keyframes.end(); ++it)
	{
		KeyframePtr kf(new Keyframe);

		//per keyframe information
		kf->duration = (*it)["duration"].as<double>();
		if ((*it)["support"])
			kf->support = (*it)["support"].as< std::string>();
		else
			kf->support = DEFAULT_KEYFRAME_SUPPORT;


		YAML::Node joints = (*it)["joints"];
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

	em << YAML::EndMap;

	//write motion
	em << YAML::Key << "motion" << YAML::Value << YAML::BeginSeq;
	for (int i = 0; i < (int)frames.size(); i++)
	{
		em << YAML::BeginMap;
		em << YAML::Key << "duration"<< YAML::Value << frames[i]->duration;
		em << YAML::Key << "support" << YAML::Value << frames[i]->support;
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
			em << YAML::EndMap;
		}
		em << YAML::EndMap;

		em << YAML::EndMap;
	}
	em << YAML::EndSeq;

	em << YAML::EndMap;
}

