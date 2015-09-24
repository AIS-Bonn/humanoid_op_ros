//Motion module to play motions from trajectories
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/package.h>

#include <motion_player/PlayMotion.h>

#include "motionplayer.h"

namespace fs = boost::filesystem;
namespace motionplayer
{

void MappedMotion::resetPlayer()
{
	for (unsigned i = 0; i < player.size(); i++)
		player[i].reset();
}

void MappedMotion::initPlayer()
{
	player.clear();

	for (unsigned i = 0; i < jointList.size(); i++)
		player.push_back(kf_player::KeyframePlayer());

	double t_old = 0;
	for (unsigned i = 0; i < frames.size(); i++)
	{
		double t = frames[i]->duration + t_old;
		t_old = t;
		
		// Parse the leg support coefficients from the support field (expect support string format '%f %f')
		std::istringstream ss(frames[i]->support);
		double suppLeftLeg = 0.0, suppRightLeg = 0.0;
		ss >> suppLeftLeg >> suppRightLeg;
		if(ss.fail())
			suppLeftLeg = suppRightLeg = 0.0; // If not both coeffs can be parsed then both become zero

		for(unsigned j = 0; j < jointList.size(); j++)
		{
			double x = frames[i]->joints[j].position;
			double v = frames[i]->joints[j].velocity;
			double effort = frames[i]->joints[j].effort;
			player[j].addKeyframe(t, x, v, effort, suppLeftLeg, suppRightLeg);
		}
	}
}

MotionPlayer::MotionPlayer()
 : m_torqueAct("/robotcontrol/fade_torque")
 , m_isRelaxed(false)
 , m_isInitialized(false)
 , isPlaying(false)
{
	ros::NodeHandle nh("~");

	m_srv_play = nh.advertiseService("/motion_player/play", &MotionPlayer::handlePlay, this);
	m_srv_update = nh.advertiseService("/motion_player/update", &MotionPlayer::handleUpdateMotion, this);

	nh.param<std::string>("/robot_name", m_robot_name, std::string());
	nh.param<std::string>("/robot_type", m_robot_type, std::string());
	ROS_INFO("Robot name: '%s'", (m_robot_name.empty() ? "<none>" : m_robot_name.c_str()));
	ROS_INFO("Robot type: '%s'", (m_robot_type.empty() ? "<none>" : m_robot_type.c_str()));

	nh.param<std::string>("InitPoseMotion", m_init_pose_name, "init_pose");
	nh.param<std::string>("InitMotion", m_init_name, "init");
	nh.param<std::string>("GetupProneMotion", m_getup_prone_name, "getup_prone");
	nh.param<std::string>("GetupSupineMotion", m_getup_supine_name, "getup_supine");
}

bool MotionPlayer::init(robotcontrol::RobotModel* model)
{
	if(!robotcontrol::MotionModule::init(model))
		return false;

	m_model = model;
	m_dT = m_model->timerDuration();

	fs::path p((ros::package::getPath("launch") + "/motions"));

	if(m_robot_name.empty() || m_robot_type.empty() ||
	   !loadMotionFiles(p / ("/" + m_robot_type + "_" + m_robot_name)))
	{
		ROS_WARN("No robot specific motions found.");
	}

	if(m_robot_type.empty() ||
	   !loadMotionFiles(p / ("/" + m_robot_type)))
	{
		ROS_WARN("No robot type specific motions found.");
	}

	if(!loadMotionFiles(p / ("/default")))
	{
		ROS_ERROR("Default motion folder not found. Failed to init motion player.");
		return false;
	}

	m_state_relaxed = m_model->registerState("relaxed");
	m_state_setting_pose = m_model->registerState("setting_pose");
	m_state_init = m_model->registerState("init");
	m_state_falling = m_model->registerState("falling");
	m_state_lying_prone = m_model->registerState("lying_prone");
	m_state_lying_supine = m_model->registerState("lying_supine");

	initMotions();
	
	size_t nj = model->numJoints();
	m_cmdPositions.resize(nj);
	m_cmdVelocities.resize(nj);
	m_cmdAccelerations.resize(nj);
	m_cmdEffort.resize(nj);
	
	// Retrieve the URDF links that will be used for setting support coefficients
	m_leftFootLink  = m_model->urdf()->getLink("left_foot_plane_link");
	m_rightFootLink = m_model->urdf()->getLink("right_foot_plane_link");

	return true;
}

void MotionPlayer::initMotions()
{
	std::map<std::string, MappedMotion>::iterator motionIt;
	for (motionIt = m_motionNames.begin(); motionIt != m_motionNames.end(); ++motionIt)
	{
		MappedMotion& motion = motionIt->second;
		std::vector<std::string> jointList = motion.jointList;
		motion.motionToModel.resize(jointList.size());
		for (unsigned i = 0; i < jointList.size(); i++)
		{
			motion.motionToModel[i] = findIndex(jointList[i]);
		}

		motion.reqState = m_model->registerState(motion.preState);
		motion.transState = m_model->registerState(motion.playState);
		motion.resState = m_model->registerState(motion.postState);
		motion.initPlayer();
	}
}

int MotionPlayer::findIndex(std::string name)
{
	for (size_t i = 0; i < m_model->numJoints(); i++)
	{
		const boost::shared_ptr<robotcontrol::Joint>& joint = (*m_model)[i];

		if (joint->name == name)
			return i;
	}
	return 0;
}

bool MotionPlayer::isTriggered()
{
	// Retrieve the current robot model state
	robotcontrol::RobotModel::State state = m_model->state();

	// Handle the case that a motion is playing already
	if(isPlaying)
	{
		// Continue playing the motion if the state is still okay
		if(state == m_playingMotion->transState)
			return true;

		// Special exception for the init_pose motion. For security reasons this motion is required to finish.
		if(m_playingMotion->motionName == m_init_pose_name)
		{
			ROS_WARN_THROTTLE(0.2, "Someone is writing dumb states while I'm playing the init pose motion. Continuing anyway.");
			m_model->setState(m_playingMotion->transState); // Force back the init pose play state
			return true;
		}

		// Abort current motion because of a changed state
		ROS_WARN("Someone changed the state to '%s' while I was still playing motion '%s', aborting motion...", m_model->currentStateLabel().c_str(), m_playingMotion->motionName.c_str());
		m_playingMotion = 0;
		stopPlaying();
		return false;
	}

	// Reset the relaxed and initialized flags depending on whether we're in a state associated with being relaxed
	if(state == m_state_relaxed || state == m_state_setting_pose)
		m_isInitialized = false;
	else
		m_isRelaxed = false;
	if(state == m_state_falling)
		m_isInitialized = false;

	// Plays init_pose motion every time the robot gets set to relaxed state after the first init
	// IMPORTANT: Make sure that the init_pose motion takes less time than a complete fade-in
	if(state == m_state_relaxed && !m_isRelaxed)
	{
		if(tryToPlay(m_init_pose_name, false))
		{
			m_isRelaxed = true;
			return true;
		}
	}

	// Plays the init motion every time after the robot fades in
	if(state == m_state_init && !m_isInitialized)
	{
		if(tryToPlay(m_init_name, false))
		{
			m_isInitialized = true;
			return true;
		}
	}

	// Plays the appropriate getup motions if the robot is lying on the ground
	if(state == m_state_lying_prone || state == m_state_lying_supine)
	{
		bool success = tryToPlay((state == m_state_lying_prone ? m_getup_prone_name : m_getup_supine_name), false);
		m_isInitialized = false;
		if(success) return true;
	}

	// Play a motion if it is triggered via a trigger state (use a robot state of "<current_state>|play_<motion>" to trigger motion <motion>)
	const std::string token = "|play_";
	std::string curStateLabel = m_model->currentStateLabel();
	size_t ind = curStateLabel.find(token);
	if(ind != std::string::npos) // We have a current robot state of "*|play_*"...
	{
		// Parse the current state and required motion to play
		std::string realStateLabel = curStateLabel.substr(0, ind);
		std::string motionName = curStateLabel.substr(ind + token.size());

		// Issue a warning if a motion is currently playing
		if(isPlaying)
		{
			if(m_playingMotion->playState.find(token) != std::string::npos)
				ROS_WARN_THROTTLE(1.0, "The currently playing motion '%s' has a playing state of '%s'. This should be fixed to not look like a motion trigger state!", m_playingMotion->motionName.c_str(), m_playingMotion->playState.c_str());
			ROS_WARN_THROTTLE(1.0, "Someone requested a motion using the '%s' trigger state, but the '%s' motion is currently playing!", curStateLabel.c_str(), m_playingMotion->motionName.c_str());
		}

		// Truncate the motion trigger component of the state variable
		if(!realStateLabel.empty())
		{
			robotcontrol::RobotModel::State newState = m_model->registerState(realStateLabel);
			m_model->setState(newState);
		}

		// Play the required motion
		if(!isPlaying && !motionName.empty())
		{
			if(tryToPlay(motionName, false))
				return true;
		}
	}

	// Return that the motion player is not triggered
	return false;
}

bool MotionPlayer::handlePlay(motion_player::PlayMotionRequest& req, motion_player::PlayMotionResponse& res)
{
	std::string motionName;
	
	if(req.type == 0) // arbitrary motion
		return tryToPlay(req.name, true);
	else if(req.type == 1) // motion from trajectory_editior
		return tryToPlay("trajectory_editor_motion", false);
	else if(req.type == 2) // frame from trajectory_editor
		return tryToPlay("trajectory_editor_frame", false);
	else
	{
		ROS_ERROR("Undefined type '%d' of play request (valid range 0-2)", (int)req.type);
		return false;
	}
}

bool MotionPlayer::tryToPlay(const std::string& motion, bool checkState)
{
	if(isPlaying)
	{
		ROS_ERROR("Someone requested motion '%s', but I am still playing '%s'!", motion.c_str(), m_playingMotion->motionName.c_str());
		return false;
	}

	if(m_motionNames.find(motion) == m_motionNames.end())
	{
		ROS_ERROR("Could not find motion '%s'!", motion.c_str());
		return false;
	}

	if(checkState && m_model->state() != m_motionNames[motion].reqState)
	{
		ROS_ERROR("Motion '%s' expects state '%s', but current state is '%s'!", motion.c_str(), m_model->stateLabel(m_motionNames[motion].reqState).c_str(), m_model->currentStateLabel().c_str());
		return false;
	}
	
	ROS_INFO("Playing motion '%s'", motion.c_str());
	play(motion);
	return true;
}

void MotionPlayer::play(const std::string& motion)
{
	if(isPlaying)
	{
		ROS_ERROR("Tried to play motion '%s', but '%s' is still playing!", motion.c_str(), m_playingMotion->motionName.c_str());
		return;
	}

	m_playingMotion = &m_motionNames[motion];

	for(unsigned i = 0; i < m_playingMotion->player.size(); i++)
	{
		kf_player::KeyframePlayer* currentPlayer = &m_playingMotion->player[i];
		int index = m_playingMotion->motionToModel[i];
		robotcontrol::Joint::Ptr joint = (*m_model)[index];
		currentPlayer->keyframes[0].x = joint->cmd.pos;
		m_playingMotion->player[i].calculateCommands();
	}

	startPlaying();
	m_playingMotion->resetPlayer();
	m_model->setState(m_playingMotion->transState);
}

void MotionPlayer::finished()
{
	m_model->setState(m_playingMotion->resState);
	m_playingMotion = 0;
	stopPlaying();
}

bool MotionPlayer::loadMotionFiles(const fs::path& dir)
{
	if (!fs::exists(dir) || !fs::is_directory(dir))
		return false;

	fs::directory_iterator end;
	fs::directory_iterator file(dir);

	std::string fName;
	std::string suffix = ".yaml";

	MappedMotion newMotion;
	int count = 0, bad = 0;

	ROS_INFO("Loaded motions in folder '%s':", dir.c_str());
	for (; file != end; ++file)
	{
		if(fs::is_directory(*file))
			continue;

		fName = file->path().leaf().string();

		if(!boost::regex_search(fName, boost::regex("\\.yaml$")))
			continue;

		fName = fName.substr(0, fName.size() - suffix.size());

		if(newMotion.load(file->path().string()))
		{
			if (m_motionNames.count(fName))
			{
				ROS_INFO("- %s [SKIPPED]", fName.c_str());
				continue;
			}

			std::pair<std::string, MappedMotion> entry(fName, newMotion);
			m_motionNames.insert(entry);
			ROS_INFO("- %s", fName.c_str());
			count++;
		}
		else
		{
			ROS_WARN("- %s [INVALID]", fName.c_str());
			bad++;
		}
	}
	if(bad > 0)
		ROS_INFO("Encountered %d bad motion files!", bad);
	ROS_INFO("Successfully loaded %d motions!", count);
	
	
	// Create fictious motions for playing fram/motion from trajectory_editor_2
	MappedMotion nullMotion;
	
	std::pair<std::string, MappedMotion> entry1("trajectory_editor_motion", nullMotion);
	m_motionNames.insert(entry1);
	
	std::pair<std::string, MappedMotion> entry2("trajectory_editor_frame", nullMotion);
	m_motionNames.insert(entry2);

	return true;
}

bool MotionPlayer::handleUpdateMotion(motion_player::StreamMotionRequest& req, motion_player::StreamMotionResponse& res)
{
	if(isPlaying)
	{
		ROS_ERROR("Tried to update motion I still play the other motion!");
		return false;
	}
	
	std::string motionName;
	
	if(req.type == 0) // arbitrary motion
		motionName = req.name;
	else if(req.type == 1) // motion from trajectory_editor
		motionName = "trajectory_editor_motion";
	else if(req.type == 2) // frame from trajectory_editor
		motionName = "trajectory_editor_frame";
	else
	{
		ROS_ERROR("Undefined type of update request (have to be 0-2)");
		return false;
	}
	
	if(m_motionNames.find(motionName) == m_motionNames.end())
	{
		ROS_ERROR("Tried to update motion '%s', but it was not found!", motionName.c_str());
		return false;
	}

	MappedMotion nMotion;

	if(!nMotion.parse(req.motion))
	{
		ROS_ERROR("Failed to parse motion '%s' for updating!", motionName.c_str());
		return false;
	}

	std::vector<std::string> jointList = nMotion.jointList;
	nMotion.motionToModel.resize(jointList.size());
	for(unsigned i = 0; i < jointList.size(); i++)
		nMotion.motionToModel[i] = findIndex(jointList[i]);

	nMotion.reqState = m_model->registerState(nMotion.preState);
	nMotion.transState = m_model->registerState(nMotion.playState);
	nMotion.resState = m_model->registerState(nMotion.postState);
	nMotion.initPlayer();

	m_motionNames[motionName] = nMotion;

	ROS_INFO("Successfully updated motion '%s'", motionName.c_str());
	return true;
}

void MotionPlayer::step()
{
	// Retrieve the system iteration time
	m_dT = m_model->timerDuration();
	
	// Don't do anything if we're not playing anything
	if(!isPlaying) return;

	// Update the pose to command to the robot
	kf_player::Keyframe cmdFrame;
	for (unsigned i = 0; i < m_playingMotion->player.size(); i++)
	{
		if(m_playingMotion->player[i].atEnd())
		{
			finished();
			return;
		}
		cmdFrame = m_playingMotion->player[i].step(m_dT);
		int index = m_playingMotion->motionToModel[i];
		m_cmdPositions[index] = cmdFrame.x;
		m_cmdVelocities[index] = cmdFrame.v;
		m_cmdAccelerations[index] = cmdFrame.a;
		m_cmdEffort[index] = cmdFrame.effort;
	}

	// Set the required support coefficients
	m_model->resetSupport();
	m_model->setSupportCoefficient(m_leftFootLink, cmdFrame.suppLeftLeg);   // Note: Each keyframe player should return the same support coefficient information,
	m_model->setSupportCoefficient(m_rightFootLink, cmdFrame.suppRightLeg); //       so we just take the last one and hope for the best...
	
	// Send off the joint position and effort commands
	writeCommands();
}

void MotionPlayer::writeCommands()
{
	size_t nj = m_cmdPositions.size();
	for(size_t i = 0; i < nj; i++)
	{
		robotcontrol::Joint::Ptr joint = (*m_model)[i];
		joint->cmd.setFromPosVelAcc(m_cmdPositions[i], m_cmdVelocities[i], m_cmdAccelerations[i]);
		joint->cmd.raw = false;
		joint->cmd.effort = m_cmdEffort[i];
	}
}

}

PLUGINLIB_EXPORT_CLASS(motionplayer::MotionPlayer, robotcontrol::MotionModule)