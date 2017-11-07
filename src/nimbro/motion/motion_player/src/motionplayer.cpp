//Motion module to play motions from trajectories
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>, Hafez Farazi <farazi@ais.uni-bonn.de>

#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/package.h>

#include <motion_player/PlayMotion.h>
#include <motion_player/MotionPlayerState.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <ctime>

#include "motionplayer.h"

#define LOWMAGICSTR "_LOW_"
#define HIGHMAGICSTR "_HIGH_"
#define DISABLEMAGSTR "##"

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
		if (ss.fail())
			suppLeftLeg = suppRightLeg = 0.0; // If not both coeffs can be parsed then both become zero

		for (unsigned j = 0; j < jointList.size(); j++)
		{
			double x = frames[i]->joints[j].position;
			double v = frames[i]->joints[j].velocity;
			double effort = frames[i]->joints[j].effort;
			player[j].addKeyframe(t, x, v, effort, suppLeftLeg, suppRightLeg,
					frames[i]->joints[j].pGain, frames[i]->joints[j].iGain,
					frames[i]->joints[j].dGain, frames[i]->joints[j].limit, frames[i]->joints[j].gainSelect,
					frames[i]->roll, frames[i]->pitch, frames[i]->yaw);
		}
	}
}

const std::string MotionPlayer::RESOURCE_PATH = "motion_player/";
const std::string MotionPlayer::CONFIG_PARAM_PATH = "/motion_player/";
const std::string MotionPlayer::m_fallingStatePrefix = "falling|play_";

MotionPlayer::MotionPlayer() : m_nh("~"), m_torqueAct(m_nh, "fade_torque"), m_isRelaxed(false), m_isInitialized(false), isPlaying(false), PM(PM_COUNT, RESOURCE_PATH)
{
	ros::NodeHandle nhs;

	m_srv_play = nhs.advertiseService(RESOURCE_PATH + "play",
			&MotionPlayer::handlePlay, this);
	m_srv_update = nhs.advertiseService(RESOURCE_PATH + "update",
			&MotionPlayer::handleUpdateMotion, this);
	m_srv_reload = nhs.advertiseService(RESOURCE_PATH + "reload",
			&MotionPlayer::handleReloadMotion, this);
	m_srv_dump_motions = nhs.advertiseService(RESOURCE_PATH + "dump_motions",
			&MotionPlayer::handleDumpMotions, this);

	nhs.param<std::string>("robot_name", m_robot_name, std::string());
	nhs.param<std::string>("robot_type", m_robot_type, std::string());
	if(m_robot_name.empty())
		ROS_ERROR("Robot name is empty or unconfigured!");
	if(m_robot_type.empty())
		ROS_ERROR("Robot type is empty or unconfigured!");
	ROS_INFO("Robot name: '%s'",
			(m_robot_name.empty() ? "<none>" : m_robot_name.c_str()));
	ROS_INFO("Robot type: '%s'",
			(m_robot_type.empty() ? "<none>" : m_robot_type.c_str()));

	m_nh.param<std::string>("InitPoseMotion", m_init_pose_name, "init_pose");
	m_nh.param<std::string>("InitMotion", m_init_name, "init");
	m_nh.param<std::string>("GetupProneMotion", m_getup_prone_name,
			"getup_prone");
	m_nh.param<std::string>("GetupSupineMotion", m_getup_supine_name,
			"getup_supine");

	// State publisher
	statePublisher = nhs.advertise<motion_player::MotionPlayerState>(
			RESOURCE_PATH + "state", 1);
	timer = m_nh.createTimer(ros::Duration(0.2), &MotionPlayer::publishState,
			this);
	PM.setName(PM_FRAME_INDEX, "FrameIndex");
	PM.setName(PM_ROLL, "Roll");
	PM.setName(PM_PITCH, "Pitch");
	PM.setName(PM_YAW, "Yaw");
	PM.setName(PM_ROLLD, "RollD");
	PM.setName(PM_PITCHD, "PitchD");
	PM.setName(PM_YAWD, "YawD");
	PM.setName(PM_DES_ROLL, "DesiredRoll");
	PM.setName(PM_DES_PITCH, "DesiredPitch");
	PM.setName(PM_DES_YAW, "DesiredYaw");
	if (!PM.checkNames())
	{
		ROS_WARN("Check checkNames function for plotM!");
	}
}

void MotionPlayer::publishState(const ros::TimerEvent& event)
{
	motion_player::MotionPlayerState stateMsg;

	stateMsg.isPlaying = isPlaying;

	if (isPlaying)
	{
		stateMsg.motionName = m_playingMotion.motionName;
		stateMsg.motionDuration = m_playingMotion.player.at(0).totalTime();
		stateMsg.currentTime = m_playingMotion.player.at(0).currentTime();
	}
	else
		stateMsg.motionName = "";

	statePublisher.publish(stateMsg);
}

bool MotionPlayer::init(robotcontrol::RobotModel* model)
{
	if (!robotcontrol::MotionModule::init(model))
		return false;

	m_model = model;
	m_dT = m_model->timerDuration();

	fs::path p((ros::package::getPath("launch") + "/motions"));

	if (m_robot_name.empty() || m_robot_type.empty()
			|| !loadMotionFiles(p / ("/" + m_robot_type + "_" + m_robot_name)))
	{
		ROS_INFO("No robot specific motions found.");
	}

	if (m_robot_type.empty() || !loadMotionFiles(p / ("/" + m_robot_type)))
	{
		ROS_INFO("No robot type specific motions found.");
	}

	if (!loadMotionFiles(p / ("/default")))
	{
		ROS_ERROR(
				"Default motion folder not found. Failed to init motion player.");
		return false;
	}

	m_state_relaxed = m_model->registerState("relaxed");
	m_state_setting_pose = m_model->registerState("setting_pose");
	m_state_init = m_model->registerState("init");
	m_state_standing = m_model->registerState("standing");
	m_state_sitting = m_model->registerState("sitting");
	m_state_falling = m_model->registerState("falling");
	m_state_lying_prone = m_model->registerState("lying_prone");
	m_state_lying_supine = m_model->registerState("lying_supine");

	initMotions();
	initRuleParameters();

	size_t nj = model->numJoints();
	m_cmdPositions.resize(nj);
	m_cmdVelocities.resize(nj);
	m_cmdAccelerations.resize(nj);
	m_cmdEffort.resize(nj);
	m_preGains.resize(nj, kf_player::nonE); //initialize it with kf_player::nonE
	m_preAngle = Eigen::Vector3f(0.0, 0.0, 0.0);
	m_integralGains.resize(nj, 0.0);

	// Retrieve the URDF links that will be used for setting support coefficients
	m_leftFootLink = m_model->urdf()->getLink("left_foot_plane_link");
	m_rightFootLink = m_model->urdf()->getLink("right_foot_plane_link");

	return true;
}

// Register rule parameter for each motion
void MotionPlayer::initRuleParameters()
{
	m_limit_inverse_space.reset(new config_server::Parameter<bool>(CONFIG_PARAM_PATH + "rule/limit_inverse/enable", 0));
	m_epsilon.reset(new config_server::Parameter<float>(CONFIG_PARAM_PATH + "rule/limit_inverse/epsilon", 0, 0.005, 1, 0.005));
	
	std::map<std::string, MappedMotion>::iterator motionIt;
	for (motionIt = m_motionNames.begin(); motionIt != m_motionNames.end(); ++motionIt)
	{
		std::string rootName = CONFIG_PARAM_PATH + "rule/" + motionIt->first + "/";
		
		std::vector<FloatParamPtr> parameters;
		
		if(motionIt->first == "trajectory_editor_motion") // Always 2 params for trajectory editor motions
		{
			FloatParamPtr param1;
			param1.reset(new config_server::Parameter<float>(rootName + "Sagittal", -1, 0.01, 1, 0));
			parameters.push_back(param1);
			
			FloatParamPtr param2;
			param2.reset(new config_server::Parameter<float>(rootName + "Lateral", -1, 0.01, 1, 0));
			parameters.push_back(param2);
		}
		else
		{
			for(size_t i = 0; i < motionIt->second.rules.size(); i++)
			{
				FloatParamPtr ruleParam;
				ruleParam.reset(new config_server::Parameter<float>(rootName + motionIt->second.rules[i].name, -1, 0.01, 1, 0));
				parameters.push_back(ruleParam);
			}
		}
		
		std::pair<std::string, std::vector<FloatParamPtr> > entry(motionIt->first, parameters);
		m_ruleParameters.insert(entry);
	}
}

void MotionPlayer::initMotions()
{
	std::map<std::string, MappedMotion>::iterator motionIt;
	for (motionIt = m_motionNames.begin(); motionIt != m_motionNames.end();
			++motionIt)
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
	std::string curStateLabel = m_model->currentStateLabel();

	// Handle the case that a motion is playing already
	if (isPlaying)
	{
		// Continue playing the motion if the state is still okay
		if (state == m_playingMotion.transState)
			return true;

		// Special exception for the init_pose motion. For security reasons this motion is required to finish.
		if (m_playingMotion.motionName == m_init_pose_name)
		{
			ROS_WARN_THROTTLE(0.2,
					"Someone is writing dumb states while I'm playing the init pose motion. Continuing anyway.");
			m_model->setState(m_playingMotion.transState); // Force back the init pose play state
			return true;
		}

		// Abort current motion because of a changed state
		ROS_WARN(
				"Someone changed the state to '%s' while I was still playing motion '%s', aborting motion...",
				m_model->currentStateLabel().c_str(),
				m_playingMotion.motionName.c_str());

		stopPlaying();
	}

	// Reset the relaxed and initialized flags depending on whether we're in a state associated with being relaxed
	if (state == m_state_relaxed || state == m_state_setting_pose)
		m_isInitialized = false;
	else
		m_isRelaxed = false;
	if (state == m_state_falling
			|| curStateLabel.compare(0, m_fallingStatePrefix.size(),
					m_fallingStatePrefix) == 0)
		m_isInitialized = false;

	// Plays init_pose motion every time the robot gets set to relaxed state after the first init
	// IMPORTANT: Make sure that the init_pose motion takes less time than a complete fade-in
	if (state == m_state_relaxed && !m_isRelaxed)
	{
		if (tryToPlay(m_init_pose_name, false))
		{
			m_isRelaxed = true;
			return true;
		}
	}

	// Plays the init motion every time after the robot fades in
	if (state == m_state_init && !m_isInitialized)
	{
		if (tryToPlay(m_init_name, false))
		{
			m_isInitialized = true;
			return true;
		}
	}

	// Plays the appropriate getup motions if the robot is lying on the ground
	if (state == m_state_lying_prone || state == m_state_lying_supine)
	{
		bool success = tryToPlay(
				(state == m_state_lying_prone ?
						m_getup_prone_name : m_getup_supine_name), false);
		m_isInitialized = false;
		if (success)
			return true;
	}

	// Play a motion if it is triggered via a trigger state (use a robot state of "<current_state>|play_<motion>" to trigger motion <motion>)
	const std::string token = "|play_";
	size_t ind = curStateLabel.find(token);
	if (ind != std::string::npos) // We have a current robot state of "*|play_*"...
	{
		// Parse the current state and required motion to play
		std::string realStateLabel = curStateLabel.substr(0, ind);
		std::string motionName = curStateLabel.substr(ind + token.size());

		// Issue a warning if a motion is currently playing
		if (isPlaying)
		{
			if (m_playingMotion.playState.find(token) != std::string::npos)
				ROS_WARN_THROTTLE(1.0,
						"The currently playing motion '%s' has a playing state of '%s'. This should be fixed to not look like a motion trigger state!",
						m_playingMotion.motionName.c_str(),
						m_playingMotion.playState.c_str());
			ROS_WARN_THROTTLE(1.0,
					"Someone requested a motion using the '%s' trigger state, but the '%s' motion is currently playing!",
					curStateLabel.c_str(), m_playingMotion.motionName.c_str());
		}

		// Truncate the motion trigger component of the state variable
		if (!realStateLabel.empty())
		{
			robotcontrol::RobotModel::State newState = m_model->registerState(
					realStateLabel);
			m_model->setState(newState);
		}

		// Play the required motion
		if (!isPlaying && !motionName.empty())
		{
			if (tryToPlay(motionName, false))
				return true;
		}
	}

	// Return that the motion player is not triggered
	return false;
}

// Save all motions (which are loaded now) to /tmp/motions_dump/<timestamp> folder
bool MotionPlayer::handleDumpMotions(motion_player::DumpMotionsRequest& req,
		motion_player::DumpMotionsResponse& res)
{
	// Get current timestamp
	std::time_t now = std::time(NULL);
	std::tm * ptm = std::localtime(&now);
	char timestamp[32];
	std::strftime(timestamp, 32, "%a, %d.%m.%Y %H:%M:%S", ptm);

	// Make folders
	std::string full_path("/tmp/motions_dump/" + std::string(timestamp));

	mkdir("/tmp/motions_dump", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	mkdir(full_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	// Save all motions there
	typedef std::map<std::string, MappedMotion>::iterator it_type;
	for (it_type iterator = m_motionNames.begin();
			iterator != m_motionNames.end(); iterator++)
		iterator->second.save(
				full_path + "/" + iterator->second.motionName + ".yaml");

	return true;
}

bool MotionPlayer::handlePlay(motion_player::PlayMotionRequest& req,
		motion_player::PlayMotionResponse& res)
{
	std::string motionName;

	if (req.type == 0) // arbitrary motion
		return tryToPlay(req.name, true);
	else if (req.type == 1) // motion from trajectory_editior
		return tryToPlay("trajectory_editor_motion", false);
	else if (req.type == 2) // frame from trajectory_editor
		return tryToPlay("trajectory_editor_frame", false);
	else
	{
		ROS_ERROR("Undefined type '%d' of play request (valid range 0-2)",
				(int )req.type);
		return false;
	}
}

bool MotionPlayer::tryToPlay(const std::string& motion, bool checkState)
{
	if (isPlaying)
	{
		ROS_ERROR("Someone requested motion '%s', but I am still playing '%s'!",
				motion.c_str(), m_playingMotion.motionName.c_str());
		return false;
	}

	if (m_motionNames.find(motion) == m_motionNames.end())
	{
		ROS_ERROR("Could not find motion '%s'!", motion.c_str());
		return false;
	}

	if (checkState && m_model->state() != m_motionNames[motion].reqState)
	{
		ROS_ERROR("Motion '%s' expects state '%s', but current state is '%s'!",
				motion.c_str(),
				m_model->stateLabel(m_motionNames[motion].reqState).c_str(),
				m_model->currentStateLabel().c_str());
		return false;
	}
	
	ROS_INFO("Playing motion '%s'", motion.c_str());
	
	play(motion);
	return true;
}

void MotionPlayer::applyRule(motionplayer::MappedMotion &motion, int rule_id, const float delta)
{
	if(rule_id < 0 || rule_id >= (int)motion.rules.size())
		return;
	
	// Apply rule
	motionfile::Rule rule = motion.rules[rule_id];
	
	for(size_t i = 0; i < rule.parts.size(); i++)
	{
		int index = motion.findFrame(rule.parts[i].frameName);
		m_ruleApplier.applyRulePart(motion.frames[index], motion.jointList, rule.parts[i], delta, true, false, 0);
	}
	
	// Update motion
	std::vector<std::string> jointList = motion.jointList;
	motion.motionToModel.resize(jointList.size());
	for (unsigned i = 0; i < jointList.size(); i++)
		motion.motionToModel[i] = findIndex(jointList[i]);

	motion.reqState = m_model->registerState(motion.preState);
	motion.transState = m_model->registerState(motion.playState);
	motion.resState = m_model->registerState(motion.postState);
	motion.initPlayer();
}

void MotionPlayer::play(const std::string& motion)
{
	if (isPlaying)
	{
		ROS_ERROR("Tried to play motion '%s', but '%s' is still playing!",
				motion.c_str(), m_playingMotion.motionName.c_str());
		return;
	}

//	if(m_motionNames[motion].player.size() >0)
//	{
//		ROS_INFO(" BEFORE Assign = %2f,   %d",m_motionNames[motion].player[0].keyframes[0].v,(int)m_motionNames[motion].player.size());
//
//	}
	m_playingMotion = m_motionNames[motion];
	
//	if(m_playingMotion.player.size() >0)
//	{
//		m_motionNames[motion].player[0].keyframes[0].v=53.23;
//		ROS_INFO(" AGAIN  = %2f,   %d",m_motionNames[motion].player[0].keyframes[0].v,(int)m_motionNames[motion].player.size());
//
//		ROS_INFO(" AFTER Assign = %2f,   %d",m_playingMotion.player[0].keyframes[0].v,(int)m_playingMotion.player.size());
//	}
//	ROS_INFO("before = %d", (int )m_playingMotion.frames.size());
	for (unsigned i = 0; i < m_playingMotion.frames.size(); i++)
	{

		size_t findPos = m_playingMotion.frames[i]->name.find(LOWMAGICSTR);

		if (m_playingMotion.frames[i]->name.find(DISABLEMAGSTR) == 0)
		{

			//ROS_INFO("* %s", m_playingMotion.frames[i]->name.c_str());
			m_playingMotion.frames.erase(m_playingMotion.frames.begin() + i);
			for (unsigned j = 0; j < m_playingMotion.player.size(); j++)
			{
				m_playingMotion.player[j].keyframes.erase(
						m_playingMotion.player[j].keyframes.begin() + i);
			}
			i--;
			continue;
		}
		else if (findPos != std::string::npos)
		{
			//ROS_INFO("** %s", m_playingMotion.frames[i]->name.c_str());
			//Note: In case of strange differences between blending keyframes(like PID gainSelect), the first one will be used.
			std::stringstream tmpSS(
					m_playingMotion.frames[i]->name.substr(
							findPos + strlen(LOWMAGICSTR),
							m_playingMotion.frames[i]->name.size()));

			bool blendIt = false;
			double lowVolt = -1;
			double highVolt = -1;
			double currentVolt = (*m_model).getVoltage();
			if (tmpSS >> lowVolt)
			{
				ROS_INFO("Frame[%s] has %s label with voltage = %.2f",
						m_playingMotion.frames[i]->name.c_str(), LOWMAGICSTR,
						lowVolt);
				if (i + 1 < m_playingMotion.frames.size()
						&& m_playingMotion.frames[i + 1]->name.find(
						DISABLEMAGSTR) != 0)
				{
					findPos = m_playingMotion.frames[i + 1]->name.find(
					HIGHMAGICSTR);
					if (findPos != std::string::npos)
					{
						std::stringstream tmpSS(
								m_playingMotion.frames[i + 1]->name.substr(
										findPos + strlen(HIGHMAGICSTR),
										m_playingMotion.frames[i + 1]->name.size()));
						if (tmpSS >> highVolt)
						{
							ROS_INFO(
									"Frame[%s] has %s label with voltage = %.2f",
									m_playingMotion.frames[i + 1]->name.c_str(),
									HIGHMAGICSTR, highVolt);
							blendIt = (lowVolt <= highVolt);
						}
					}
				}
			}

			if (blendIt)
			{
				int delInx = i + 1;
				if (currentVolt <= lowVolt)
				{
					delInx = i + 1;
				}
				else if (currentVolt >= highVolt)
				{
					delInx = i;
				}
				else
				{
					delInx = i + 1;
					//----------Bgn Interpolation----------
					Interpolator curInterpolate(lowVolt, highVolt, currentVolt);
					m_playingMotion.frames[i]->duration =
							curInterpolate.Interpolate(
									m_playingMotion.frames[i]->duration,
									m_playingMotion.frames[i + 1]->duration);
					m_playingMotion.frames[i]->roll =
							curInterpolate.Interpolate(
									m_playingMotion.frames[i]->roll,
									m_playingMotion.frames[i + 1]->roll);
					m_playingMotion.frames[i]->pitch =
							curInterpolate.Interpolate(
									m_playingMotion.frames[i]->pitch,
									m_playingMotion.frames[i + 1]->pitch);
					m_playingMotion.frames[i]->yaw = curInterpolate.Interpolate(
							m_playingMotion.frames[i]->yaw,
							m_playingMotion.frames[i + 1]->yaw);
					//Interpolate m_playingMotion.frames[i]->support
					{
						// Parse the leg support coefficients from the support field (expect support string format '%f %f')
						std::istringstream ssLow(
								m_playingMotion.frames[i]->support);
						double suppLeftLegLow = 0.0, suppRightLegLow = 0.0;
						ssLow >> suppLeftLegLow >> suppRightLegLow;
						if (ssLow.fail())
							suppLeftLegLow = suppRightLegLow = 0.0; // If not both coeffs can be parsed then both become zero

						std::istringstream ssHigh(
								m_playingMotion.frames[i + 1]->support);
						double suppLeftLegHigh = 0.0, suppRightLegHigh = 0.0;
						ssHigh >> suppLeftLegHigh >> suppRightLegHigh;
						if (ssHigh.fail())
							suppLeftLegHigh = suppRightLegHigh = 0.0; // If not both coeffs can be parsed then both become zero

						std::stringstream tmpSuportCoefSS("");
						tmpSuportCoefSS
								<< curInterpolate.Interpolate(suppLeftLegLow,
										suppLeftLegHigh) << " "
								<< curInterpolate.Interpolate(suppRightLegLow,
										suppRightLegHigh);
						m_playingMotion.frames[i]->support =
								tmpSuportCoefSS.str();
					}

					for (unsigned j = 0; j < m_playingMotion.player.size(); j++)
					{
						m_playingMotion.player[j].keyframes[i].t =
								curInterpolate.Interpolate(
										m_playingMotion.player[j].keyframes[i].t,
										m_playingMotion.player[j].keyframes[i
												+ 1].t);
						m_playingMotion.player[j].keyframes[i].x =
								curInterpolate.Interpolate(
										m_playingMotion.player[j].keyframes[i].x,
										m_playingMotion.player[j].keyframes[i
												+ 1].x);
						m_playingMotion.player[j].keyframes[i].v =
								curInterpolate.Interpolate(
										m_playingMotion.player[j].keyframes[i].v,
										m_playingMotion.player[j].keyframes[i
												+ 1].v);
						m_playingMotion.player[j].keyframes[i].a =
								curInterpolate.Interpolate(
										m_playingMotion.player[j].keyframes[i].a,
										m_playingMotion.player[j].keyframes[i
												+ 1].a);
						m_playingMotion.player[j].keyframes[i].pGain =
								curInterpolate.Interpolate(
										m_playingMotion.player[j].keyframes[i].pGain,
										m_playingMotion.player[j].keyframes[i
												+ 1].pGain);
						m_playingMotion.player[j].keyframes[i].iGain =
								curInterpolate.Interpolate(
										m_playingMotion.player[j].keyframes[i].iGain,
										m_playingMotion.player[j].keyframes[i
												+ 1].iGain);
						m_playingMotion.player[j].keyframes[i].dGain =
								curInterpolate.Interpolate(
										m_playingMotion.player[j].keyframes[i].dGain,
										m_playingMotion.player[j].keyframes[i
												+ 1].dGain);
						m_playingMotion.player[j].keyframes[i].roll =
								curInterpolate.Interpolate(
										m_playingMotion.player[j].keyframes[i].roll,
										m_playingMotion.player[j].keyframes[i
												+ 1].roll);
						m_playingMotion.player[j].keyframes[i].pitch =
								curInterpolate.Interpolate(
										m_playingMotion.player[j].keyframes[i].pitch,
										m_playingMotion.player[j].keyframes[i
												+ 1].pitch);
						m_playingMotion.player[j].keyframes[i].yaw =
								curInterpolate.Interpolate(
										m_playingMotion.player[j].keyframes[i].yaw,
										m_playingMotion.player[j].keyframes[i
												+ 1].yaw);
						m_playingMotion.player[j].keyframes[i].suppLeftLeg =
								curInterpolate.Interpolate(
										m_playingMotion.player[j].keyframes[i].suppLeftLeg,
										m_playingMotion.player[j].keyframes[i
												+ 1].suppLeftLeg);
						m_playingMotion.player[j].keyframes[i].suppRightLeg =
								curInterpolate.Interpolate(
										m_playingMotion.player[j].keyframes[i].suppRightLeg,
										m_playingMotion.player[j].keyframes[i
												+ 1].suppRightLeg);
						m_playingMotion.player[j].keyframes[i].effort =
								curInterpolate.Interpolate(
										m_playingMotion.player[j].keyframes[i].effort,
										m_playingMotion.player[j].keyframes[i
												+ 1].effort);
					}
					//----------End Interpolation----------
				}

				m_playingMotion.frames.erase(
						m_playingMotion.frames.begin() + delInx);
				for (unsigned j = 0; j < m_playingMotion.player.size(); j++)
				{
					m_playingMotion.player[j].keyframes.erase(
							m_playingMotion.player[j].keyframes.begin()
									+ delInx);
				}
			}
		}
		else
		{
			//ROS_INFO("*** %s", m_playingMotion.frames[i]->name.c_str());
		}
	}
	//ROS_INFO("after = %d", (int )m_playingMotion.frames.size());
	
	// Try to apply rules
	bool apply = false;
	
	if(m_playingMotion.rules.size() > 0)
	{
		std::vector<FloatParamPtr> parameters = m_ruleParameters[motion];
		
		// Check if we can apply every rule
		bool limit_inverse = m_limit_inverse_space->get();
		float epsilon = m_epsilon->get();
		
		for(size_t i = 0; i < parameters.size(); i++)
		{
			if(i >= m_playingMotion.rules.size())
				break;
			
			float delta = parameters[i]->get();
			
			if(delta != 0)
			{
				apply = m_playingMotion.canApplyRule(i, delta, limit_inverse, epsilon);
				if(!apply)
					break;
			}
		}
		
		if(apply) // Apply rules
		{
			for(size_t i = 0; i < parameters.size(); i++)
			{
				if(i >= m_playingMotion.rules.size())
					break;
				
				float delta = parameters[i]->get();
				
				if(delta != 0)
					apply = m_playingMotion.applyRule(i, delta, limit_inverse, epsilon);
			}
			ROS_INFO("Successfully applied all rules!");
		}
		else
			ROS_ERROR("Rules were NOT applied");
	}
	
	// Update motion
	if(apply)
	{
		std::vector<std::string> jointList = m_playingMotion.jointList;
		m_playingMotion.motionToModel.resize(jointList.size());
		for (unsigned i = 0; i < jointList.size(); i++)
			m_playingMotion.motionToModel[i] = findIndex(jointList[i]);

		m_playingMotion.reqState = m_model->registerState(m_playingMotion.preState);
		m_playingMotion.transState = m_model->registerState(m_playingMotion.playState);
		m_playingMotion.resState = m_model->registerState(m_playingMotion.postState);
		m_playingMotion.initPlayer();
	}

	for (unsigned i = 0; i < m_playingMotion.player.size(); i++)
	{
//		ROS_INFO("player (%d) = %d",(int)i,(int)m_playingMotion.player[i].keyframes.size());
//		ROS_INFO("player2 (%d) = %d",(int)i,(int)m_playingMotion.player[i].commands.size());
		kf_player::KeyframePlayer* currentPlayer = &m_playingMotion.player[i];
		int index = m_playingMotion.motionToModel[i];
		robotcontrol::Joint::Ptr joint = (*m_model)[index];
		currentPlayer->keyframes[0].x = joint->cmd.pos;
		m_playingMotion.player[i].calculateCommands();
	}

	startPlaying();
	m_playingMotion.resetPlayer();
	m_model->setState(m_playingMotion.transState);
}

void MotionPlayer::finished()
{
	m_model->setState(m_playingMotion.resState);
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

	ROS_INFO("Loading motions: %s", dir.c_str());
	for (; file != end; ++file)
	{
		if (fs::is_directory(*file))
			continue;

		fName = file->path().leaf().string();

		if (!boost::regex_search(fName, boost::regex("\\.yaml$")))
			continue;

		fName = fName.substr(0, fName.size() - suffix.size());

		if (newMotion.load(file->path().string()))
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
	if (bad > 0)
		ROS_INFO("Encountered %d bad motion files!", bad);
	ROS_INFO("Successfully loaded %d motions!", count);

	// Create fictious motions for playing fram/motion from trajectory_editor_2
	MappedMotion nullMotion;

	std::pair<std::string, MappedMotion> entry1("trajectory_editor_motion",
			nullMotion);
	m_motionNames.insert(entry1);

	std::pair<std::string, MappedMotion> entry2("trajectory_editor_frame",
			nullMotion);
	m_motionNames.insert(entry2);

	return true;
}

bool MotionPlayer::reloadMotionFiles(const fs::path& dir)
{
	if (!fs::exists(dir) || !fs::is_directory(dir))
		return false;

	fs::directory_iterator end;
	fs::directory_iterator file(dir);

	std::string fName;
	std::string suffix = ".yaml";

	MappedMotion newMotion;
	int count = 0, bad = 0;

	ROS_INFO("Reloading motions: %s", dir.c_str());
	for (; file != end; ++file)
	{
		if (fs::is_directory(*file))
			continue;

		fName = file->path().leaf().string();

		if (!boost::regex_search(fName, boost::regex("\\.yaml$")))
			continue;

		fName = fName.substr(0, fName.size() - suffix.size());

		if (newMotion.load(file->path().string()))
		{
			if (m_motionNames.count(fName))
			{
				m_motionNames[fName] = newMotion;
				ROS_INFO("- %s", fName.c_str());
				count++;
			}
			else
			{
				std::pair<std::string, MappedMotion> entry(fName, newMotion);
				m_motionNames.insert(entry);
				
				// add motion
				ROS_ERROR("Programming Error!");
				return false;
			}
		}
		else
		{
			ROS_WARN("- %s [INVALID]", fName.c_str());
			bad++;
		}
	}
	if (bad > 0)
		ROS_INFO("Encountered %d bad motion files!", bad);
	ROS_INFO("Successfully reloaded %d motions!", count);

	return true;
}

bool MotionPlayer::handleUpdateMotion(motion_player::StreamMotionRequest& req,
		motion_player::StreamMotionResponse& res)
{
	if (isPlaying)
	{
		ROS_ERROR(
				"Tried to update motion. Not possible because I am still playing a motion!");
		return false;
	}

	std::string motionName;

	if (req.type == 0) // arbitrary motion
		motionName = req.name;
	else if (req.type == 1) // motion from trajectory_editor
		motionName = "trajectory_editor_motion";
	else if (req.type == 2) // frame from trajectory_editor
		motionName = "trajectory_editor_frame";
	else
	{
		ROS_ERROR("Undefined type of update request (have to be [0, 2])");
		return false;
	}

	if (m_motionNames.find(motionName) == m_motionNames.end())
	{
		ROS_ERROR("Tried to update motion '%s', but it was not found!",
				motionName.c_str());
		return false;
	}

	MappedMotion nMotion;

	if (!nMotion.parse(req.motion))
	{
		ROS_ERROR("Failed to parse motion '%s' for updating!",
				motionName.c_str());
		return false;
	}

	std::vector<std::string> jointList = nMotion.jointList;
	nMotion.motionToModel.resize(jointList.size());
	for (unsigned i = 0; i < jointList.size(); i++)
		nMotion.motionToModel[i] = findIndex(jointList[i]);

	nMotion.reqState = m_model->registerState(nMotion.preState);
	nMotion.transState = m_model->registerState(nMotion.playState);
	nMotion.resState = m_model->registerState(nMotion.postState);
	nMotion.initPlayer();

	m_motionNames[motionName] = nMotion;

	ROS_INFO("Successfully updated motion '%s'", motionName.c_str());
	return true;
}

bool MotionPlayer::handleReloadMotion(std_srvs::Empty::Request& req,
		std_srvs::Empty::Response& res)
{
	if (isPlaying)
	{
		ROS_ERROR(
				"Tried to reload motion. Not possible because I am still playing a motion!");
		return false;
	}

	if (model()->state() == m_state_relaxed || model()->state() == m_state_init
			|| model()->state() == m_state_standing
			|| model()->state() == m_state_sitting)
	{
		fs::path p((ros::package::getPath("launch") + "/motions"));

		if (m_robot_name.empty() || m_robot_type.empty()
				|| !reloadMotionFiles(
						p / ("/" + m_robot_type + "_" + m_robot_name)))
		{
			ROS_INFO("No robot specific motions found.");
		}

		if (m_robot_type.empty() || !reloadMotionFiles(p / ("/" + m_robot_type)))
		{
			ROS_INFO("No robot type specific motions found.");
		}

		if (!reloadMotionFiles(p / ("/default")))
		{
			ROS_ERROR(
					"Default motion folder not found. Failed to init motion player.");
			return false;
		}
		initMotions();
		ROS_INFO("Successfully reloaded motions");
		return true;
	}
	else
	{
		ROS_ERROR("You are not in the right state for reloading motions.");
		return false;
	}

}

void MotionPlayer::step()
{
	// Retrieve the system iteration time
	m_dT = m_model->timerDuration();

	// Don't do anything if we're not playing anything
	if (!isPlaying)
		return;

	// Update the pose to command to the robot
	kf_player::Keyframe cmdFrame;
	//ROS_INFO("Platying size= %ld",m_playingMotion.player.size());

	//TODO: put this chunk in a good place
	//BGN
	PM.clear();
	double pitchPR = m_model->robotFPitchPR();
	double pitchDPR = m_model->robotDFPitchPR();
	double rollPR = m_model->robotFRollPR();
	double rollDPR = m_model->robotDFRollPR();
	double yaw = m_model->robotFYaw();
	double yawD = m_model->robotDFYaw();

	PM.plotScalar(m_playingMotion.player[0].currentState.playingIndex,
			PM_FRAME_INDEX);

	PM.plotScalar(rollPR, PM_ROLL);
	PM.plotScalar(pitchPR, PM_PITCH);
	PM.plotScalar(yaw, PM_YAW);

	PM.plotScalar(rollDPR, PM_ROLLD);
	PM.plotScalar(pitchDPR, PM_PITCHD);
	PM.plotScalar(yawD, PM_YAWD);

	PM.plotScalar(m_playingMotion.player[0].currentState.roll, PM_DES_ROLL);
	PM.plotScalar(m_playingMotion.player[0].currentState.pitch, PM_DES_PITCH);
	PM.plotScalar(m_playingMotion.player[0].currentState.yaw, PM_DES_YAW);

	for (unsigned i = 0; i < m_playingMotion.player.size(); i++)
	{
		if (m_playingMotion.player[i].atEnd())
		{
			finished();
			return;
		}
		cmdFrame = m_playingMotion.player[i].step(m_dT);
		int index = m_playingMotion.motionToModel[i];

		m_cmdPositions[index] = cmdFrame.x;
		m_cmdVelocities[index] = cmdFrame.v;
		m_cmdAccelerations[index] = cmdFrame.a;
		m_cmdEffort[index] = cmdFrame.effort;
		
		robotcontrol::Joint::Ptr joint = (*m_model)[index];

		if (m_playingMotion.pidEnabled)
		{
			//To detect change in gains
			bool controllerChanged = (m_preGains[i] != cmdFrame.gainSelect);//|| (m_preAngle(0)!=cmdFrame.roll || m_preAngle(1)!=cmdFrame.pitch||m_preAngle(2)!=cmdFrame.yaw)
			//The reason I use this bool rather than check the frame change is having integral valid if two continuous frames have the same desired angles
			if (controllerChanged)
			{

				m_integralGains[i] = 0.0;			//reset the integrals
				PM.plotEvent("state/cC_" + joint->name);
			}

			const double aplphaRoll = 0.997692;
			const double aplphaPitch = aplphaRoll;
			const double aplphaYaw = aplphaRoll;
			cmdFrame.iGain = cmdFrame.iGain / 30;
			cmdFrame.pGain = cmdFrame.pGain * 3;
			double errorC = 0.0;
			double correctionC = 0.0;
			switch (cmdFrame.gainSelect)
			{
			case kf_player::rollE:
				errorC = cmdFrame.roll - rollPR;
				correctionC = cmdFrame.pGain * (errorC)
						+ cmdFrame.iGain * m_integralGains[i]
						+ cmdFrame.dGain * (-rollDPR);
				m_integralGains[i] = errorC + aplphaRoll * m_integralGains[i];
				//ROS_INFO("Roll(%.2f - %.2f) =>  %.2f = %.2f * %.2f + %.2f * %.2f + %.2f * %.2f",cmdFrame.roll,rollPR,correctionC,cmdFrame.pGain,(errorC),cmdFrame.iGain,m_integralGains[i],cmdFrame.dGain,(-rollDPR));
				break;
			case kf_player::pitchE:
				errorC = cmdFrame.pitch - pitchPR;
				correctionC = cmdFrame.pGain * (errorC)
						+ cmdFrame.iGain * m_integralGains[i]
						+ cmdFrame.dGain * (-pitchDPR);
				m_integralGains[i] = errorC + aplphaPitch * m_integralGains[i];
				//ROS_INFO("Pitch(%.2f - %.2f) =>  %.2f = %.2f * %.2f + %.2f * %.2f + %.2f * %.2f",cmdFrame.pitch,pitchPR,correctionC,cmdFrame.pGain,(errorC),cmdFrame.iGain,m_integralGains[i],cmdFrame.dGain,(-pitchDPR));
				break;
			case kf_player::yawE:
				errorC = cmdFrame.yaw - yaw;
				correctionC = cmdFrame.pGain * (errorC)
						+ cmdFrame.iGain * m_integralGains[i]
						+ cmdFrame.dGain * (-yawD);
				m_integralGains[i] = errorC + aplphaYaw * m_integralGains[i];
				break;
			default:
				m_integralGains[i] = 0.0;			//reset the integrals
				break;
			}
			
			int sign = 1;
			if(correctionC < 0)
				sign = -1;
			
			double limit = cmdFrame.limit;
				
			// Check limit
			if(fabs(correctionC) < limit)
				m_cmdPositions[index] += correctionC;
			else
				m_cmdPositions[index] += limit * sign;

			//Save current value to the pre value
			m_preGains[i] = cmdFrame.gainSelect;
			m_preAngle(0) = cmdFrame.roll;
			m_preAngle(1) = cmdFrame.pitch;
			m_preAngle(2) = cmdFrame.yaw;
		}
	}
	PM.publish();
	//END

	// Set the required support coefficients
	m_model->resetSupport();
	m_model->setSupportCoefficient(m_leftFootLink, cmdFrame.suppLeftLeg); // Note: Each keyframe player should return the same support coefficient information,
	m_model->setSupportCoefficient(m_rightFootLink, cmdFrame.suppRightLeg); //       so we just take the last one and hope for the best...

	// Send off the joint position and effort commands
	writeCommands();
}

void MotionPlayer::writeCommands()
{
	size_t nj = m_cmdPositions.size();
	for (size_t i = 0; i < nj; i++)
	{
		robotcontrol::Joint::Ptr joint = (*m_model)[i];
		joint->cmd.setFromPosVelAcc(m_cmdPositions[i], m_cmdVelocities[i],
				m_cmdAccelerations[i]);
		joint->cmd.raw = false;
		joint->cmd.effort = m_cmdEffort[i];
	}
}

}

PLUGINLIB_EXPORT_CLASS(motionplayer::MotionPlayer, robotcontrol::MotionModule)
