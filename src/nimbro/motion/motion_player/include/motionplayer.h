//Motion module to play motions from trajectories
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>, Hafez Farazi <farazi@ais.uni-bonn.de>

#ifndef MOTION_PLAYER_H
#define MOTION_PLAYER_H

#include <robotcontrol/model/robotmodel.h>
#include <robotcontrol/motionmodule.h>
#include <robotcontrol/FadeTorqueAction.h>

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/progress.hpp"
#include "boost/regex.hpp"

#include <actionlib/client/simple_action_client.h>

#include <motion_player/PlayMotion.h>
#include <motion_player/StreamMotion.h>
#include <motion_player/DumpMotions.h>

#include <keyframe_player/KeyframePlayer.h>
#include <motion_file/motionfile.h>
#include <plot_msgs/plot_manager.h>
#include <ros/timer.h>
#include <Interpolator.hpp>

#include <std_srvs/Empty.h>

#include <motion_file/ruleapplier.h>

namespace motionplayer
{

class MappedMotion: public motionfile::Motion
{
public:
	std::vector<int> motionToModel;
	robotcontrol::RobotModel::State reqState;
	robotcontrol::RobotModel::State transState;
	robotcontrol::RobotModel::State resState;

	std::vector<kf_player::KeyframePlayer> player;

	MappedMotion(const MappedMotion& _in) :
		motionfile::Motion(_in)
	{
		reqState = _in.reqState;
		transState = _in.transState;
		resState = _in.resState;
		motionToModel = _in.motionToModel;
		player = _in.player;
	} // copy ctor

	MappedMotion()
	{

	}

	MappedMotion& operator=(MappedMotion _in)
	{
		if (this != &_in)
		{
			motionfile::Motion::operator=(_in);
			reqState = _in.reqState;
			transState = _in.transState;
			resState = _in.resState;
			motionToModel = _in.motionToModel;
			player = _in.player;
		}
		return *this;
	}

	void resetPlayer();
	void initPlayer();
};

class MotionPlayer : public robotcontrol::MotionModule
{
public:
	typedef boost::shared_ptr<config_server::Parameter<float> > FloatParamPtr;
	typedef boost::shared_ptr<config_server::Parameter<bool> >  BoolParamPtr;
	
	MotionPlayer();
	virtual ~MotionPlayer() {}

	virtual bool init(robotcontrol::RobotModel* model);
	virtual void step();
	virtual bool isTriggered();

	static const std::string RESOURCE_PATH;
	static const std::string CONFIG_PARAM_PATH;

private:
	ros::NodeHandle m_nh;

	std::map<std::string, MappedMotion>  m_motionNames;
	std::map<std::string, std::vector<FloatParamPtr> > m_ruleParameters;
	BoolParamPtr  m_limit_inverse_space;
	FloatParamPtr m_epsilon;
	
	MappedMotion m_playingMotion;
	
	motionfile::RuleApplier m_ruleApplier;

	robotcontrol::RobotModel* m_model;
	double m_dT;
	
	boost::shared_ptr<const urdf::Link> m_leftFootLink;  // Left foot URDF link (used for setting support coefficients)
	boost::shared_ptr<const urdf::Link> m_rightFootLink; // Right foot URDF link (used for setting support coefficients)

	ros::ServiceServer m_srv_play;
	ros::ServiceServer m_srv_update;
	ros::ServiceServer m_srv_reload;
	ros::ServiceServer m_srv_dump_motions;

	std::vector<double> m_cmdPositions;
	std::vector<double> m_cmdVelocities;
	std::vector<double> m_cmdAccelerations;
	std::vector<double> m_cmdEffort;

	//previous gain_select
	std::vector<kf_player::gainSelectEnum> m_preGains;
	//previous angles(roll,pitch,yaw)
	Eigen::Vector3f m_preAngle;
	std::vector<double> m_integralGains;


	actionlib::SimpleActionClient<robotcontrol::FadeTorqueAction> m_torqueAct;

	// Predefined states
	robotcontrol::RobotModel::State m_state_relaxed;
	robotcontrol::RobotModel::State m_state_setting_pose;
	robotcontrol::RobotModel::State m_state_init;
	robotcontrol::RobotModel::State m_state_standing;
	robotcontrol::RobotModel::State m_state_sitting;
	robotcontrol::RobotModel::State m_state_falling;
	robotcontrol::RobotModel::State m_state_lying_prone;
	robotcontrol::RobotModel::State m_state_lying_supine;
	static const std::string m_fallingStatePrefix;

	// Robot specification
	std::string m_robot_name;
	std::string m_robot_type;

	// Standard motion names
	std::string m_init_pose_name;
	std::string m_init_name;
	std::string m_getup_prone_name;
	std::string m_getup_supine_name;
	
	ros::Timer timer;
	ros::Publisher statePublisher;

	bool handlePlay(motion_player::PlayMotionRequest& req, motion_player::PlayMotionResponse& res);
	bool handleUpdateMotion(motion_player::StreamMotionRequest& req, motion_player::StreamMotionResponse& res);
	bool handleReloadMotion(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
	bool handleDumpMotions(motion_player::DumpMotionsRequest& req, motion_player::DumpMotionsResponse& res); // Saves all currently loaded motions into temp folder

	bool tryToPlay(const std::string& motion, bool checkState = true);
	void play(const std::string& motion);
	void finished();

	bool m_isRelaxed;
	bool m_isInitialized;
	bool isPlaying;

	inline void startPlaying() { isPlaying = true; };
	inline void stopPlaying() { isPlaying = false; };

	void writeCommands();

	void initMotions();
	void initRuleParameters();
	
	void applyRule(motionplayer::MappedMotion &motion, int rule_id, const float delta);
	
	void publishState(const ros::TimerEvent& event);

	bool loadMotionFiles(const boost::filesystem::path& dir);
	bool reloadMotionFiles(const boost::filesystem::path& dir);
	int findIndex(std::string name);

	enum PMIds
	{
		PM_FRAME_INDEX=0,
		PM_ROLL,
		PM_PITCH,
		PM_YAW,
		PM_ROLLD,
		PM_PITCHD,
		PM_YAWD,
		PM_DES_ROLL,
		PM_DES_PITCH,
		PM_DES_YAW,
		PM_COUNT
	};
	plot_msgs::PlotManagerFS PM;
};

}

#endif
