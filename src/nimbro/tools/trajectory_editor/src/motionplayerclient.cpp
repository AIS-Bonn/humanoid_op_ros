#include <trajectory_editor/motionplayerclient.h>

#include <motion_player/StreamMotion.h>
#include <motion_player/PlayMotion.h>

// Types for motion player. Do not change
#define TYPE_UPDATE_MOTION 0 // Updates motion on motion player as it is
#define TYPE_MOTION 1
#define TYPE_FRAME 2

MotionPlayerClient::MotionPlayerClient()
	:m_has_delayed_motion(false)
{
	ros::NodeHandle nh("~");

	m_update_client = nh.serviceClient<motion_player::StreamMotion>("/motion_player/update");
	m_play_client   = nh.serviceClient<motion_player::PlayMotion>("/motion_player/play");
}

void MotionPlayerClient::playMotion(motionfile::Motion motion, float duration_factor)
{
	if(motion.frames.size() < 1)
		return;

	// Play first frame first (init robot beginning position)
	double old_duration = motion.frames[0]->duration;
	playFrame(motion.frames[0], motion.jointList, 1.5);
	motion.frames[0]->duration = old_duration;
	
	// Apply duration factor to each frame of motion
	for(unsigned i = 0; i < motion.frames.size(); i++)
		motion.frames[i]->duration *= duration_factor;
	
	m_has_delayed_motion = true;
	m_delayed_motion = motion;
}

void MotionPlayerClient::playFrame(KeyframePtr frame, std::vector<std::string> joint_list, float duration_factor)
{
	// Construct temp motion
	motionfile::Motion tempMotion;
	tempMotion.motionName = "FramePlay";
	tempMotion.jointList  = joint_list;
	tempMotion.playState  = "init";
	tempMotion.preState   = "init";
	tempMotion.postState  = "init";
	
	// Caclulate duration with factor
	double duration = frame->duration * duration_factor;
	if(duration < 1)
		duration = 1; // Minimum possible duration
	
	frame->duration = duration;
	
	tempMotion.frames.push_back(frame);
	tempMotion.frames.push_back(frame);
	
	 // Update & play frame
	requestUpdate(tempMotion, TYPE_FRAME);
	requestPlay(tempMotion, TYPE_FRAME);
}

void MotionPlayerClient::playSequence(motionfile::Motion motion, std::vector<int> indices)
{
	if(motion.frames.size() < 1)
		return;
	
	// Construct motion with given frames
	motionfile::Motion sequenceMotion;
	sequenceMotion.motionName = motion.motionName + "_sequence";
	sequenceMotion.jointList  = motion.jointList;
	sequenceMotion.playState  = motion.playState;
	sequenceMotion.preState   = motion.preState;
	sequenceMotion.postState  = motion.postState;
	
	for(unsigned i = 0; i < indices.size(); i++)
		sequenceMotion.frames.push_back(motion.frames[indices.at(i)]);
	
	// Play first frame of motion
	playFrame(sequenceMotion.frames[0], motion.jointList, 1.5);
	
	m_has_delayed_motion = true;
	m_delayed_motion = sequenceMotion;
}

void MotionPlayerClient::updateMotion(motionfile::Motion motion)
{
	if(motion.frames.size() < 1)
		return;
	
	requestUpdate(motion, TYPE_UPDATE_MOTION);
}

void MotionPlayerClient::tryPlayDelayedMotion()
{
	if(m_has_delayed_motion == false)
		return;
	
	if(requestUpdate(m_delayed_motion, TYPE_MOTION) == false)
		return;
	
	requestPlay(m_delayed_motion, TYPE_MOTION);
	m_has_delayed_motion = false;
}

bool MotionPlayerClient::requestPlay(motionfile::Motion& motion, int type)
{
	motion_player::PlayMotion play;
	play.request.name = motion.motionName;
	play.request.type = type;

	return m_play_client.call(play);
}

bool MotionPlayerClient::requestUpdate(motionfile::Motion& motion, int type)
{
	motion_player::StreamMotion update;
	update.request.name = motion.motionName;
	update.request.motion = motion.dump();
	update.request.type = type;

	return m_update_client.call(update);
}

MotionPlayerClient::~MotionPlayerClient()
{

}