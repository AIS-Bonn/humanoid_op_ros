// Handles requests for play moton/frame, update motion, etc
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef MOTION_PLAYER_CLIENT_H
#define MOTION_PLAYER_CLIENT_H

#include <vector>
#include <string>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>

#include <motion_file/motionfile.h>

class MotionPlayerClient
{
public:
	typedef boost::shared_ptr<motionfile::Keyframe> KeyframePtr;
	
    MotionPlayerClient();
    ~MotionPlayerClient();
	
	void playMotion(motionfile::Motion motion, float duration_factor);
	void playFrame(KeyframePtr frame, std::vector<std::string> joint_list, float duration_factor);
	void playSequence(motionfile::Motion motion, std::vector<int> indices);
	void updateMotion(motionfile::Motion motion);
	
	void tryPlayDelayedMotion(); // Plays a delayed motion if it exists
	
private:
	bool requestPlay(motionfile::Motion &motion, int type);
	bool requestUpdate(motionfile::Motion &motion, int type);

private:
	bool m_has_delayed_motion;
	motionfile::Motion m_delayed_motion; // Motion which is played after first frame was played completely
	
	ros::ServiceClient m_update_client;
	ros::ServiceClient m_play_client;
};

#endif
