//Simple face focusing for demonstration purpose
//author: Sebastian Schüller

#ifndef FACE_TRACKER_H
#define FACE_TRACKER_H

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <config_server/parameter.h>
#include <face_detection/Faces.h>
#include <head_control/LookAtTarget.h>

class FaceTracker
{
public:
	FaceTracker(ros::NodeHandle nh);
	~FaceTracker(){};

	void startTracking();
	void stopTracking();

private:
	enum tracking_states
	{
		LOOK_FOR_TARGET,
		MOVE_TO_TARGET,
		IGNORE_TARGETS,
		LOCAL_TRACKING,
		SHUTDOWN
	};


	void initTargets();
	void handleFaces(const face_detection::FacesConstPtr& faces);


	bool countdownDone(int* countdown);

	head_control::LookAtTarget findClosestFace(const std::vector< face_detection::Rectangle >& faces);
	head_control::LookAtTarget findMostCentralTarget(const std::vector< face_detection::Rectangle >& faces);
	head_control::LookAtTarget findNeighborTarget(const std::vector< face_detection::Rectangle >& faces);


	ros::NodeHandle m_nh;
	ros::Subscriber m_faces_sub;
	ros::Publisher m_target_pub;

	tracking_states m_current_state;

	const int DFLT_CNT;
	int m_shutdown_countdown;
	int m_ignore_countdown;
	int m_init_countdown;

	bool m_init_initialised;
	bool m_shutdown_initialised;


	head_control::LookAtTarget m_zero_target;
	head_control::LookAtTarget m_target;

	config_server::Parameter< bool > m_enable_head;
	config_server::Parameter< float > m_local_threshold;
	config_server::Parameter< int > m_rec_attempts;
	int m_attempts;

};

#endif