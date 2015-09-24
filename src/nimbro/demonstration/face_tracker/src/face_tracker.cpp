//Simple face focusing for demonstration purpose
//Author: Sebastian Schüller

#include <cmath>

#include <face_tracker/face_tracker.h>

inline double distance(const head_control::LookAtTarget& a, const head_control::LookAtTarget& b)
{return fabs(sqrt((a.vec.x - b.vec.x) * (a.vec.x - b.vec.x) + (a.vec.y - b.vec.y) * (a.vec.y - b.vec.y)));}

inline double distance(const face_detection::Rectangle& a)
{return (double) fabs((sqrt((a.x) * (a.x) + (a.y) * (a.y))));}


FaceTracker::FaceTracker(ros::NodeHandle nh)
 : m_nh(nh)
 , m_current_state(LOOK_FOR_TARGET)
 , DFLT_CNT(50)
 , m_shutdown_countdown(DFLT_CNT)
 , m_ignore_countdown(DFLT_CNT)
 , m_init_countdown(0)
 , m_init_initialised(true)
 , m_shutdown_initialised(false)
 , m_enable_head("/head_control/head_enabled", false)
 , m_local_threshold("/face_tracker/local_threshold", 0, 0.1, 400, 50)
 , m_rec_attempts("/face_tracker/attempts", 0, 1, 100, 10)
 , m_attempts(m_rec_attempts())
{

	initTargets();

	m_faces_sub = m_nh.subscribe<face_detection::Faces>(
		"/face_detection/faces",
		20,
		&FaceTracker::handleFaces,
		this
	);

	m_target_pub = m_nh.advertise<head_control::LookAtTarget>("/head_control/target", 1);

}

void FaceTracker::initTargets()
{
	m_zero_target.is_angular_data = true;
	m_zero_target.is_relative     = false;
	m_zero_target.vec.x = 0;
	m_zero_target.vec.y = 0;
	m_zero_target.vec.z = 0;
}

bool FaceTracker::countdownDone(int* countdown)
{
	//ROS_INFO("Current Countdown is: %d", *countdown);
	if ((*countdown) < 0)
	{
		//ROS_INFO("Should reset countdown");
		*countdown = DFLT_CNT;
		return true;
	}
	*countdown -= 1;
	return false;
}


void FaceTracker::handleFaces(const face_detection::FacesConstPtr& faces)
{
	switch (m_current_state)
	{
		case LOOK_FOR_TARGET:
			//Move to default position ((0,0))
			//and select largest rectangle as main target
			if (! m_init_initialised)
			{
				m_target_pub.publish<head_control::LookAtTarget>(m_zero_target);
				m_init_initialised = true;
				m_init_countdown = DFLT_CNT;
			}

			//If state wasn't set from the outside, wait some time to make sure
			//that the head is at point (0,0)
			if (! countdownDone(&m_init_countdown))
				break;

			ROS_INFO("Looking for possible target");
			if (faces->faces.size() == 0)
				break;

			//Find the closest face to the camera
			m_target = findClosestFace(faces->faces);
			ROS_INFO("Set target to Rectangle at (%f,%f)", m_target.vec.x, m_target.vec.y);
			//Change to the next state
			ROS_INFO("Change to state: MOVE_TO_TARGET");
			m_current_state = MOVE_TO_TARGET;
			break;

		case MOVE_TO_TARGET:
			//Move to the main target
			ROS_INFO("Move the head to the main target");
			m_target_pub.publish<head_control::LookAtTarget>(m_target);
			ROS_INFO("Change to state: IGNORE_TARGETS");
			m_current_state = IGNORE_TARGETS;
			break;

		case IGNORE_TARGETS:
			//Ignore any target for a given time
			if (! countdownDone(&m_ignore_countdown))
				break;
			m_target = findMostCentralTarget(faces->faces);
			ROS_INFO("Refine main target to (%f,%f)", m_target.vec.x, m_target.vec.y);


			ROS_INFO("Change to state: LOCAL_TRACKING");
			m_current_state = LOCAL_TRACKING;

		case LOCAL_TRACKING:
		{
			//Update local target
			if (m_attempts <= 0)
			{
				m_current_state = LOOK_FOR_TARGET;
				m_attempts = m_rec_attempts();
				break;
			}
			m_attempts--;

			//TODO Do actual local tracking
// 			head_control::LookAtTarget local_target = findNeighborTarget(faces->faces);
// 			m_target_pub.publish<head_control::LookAtTarget>(local_target);
			break;
		}
		case SHUTDOWN:
			if (! m_shutdown_initialised)
			{
				m_target_pub.publish<head_control::LookAtTarget>(m_zero_target);
				m_shutdown_initialised = true;
			}
			if (countdownDone(&m_shutdown_countdown))
				m_enable_head.set(false);
			break;
	}
}

head_control::LookAtTarget FaceTracker::findClosestFace(const std::vector< face_detection::Rectangle >& faces)
{
	if (faces.size() == 0)
		return head_control::LookAtTarget();

	unsigned int i;
	face_detection::Rectangle closest_rec = faces[0];
	for (i = 0; i < faces.size(); ++i)
	{
		if (closest_rec.height < faces[i].height)
			closest_rec = faces[i];
	}

	head_control::LookAtTarget target;
	target.is_angular_data = false;
	target.is_relative     = true;
	target.vec.z = 1;
	target.vec.x = closest_rec.x;
	target.vec.y = closest_rec.y;
	return target;
}

head_control::LookAtTarget FaceTracker::findMostCentralTarget(const std::vector< face_detection::Rectangle >& faces)
{
	if (faces.size() == 0)
		return head_control::LookAtTarget();

	face_detection::Rectangle closest_rec = faces[0];
	double min_dist = distance(closest_rec);
	unsigned int i;
	for (i = 0; i < faces.size(); ++i)
	{
		if (distance(faces[i]) <= min_dist)
		{
			min_dist    = distance(faces[i]);
			closest_rec = faces[i];
		}
	}

	head_control::LookAtTarget target;
	target.is_angular_data = false;
	target.is_relative     = true;
	target.vec.z = 1;
	target.vec.x = closest_rec.x;
	target.vec.y = closest_rec.y;
	return target;
}

head_control::LookAtTarget FaceTracker::findNeighborTarget(const std::vector< face_detection::Rectangle >& faces)
{
	if (faces.size() == 0)
		return head_control::LookAtTarget();
	return head_control::LookAtTarget();
}

void FaceTracker::startTracking()
{
	m_enable_head.set(true);
	m_current_state    = LOOK_FOR_TARGET;
	m_init_countdown   = 0;
	m_init_initialised = true;
}

void FaceTracker::stopTracking()
{m_current_state = SHUTDOWN;}
