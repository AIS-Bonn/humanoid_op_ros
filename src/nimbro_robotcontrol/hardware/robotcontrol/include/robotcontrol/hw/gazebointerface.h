// Gazebo hardware interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef GAZEBOINTERFACE_H
#define GAZEBOINTERFACE_H

#include "roscontrolinterface.h"
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>

namespace robotcontrol
{

class GazeboInterface : public ROSControlInterface
{
public:
	GazeboInterface();
	virtual ~GazeboInterface();

	virtual bool init(RobotModel* model);
	virtual bool readJointStates();
	
private:
	void handleModelStates(const gazebo_msgs::ModelStatesConstPtr& ms);

	RobotModel* m_model;
	std::string m_modelName;
	gazebo_msgs::ModelStatesConstPtr m_last_modelStates;
	ros::Time m_last_modelStatesStamp;
	ros::Time m_initTime;
	ros::Subscriber m_sub_modelStates;
	bool m_publishOdom;
	tf::TransformBroadcaster m_pub_tf;
};

}

#endif
