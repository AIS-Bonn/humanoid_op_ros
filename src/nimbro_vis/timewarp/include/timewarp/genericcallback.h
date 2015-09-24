// Generic callback for ROS callback queue
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef GENERICCALLBACK_H
#define GENERICCALLBACK_H
#include <ros/callback_queue_interface.h>

namespace timewarp
{

class GenericCallback : public ros::CallbackInterface
{
public:
	explicit GenericCallback(const boost::function<void()>& cb);
	virtual ~GenericCallback();
	virtual CallResult call();
private:
	boost::function<void()> m_cb;
};

}

#endif
