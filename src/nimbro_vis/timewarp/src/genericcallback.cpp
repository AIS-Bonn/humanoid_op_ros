// Generic callback for ROS callback queue
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <timewarp/genericcallback.h>

namespace timewarp
{

GenericCallback::GenericCallback(const boost::function< void() >& cb)
 : m_cb(cb)
{
}

GenericCallback::~GenericCallback()
{
}

ros::CallbackInterface::CallResult GenericCallback::call()
{
	m_cb();
	return Success;
}

}
