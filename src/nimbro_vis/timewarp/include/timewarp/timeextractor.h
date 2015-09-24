// Extracts time information from a buffer entry
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TIMEEXTRACTOR_H
#define TIMEEXTRACTOR_H

#include <visualization_msgs/MarkerArray.h>

namespace timewarp
{

// Variant for Messages providing a header (SFINAE in action!)
template <typename T>
struct has_header {
	typedef char yes[1];
	typedef char no[2];

	template <typename C>
	static yes& test(typename C::_header_type*);

	template <typename>
	static no& test(...);

	// If the "sizeof" of the result of calling test<T>(0) is equal to sizeof(yes),
	// the first overload worked (passed null pointer) and T has a nested type named '_header_type'.
	// This member can be used as: has_header<TypeToTest>::value (evaluates to bool)
	static const bool value = sizeof(test<T>(0)) == sizeof(yes);
};

template<class MsgType>
typename boost::enable_if<has_header<MsgType>, ros::Time>::type extractTime(const boost::shared_ptr<topic_tools::ShapeShifter> data)
{
	boost::shared_ptr<MsgType> ptr = data->instantiate<MsgType>();
	assert(ptr);

	return ptr->header.stamp;
}

template<class MsgType>
typename boost::disable_if<has_header<MsgType>, ros::Time>::type extractTime(const boost::shared_ptr<topic_tools::ShapeShifter>);

// Specializations
template<> inline
ros::Time extractTime<visualization_msgs::MarkerArray>(const boost::shared_ptr<topic_tools::ShapeShifter> data)
{
	boost::shared_ptr<visualization_msgs::MarkerArray> ptr = data->instantiate<visualization_msgs::MarkerArray>();

	if(!ptr || ptr->markers.size() == 0 || ptr->markers[0].header.stamp == ros::Time(0))
		return ros::Time();

	return ptr->markers[0].header.stamp;
}

}

#endif
