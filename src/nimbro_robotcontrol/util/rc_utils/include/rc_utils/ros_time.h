// Utilities for ROS time
// File: ros_time.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef ROS_TIME_H
#define ROS_TIME_H

// Includes
#include <ros/time.h>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @name ROS Time Functions (rc_utils/ros_time.h)
	**/
	///@{

	//! @brief Set a ROS `ros::Time` object to zero.
	inline void zeroRosTime(ros::Time& time)
	{
		// Write zero as required
		time.sec = time.nsec = 0;
	}

	//! @brief Set a ROS `ros::WallTime` object to zero.
	inline void zeroRosTime(ros::WallTime& time)
	{
		// Write zero as required
		time.sec = time.nsec = 0;
	}

	///@}
}

#endif /* ROS_TIME_H */
// EOF