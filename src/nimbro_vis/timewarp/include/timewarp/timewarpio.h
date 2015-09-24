// Provides functionality for loading and saving timewarp data to a bag file
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef TIMEWARPIO_H
#define TIMEWARPIO_H

// Includes
#include <string>
#include <ros/time.h>

// Timewarp namespace
namespace timewarp
{
	// Forward declarations
	class TimeWarpNode;
	
	// Timewarp I/O class
	class TimeWarpIO
	{
	public:
		// Bag I/O functions
		static bool load(TimeWarpNode* tw, const std::string& path);
		static bool save(const TimeWarpNode* tw, const std::string& path, bool append, ros::Time startTime = ros::Time(0.0), ros::Time stopTime = ros::Time(0.0));
		
	private:
		// Constructor
		TimeWarpIO() {}
	};
}

#endif
// EOF