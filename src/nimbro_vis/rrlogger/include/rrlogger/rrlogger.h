// Maintain round robin rosbags
// author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#ifndef RRLOGGER_H
#define RRLOGGER_H

#include <rosbag/bag.h>
#include <topic_tools/shape_shifter.h>


namespace logger
{

class RRLogger
{
public:
	RRLogger();
	~RRLogger();

private:
	void openBag();
	void writeCb(const boost::shared_ptr< topic_tools::ShapeShifter >& data, const std::string& topic);

	int strToInt(std::string val);
	int compareDate(std::string a, std::string b);
	std::string nameFromDate();

	ros::NodeHandle m_nh;
	rosbag::Bag m_bag;
	std::vector<ros::Subscriber> m_subscribers;
	bool m_deleteLog;
	const int m_numOfLogs;

	const uint32_t QUEUE_SIZE;
	const std::string LOG_PATH;
};


}

#endif