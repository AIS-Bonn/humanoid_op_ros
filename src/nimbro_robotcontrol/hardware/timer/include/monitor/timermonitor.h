//Monitors deviation of timer intervall
//Author: Sebastian Schüller

#ifndef TIMERMONITOR_H
#define TIMERMONITOR_H

#include <fstream>
#include <ros/time.h>


class TimerMonitor
{
public:
	TimerMonitor(const std::string& fileName);
	~TimerMonitor();

	void setBeginningTimestamp(ros::Time timeStamp);
	void setEndingTimestamp(ros::Time timeStamp);

private:
	ros::Time m_beginningTimestamp;
	ros::Time m_endingTimestamp;

	ros::Duration m_sleepDuration;
	ros::Duration m_workDuration;
	ros::Duration m_interval;

	long unsigned int m_cycles;

	std::ofstream m_logFile;
	std::ofstream m_plotFile;

	void print();
};

#endif