//Monitors deviation of timer intervall
//Author: Sebastian Schülle

#include "monitor/timermonitor.h"

#include <ros/console.h>

TimerMonitor::TimerMonitor(const std::string& fileName)
 : m_beginningTimestamp(ros::Time::now())
 , m_endingTimestamp(ros::Time::now())
 , m_cycles(0)
{
// 	m_logFile.open(fileName + ".log");
// 	if (!m_logFile.is_open())
// 		ROS_ERROR("Could not open TimerMonitor log file!");

	m_plotFile.open((fileName + ".plot").c_str());
	if (!m_plotFile.is_open())
		ROS_ERROR("Could not open TimerMonitor plot file!");
}


TimerMonitor::~TimerMonitor()
{
// 	if (m_logFile.is_open())
// 		m_logFile.close();

	if (m_plotFile.is_open())
		m_plotFile.close();
}


void TimerMonitor::setBeginningTimestamp(ros::Time timeStamp)
{
	m_beginningTimestamp = timeStamp;
	m_cycles++;
}


void TimerMonitor::setEndingTimestamp(ros::Time timeStamp)
{
	m_interval = timeStamp - m_endingTimestamp;
	m_sleepDuration = timeStamp - m_beginningTimestamp;
	m_workDuration = m_beginningTimestamp - timeStamp;

	print();
	m_endingTimestamp = timeStamp;
}

void TimerMonitor::print()
{
	m_plotFile << m_cycles << " " << m_interval.toSec() << " " << m_sleepDuration.toSec() << " " << m_workDuration.toSec() << std::endl;
}
