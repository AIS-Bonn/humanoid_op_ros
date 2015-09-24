

#ifndef NANOSLEEPTIMER_H
#define NANOSLEEPTIMER_H

#define MONITOR

#include <time.h>
#include <monitor/timermonitor.h>

class NanosleepTimer
{
public:
	NanosleepTimer(double rate);
	virtual ~NanosleepTimer();

	long unsigned int sleep();

private:
	double m_rate;

	struct timespec m_currentTime;
	struct timespec m_nextTime;
#ifdef MONITOR
	TimerMonitor m_monitor;
#endif

};

#endif