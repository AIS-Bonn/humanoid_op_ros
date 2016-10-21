#ifndef TIMER_H
#define TIMER_H

// #define MONITOR

#include <cmath>
#include <time.h>
#include <monitor/timermonitor.h>

class MotionTimer
{
public:
	MotionTimer(double rate);
	virtual ~MotionTimer();

	long unsigned int sleep();

	int cyclesForTime(double interval) const;

private:
	int m_timerfd;
	double m_rate;

#ifdef MONITOR
	TimerMonitor m_monitor;
#endif
};

#endif