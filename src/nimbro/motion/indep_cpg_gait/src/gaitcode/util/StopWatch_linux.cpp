#include "StopWatch.h"
#include <QDebug>

StopWatch::StopWatch()
{
	// High precision tick counters for real time measurement.
	clock_gettime(CLOCK_MONOTONIC, &m_start);
	m_lastRestart = m_start;
}

// Resets the elapsed time to 0.
void StopWatch::restart()
{
	clock_gettime(CLOCK_MONOTONIC, &m_lastRestart);
}

inline double diffToDouble(const timespec& now, const timespec& last)
{
	return ((double)(now.tv_sec - last.tv_sec)) + ((double)now.tv_nsec - (double)last.tv_nsec) / 1000.0 / 1000.0 / 1000.0;
}

// Returns a time stamp expressed in seconds since the last restart.
double StopWatch::elapsedTime()
{
	timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	return diffToDouble(now, m_lastRestart);
}

// Returns a time stamp expressed in seconds since program start.
double StopWatch::programTime()
{
	timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	return diffToDouble(now, m_start);
}

// Returns a system timestamp expressed in seconds since I don't know when.
double StopWatch::systemTime()
{
	timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	return ((double)now.tv_sec) + ((double)now.tv_nsec) / 1000.0 / 1000.0 / 1000.0;
}

// Returns a time stamp expressed in seconds since program start.
double StopWatch::time()
{
	timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	return diffToDouble(now, m_start);
}

void StopWatch::pause()
{
}

void StopWatch::resume()
{
}
