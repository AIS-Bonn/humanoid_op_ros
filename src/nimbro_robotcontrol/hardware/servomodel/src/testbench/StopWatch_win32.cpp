#include "StopWatch.h"
#include <QDebug>

StopWatch::StopWatch()
{
	// High precision tick counters for real time measurement.
	QueryPerformanceFrequency(&cpuTicksPerSecond);
	QueryPerformanceCounter(&cpuTicksAtProgramStart);
	cpuTicksAtLastRestart = cpuTicksAtProgramStart;
}

// Resets the elapsed time to 0.
void StopWatch::restart()
{
	QueryPerformanceCounter(&cpuTicksAtLastRestart);
}

// Returns a time stamp expressed in seconds since the last restart.
double StopWatch::elapsedTime()
{
	QueryPerformanceCounter(&currentCpuTicks);
	return ((double)currentCpuTicks.QuadPart - (double)cpuTicksAtLastRestart.QuadPart) / (double)cpuTicksPerSecond.QuadPart;
}

// Returns a time stamp expressed in seconds since program start.
double StopWatch::programTime()
{
	QueryPerformanceCounter(&currentCpuTicks);
	return ((double)currentCpuTicks.QuadPart - (double)cpuTicksAtProgramStart.QuadPart) / (double)cpuTicksPerSecond.QuadPart;
}

// Returns a system timestamp expressed in seconds since I don't know when.
double StopWatch::systemTime()
{
	QueryPerformanceCounter(&currentCpuTicks);
	return (double)currentCpuTicks.QuadPart / (double)cpuTicksPerSecond.QuadPart;
}

// Returns a time stamp expressed in seconds since program start.
double StopWatch::time()
{
	QueryPerformanceCounter(&currentCpuTicks);
	return ((double)currentCpuTicks.QuadPart - (double)cpuTicksAtProgramStart.QuadPart) / (double)cpuTicksPerSecond.QuadPart;
}
