#ifndef STOPWATCH_H_
#define STOPWATCH_H_

#ifdef WIN32
#include "windows.h"
#else
#include <time.h>
#endif

	class StopWatch
	{
	#ifdef WIN32
		LARGE_INTEGER cpuTicksAtProgramStart;
		LARGE_INTEGER cpuTicksAtLastRestart;
	LARGE_INTEGER cpuTicksPerSecond;
	LARGE_INTEGER currentCpuTicks;
	#else
		timespec m_start;
		timespec m_lastRestart;
	#endif
		double m_offset;

	public:
		StopWatch();
		~StopWatch(){};
		void restart();
		double elapsedTime();
		double programTime();
		double systemTime();
		double time();

		void pause();
		void resume();
	};

#endif /* STOPWATCH_H_ */
