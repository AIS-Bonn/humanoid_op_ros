#include "nanosleeptimer.h"

#include <errno.h>

const int SECONDTONANO = 1000000000;


NanosleepTimer::NanosleepTimer(double rate)
 : m_rate(rate * SECONDTONANO)
#ifdef MONITOR
 , m_monitor("nanosleeptimer")
#endif
{
	clock_gettime(CLOCK_MONOTONIC, &m_nextTime);
}

NanosleepTimer::~NanosleepTimer()
{}

long unsigned int NanosleepTimer::sleep()
{
#ifdef MONITOR
	m_monitor.setBeginningTimestamp(ros::Time::now());
#endif
	clock_gettime(CLOCK_MONOTONIC, &m_currentTime);

	do
	{
		m_nextTime.tv_sec += (m_nextTime.tv_nsec + m_rate) / SECONDTONANO;
		m_nextTime.tv_nsec = (int)(m_nextTime.tv_nsec + m_rate) % 1000000000;
	}
	while (m_currentTime.tv_sec > m_nextTime.tv_sec
		|| (m_currentTime.tv_sec == m_nextTime.tv_sec
		&& m_currentTime.tv_nsec > m_nextTime.tv_nsec));

	int slErr;
	while(true)
	{
		slErr = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &m_nextTime, NULL);
		if (slErr != EINTR)
			break;
	}
#ifdef MONITOR
	m_monitor.setEndingTimestamp(ros::Time::now());
#endif
	return 0;
}




////////////////////////////////////////////////////////////////////////////////////////////////

/* old timer stuff
 v o**id MotionTimer::start()
 {
	 const double NANOTOSECOND = 1000000000.0;
	 const double NANOTOMILLI = 1000000.0;

	 struct timespec next_time;
	 struct timespec current_time;
	 clock_gettime(CLOCK_MONOTONIC, &next_time);


	 while(1)
	 {
		 clock_gettime(CLOCK_MONOTONIC, &current_time);

		 do
		 {
			 next_time.tv_sec += (next_time.tv_nsec + m_rate * 1000000) / 1000000000;
			 next_time.tv_nsec = (int)(next_time.tv_nsec + m_rate * 1000000) % 1000000000;
			 }
			 while(current_time.tv_sec > next_time.tv_sec
				 || (current_time.tv_sec == next_time.tv_sec
				 && current_time.tv_nsec > next_time.tv_nsec));
			 int sleep_error;
			 while(1)
			 {
				 sleep_error = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
				 if (sleep_error != EINTR)
					 break;
			 }

			 }
			 */