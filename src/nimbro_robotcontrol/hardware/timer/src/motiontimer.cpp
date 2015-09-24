#include <time.h>
#include <errno.h>
#include <sys/timerfd.h>

#include <ros/console.h>

#include "motiontimer.h"


const double SECONDTONANO = 1000000000;

MotionTimer::MotionTimer(double rate)
#ifdef MONITOR
 : m_monitor("timerfd")
#endif
{
	m_timerfd = timerfd_create(CLOCK_MONOTONIC, 0);

	itimerspec timespc;
	memset(&timespc, 0, sizeof(timespc));

	timespc.it_interval.tv_sec = 0;
	timespc.it_interval.tv_nsec = (long)(rate * SECONDTONANO);
	timespc.it_value.tv_sec = 0;
	timespc.it_value.tv_nsec = 1;

	if(timerfd_settime(m_timerfd, 0, &timespc, 0) != 0)
	{
		perror("timerfd_settime");
		ROS_ERROR("Could not setup timer");
		abort();
	}
}

MotionTimer::~MotionTimer()
{
	close(m_timerfd);
}


long unsigned int MotionTimer::sleep()
{
#ifdef MONITOR
	m_monitor.setBeginningTimestamp(ros::Time::now());
#endif
	int result;
	uint64_t expirations;

	do
		result = read(m_timerfd, &expirations, sizeof(expirations));
	while (result == EINTR);

	if (result < 0)
	{
		perror("read");
		ROS_ERROR("Could not read from timer");
		return -1;
	}

#ifdef MONITOR
	m_monitor.setEndingTimestamp(ros::Time::now());
#endif

	return expirations;
}





////////////////////////////////////////////////////////////////////////////////////////////////

/* old timer stuff
 v o*id MotionTimer::start()
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