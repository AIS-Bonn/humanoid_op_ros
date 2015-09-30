//VisionRate.cpp
// Created on: May 21, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <vision_module/Tools/VisoinRate.hpp>

VisionRate::VisionRate(double rate, bool _steady) :
		timer(io_service), timer2(io_service), steady(_steady)
{
	timeTowait = 1000000. / rate;
	if (steady)
	{
		timer.expires_from_now(boost::chrono::microseconds(timeTowait));
	}
	else
	{
		timer2.expires_from_now(boost::posix_time::microseconds(timeTowait));
	}
}

VisionRate::~VisionRate()
{
}

void VisionRate::sleep()
{

	if (steady)
	{
		timer.wait();
		timer.expires_from_now(boost::chrono::microseconds(timeTowait));
	}
	else
	{
		timer2.wait();
		timer2.expires_from_now(boost::posix_time::microseconds(timeTowait));
	}
}

