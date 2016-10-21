//VisionTimer.hpp
// Created on: Mar 29, 2016
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <ctime>
#include <stdio.h>
#include <iostream>
#include <sys/stat.h>
/**
* @ingroup VisionModule
*
* @brief To set a timer and checked if the timeout exceeded
**/
class VisionTimer
{
public:
	double timeoutSec;
	VisionTimer(double timeoutSec=10.0):timeoutSec(timeoutSec)
	{
		clock_gettime(CLOCK_REALTIME, &beg_);
		beg_.tv_sec -= timeoutSec + 1;
	}

	double elapsed()
	{
		clock_gettime(CLOCK_REALTIME, &end_);
		return end_.tv_sec - beg_.tv_sec
				+ (end_.tv_nsec - beg_.tv_nsec) / 1000000000.;
	}

	void reset()
	{
		clock_gettime(CLOCK_REALTIME, &beg_);
	}

	bool IsDead(float timeout)
	{
		return (elapsed() > timeout);
	}

	bool IsDead()
	{
		return (elapsed() > timeoutSec);
	}

private:
	timespec beg_, end_;
};
