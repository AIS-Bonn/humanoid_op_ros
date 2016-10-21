//VisoinRate.hpp
// Created on: May 21, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#define BOOST_ASIO_DISABLE_STD_CHRONO
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/chrono.hpp>
#include <iostream>
#define SECONDTONANO  1000000000
using namespace std;
/**
* @ingroup VisionModule
*
* @brief For fixing the loop rate
**/
class VisionRate
{
public:
	VisionRate(double rate,bool _steay=true);
	virtual ~VisionRate();
	void sleep();
	void Destroy();
private:
	bool destroyed;
	boost::asio::io_service io_service;
	boost::asio::steady_timer timer;
	boost::asio::deadline_timer timer2;
	int timeTowait;
	bool steady;
};
