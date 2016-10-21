// Created on: June 9, 2016
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#pragma once
#include <math.h>

class Interpolator
{
private:
	double lowX;
	double highX;
	double queryX;
public:

	inline Interpolator(double lowX, double highX, double queryX = 0) :
			lowX(lowX), highX(highX), queryX(queryX)
	{

	}

	inline double Interpolate(double lowX, double lowY, double highX,
			double highY, double queryX)
	{
		return (lowY + (highY - lowY) * (queryX - lowX) / (highX - lowX));
	}

	inline double Interpolate(double lowY, double highY, double queryX)
	{
		return Interpolate(lowX, lowY, highX, highY, queryX);
	}

	inline double Interpolate(double lowY, double highY)
	{
		return Interpolate(lowX, lowY, highX, highY, queryX);
	}

	inline virtual ~Interpolator()
	{

	}
};
