// Slope limiting
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROBOTCONTROL_SLOPELIMITED_H
#define ROBOTCONTROL_SLOPELIMITED_H

#include <string>

namespace robotcontrol
{

template<class T>
T slopeLimited(T currentValue, T newValue, T slope)
{
	T delta = newValue - currentValue;

	if(delta > slope)
		delta = slope;
	else if(delta < -slope)
		delta = -slope;

	return currentValue + delta;
}

}

#endif
