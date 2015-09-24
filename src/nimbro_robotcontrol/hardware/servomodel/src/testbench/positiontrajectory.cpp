// Position-based trajectory
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "positiontrajectory.h"

PositionTrajectory::PositionTrajectory(double h)
 : m_h(h)
{
}

double PositionTrajectory::velocity(double time) const
{
	return (position(time+m_h) - position(time)) / m_h;
}

double PositionTrajectory::acceleration(double time) const
{
	return (velocity(time+m_h) - velocity(time)) / m_h;
}
