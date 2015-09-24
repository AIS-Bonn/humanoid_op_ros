// Position-based trajectory
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef POSITIONTRAJECTORY_H
#define POSITIONTRAJECTORY_H

#include "trajectory.h"

class PositionTrajectory : public Trajectory
{
public:
	PositionTrajectory(double h = 0.005);

	virtual double velocity(double time) const;
	virtual double acceleration(double time) const;
private:
	double m_h;
};

#endif
