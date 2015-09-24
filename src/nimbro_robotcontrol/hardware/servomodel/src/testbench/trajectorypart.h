// Part of a composite trajectory
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TRAJECTORYPART_H
#define TRAJECTORYPART_H

#include "positiontrajectory.h"

class CompositeTrajectory;

class TrajectoryPart : public PositionTrajectory
{
Q_OBJECT
public:
	TrajectoryPart(CompositeTrajectory* traj, int id);
	virtual ~TrajectoryPart();

	virtual double position(double time) const;
	virtual double endTime() const;
private:
	CompositeTrajectory* m_traj;
	int m_id;
};

#endif
