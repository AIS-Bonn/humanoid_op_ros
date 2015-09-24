// Part of a composite trajectory
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "trajectorypart.h"
#include "compositetrajectory.h"

TrajectoryPart::TrajectoryPart(CompositeTrajectory* traj, int id)
 : PositionTrajectory()
 , m_traj(traj)
 , m_id(id)
{
}

TrajectoryPart::~TrajectoryPart()
{
}

double TrajectoryPart::endTime() const
{
	return m_traj->endTime();
}

double TrajectoryPart::position(double time) const
{
	return m_traj->position(m_id, time);
}

