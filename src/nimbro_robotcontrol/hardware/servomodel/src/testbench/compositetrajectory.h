// Composite trajectory
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef COMPOSITETRAJECTORY_H
#define COMPOSITETRAJECTORY_H

class CompositeTrajectory
{
public:
	virtual double position(int id, double time) const;
	virtual double endTime() const;
private:
};

#endif
