// Chirp trajectory
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CHIRPTRAJECTORY_H
#define CHIRPTRAJECTORY_H

#include "positiontrajectory.h"

class ChirpTrajectory : public PositionTrajectory
{
Q_OBJECT
public:
	virtual double position(double time) const;
	virtual double endTime() const;
private:
	double m_startAngle;
	double m_endAngle;
	double m_amplitude;
	double m_phase;
	double m_frequency;
	double m_offset;

	void update();
};

#endif
