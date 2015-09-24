// Sinus trajectory
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef SINUSTRAJECTORY_H
#define SINUSTRAJECTORY_H

#include "positiontrajectory.h"

class SinusTrajectory : public PositionTrajectory
{
Q_OBJECT
Q_PROPERTY(double startAngle READ startAngle WRITE setStartAngle)
Q_PROPERTY(double endAngle READ endAngle WRITE setEndAngle)
Q_PROPERTY(double frequency READ frequency WRITE setFrequency)
Q_PROPERTY(double speedFactor READ speedFactor WRITE setSpeedFactor)
Q_PROPERTY(double amplitudeFactor READ amplitudeFactor WRITE setAmplitudeFactor)
public:
	SinusTrajectory();

	virtual double position(double time) const;
	virtual double endTime() const;

	void setStartAngle(double a);
	void setEndAngle(double a);
	void setFrequency(double f);
	void setSpeedFactor(double s);
	void setAmplitudeFactor(double a);

	inline double startAngle() const
	{ return m_startAngle; }
	inline double endAngle() const
	{ return m_endAngle; }
	inline double frequency() const
	{ return m_frequency; }
	inline double speedFactor() const
	{ return m_speedFactor; }
	inline double amplitudeFactor() const
	{ return m_amplitudeFactor; }
private:
	double m_startAngle;
	double m_endAngle;
	double m_amplitude;
	double m_phase;
	double m_frequency;
	double m_offset;
	double m_speedFactor;
	double m_amplitudeFactor;

	void update();
	double frequency(double t) const;
	double amplitude(double t) const;
};

#endif
