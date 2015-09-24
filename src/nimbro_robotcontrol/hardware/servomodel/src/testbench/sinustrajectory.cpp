// Sinus trajectory
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "sinustrajectory.h"

#include <QtCore/QDebug>

#include <math.h>

SinusTrajectory::SinusTrajectory()
 : m_frequency(0.5 / 5)
 , m_speedFactor(0)
 , m_amplitudeFactor(0)
{
	setStartAngle(0);
	setEndAngle(M_PI/2.0);

	addParameter("startAngle", -M_PI/2.0, M_PI/32.0, M_PI/2.0, "Start angle");
	addParameter("endAngle", -M_PI/2.0, M_PI/32.0, M_PI, "End angle");
	addParameter("frequency", 0.01, 0.01, 2.0, "Frequency");
	addParameter("speedFactor", -0.1, 0.0001, 0.2, "Speed factor");
	addParameter("amplitudeFactor", 0.0, 0.0001, 0.4, "Amplitude factor");
}

void SinusTrajectory::setStartAngle(double a)
{
	m_startAngle = a;
	update();
}

void SinusTrajectory::setEndAngle(double a)
{
	m_endAngle = a;
	update();
}

void SinusTrajectory::setFrequency(double f)
{
	m_frequency = f;
	changed();
}

void SinusTrajectory::setSpeedFactor(double s)
{
	m_speedFactor = s;
	changed();
}

void SinusTrajectory::setAmplitudeFactor(double a)
{
	m_amplitudeFactor = a;
	changed();
}

double SinusTrajectory::amplitude(double t) const
{
	double G = m_amplitudeFactor * 5.0;
	double k = 1.5;

	return m_amplitude
		- G / (1.0 + exp(-k*G*(t-15))*(G/0.001 - 1))
// 		+ G / (1.0 + exp(-k*G*(t-17.5))*(G/0.001 - 1))
	;
}

double SinusTrajectory::frequency(double t) const
{
	double G = m_speedFactor * 5.0;
	double k = 1.0;

	return m_frequency + G / (1.0 + exp(-k*G*(t-10.0))*(G/0.001 - 1));
}

double SinusTrajectory::position(double time) const
{
	return amplitude(time) * sin(2.0 * M_PI * frequency(time) * time + m_phase) + m_offset;
}

double SinusTrajectory::endTime() const
{
// 	return 2.0 / m_frequency;
	return 35.0;
}

void SinusTrajectory::update()
{
	m_amplitude = fabs(m_endAngle - m_startAngle);

	if(m_endAngle > m_startAngle)
	{
		m_offset = m_startAngle + m_amplitude;
		m_phase = -M_PI/2.0;
	}
	else
	{
		m_offset = m_endAngle;
		m_phase = M_PI/2.0;
	}
// 	m_phase = 0;

// 	m_offset = 0;

	changed();
}
