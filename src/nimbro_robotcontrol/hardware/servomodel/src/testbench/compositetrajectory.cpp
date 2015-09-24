// Composite trajectory
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "compositetrajectory.h"

#include <math.h>
#include <boost/concept_check.hpp>

const double STEP_PATTERN_LENGTH = 6.0 + 5.0 * 0.9;
const double NUM_STEP_PATTERNS = 2.0;

const double TIME_WAIT = 1.0;

const double KICK_DRAWBACK_HIP = 50.0 * M_PI/180.0;
const double KICK_DRAWBACK_KNEE = 45.0 * M_PI/180.0;
const double KICK_DEST_HIP = -60.0 * M_PI/180.0;
const double KICK_DEST_KNEE = 0;
const double KICK_LENGTH = 0.9;
const double KICK_STOP_LENGTH = 0.4;
const double KICK_DRAWBACK_LEN = 2.0;

const double NUM_KICK_PATTERNS = 5.0;

const int ID_KNEE = 14;
const int ID_HIP = 12;

double sinFromTo(double from, double to, double freq, double time)
{
	const double AMPLITUDE = to-from;

	return 0.5 * AMPLITUDE * sin(freq * 2.0 * M_PI * time - M_PI/2.0) + 0.5*AMPLITUDE + from;
}

double CompositeTrajectory::endTime() const
{
	return NUM_STEP_PATTERNS * STEP_PATTERN_LENGTH + TIME_WAIT
		+ NUM_KICK_PATTERNS * (KICK_LENGTH + KICK_STOP_LENGTH + KICK_DRAWBACK_LEN);
}

double stepPattern(int id, double time)
{
	if(time < 6.0)
	{
		const double ANGLE = 60.0 * M_PI / 180.0;
		const double FREQ = 0.5;

		switch(id)
		{
			case ID_HIP: return sinFromTo(0, -ANGLE, FREQ, time);
			case ID_KNEE: return sinFromTo(0, 2.0*ANGLE, FREQ, time);
		}
	}
	else
	{
		const double ANGLE = 30.0 * M_PI / 180.0;
		const double FREQ = 1.0 / 0.9;
		time -= 6.0;
		switch(id)
		{
			case ID_HIP: return sinFromTo(0, -ANGLE, FREQ, time);
			case ID_KNEE: return sinFromTo(0, 2.0*ANGLE, FREQ, time);
		}
	}

	return 0.0;
}

double kickPattern(int id, double time)
{
	double FREQ = 1.0 / (KICK_LENGTH - 0.25) / 2.0;
	double hip_freq = (time / KICK_LENGTH) * FREQ;

	double amplitudeFactor = 0.5 * (KICK_DRAWBACK_HIP / (KICK_LENGTH/2));
	double amplitude = amplitudeFactor * time * time;

	if(time <= KICK_LENGTH)
	{
		switch(id)
		{
			case ID_HIP: return amplitude * sin(2.0 * M_PI * hip_freq * time);
			case ID_KNEE:
			{
				double hipZeroTime = sqrt(0.5 / FREQ * KICK_LENGTH);

				if(time > hipZeroTime)
					return 0;

				return sinFromTo(KICK_DEST_KNEE, KICK_DRAWBACK_KNEE, 1.0 / hipZeroTime, time);
			}
		}
	}
	time -= KICK_LENGTH;

	double endPos = kickPattern(ID_HIP, KICK_LENGTH);
	double endVel = 2.0 * amplitudeFactor * KICK_LENGTH * sin(2.0 * M_PI * FREQ * KICK_LENGTH)
		+ 4.0 * M_PI * FREQ * amplitudeFactor * KICK_LENGTH * KICK_LENGTH * cos(2.0 * M_PI * FREQ * KICK_LENGTH);

	double stopPos = endPos + 0.5 * endVel * KICK_STOP_LENGTH;

	if(time <= KICK_STOP_LENGTH)
	{
		switch(id)
		{
			case ID_HIP:
			{
				return endPos + endVel * time - 0.5 * endVel / KICK_STOP_LENGTH * time*time;
			}
			case ID_KNEE:
				return 0;
		}
	}
	time -= KICK_STOP_LENGTH;

	double k = stopPos;
	switch(id)
	{
		case ID_HIP: return k - k/KICK_DRAWBACK_LEN * time;
		case ID_KNEE: return 0;
	}

	return 0;
}

double CompositeTrajectory::position(int id, double time) const
{
	if(id != ID_HIP && id != ID_KNEE)
		return 0.0;

	if(time < NUM_STEP_PATTERNS * STEP_PATTERN_LENGTH)
	{
		double ptime = time;
		while(ptime >= STEP_PATTERN_LENGTH)
			ptime -= STEP_PATTERN_LENGTH;

		double pos = stepPattern(id, ptime);

		if(id == ID_HIP)
		{
			const double ANGLE = 70.0 * M_PI / 180.0;
			const double FREQ = 1.0 / (NUM_STEP_PATTERNS * STEP_PATTERN_LENGTH);
			pos += 0.5 * ANGLE * sin(FREQ * 2.0 * M_PI * time + M_PI/2.0) - 0.5*ANGLE;
		}

		return pos;
	}
	time -= NUM_STEP_PATTERNS*STEP_PATTERN_LENGTH;

	if(time < TIME_WAIT)
		return 0;
	time -= TIME_WAIT;

	double ptime = time;
	while(ptime >= KICK_LENGTH + KICK_STOP_LENGTH + KICK_DRAWBACK_LEN)
		ptime -= KICK_LENGTH + KICK_STOP_LENGTH + KICK_DRAWBACK_LEN;
	return kickPattern(id, ptime);
}

