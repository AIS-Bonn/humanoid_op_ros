// Fits servo model using least squares
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef COMMANDFITTER_H
#define COMMANDFITTER_H

class ServoCommandGenerator;
class ValueLogger;
class Trajectory;

class CommandFitter
{
public:
	CommandFitter(ServoCommandGenerator* gen);
	virtual ~CommandFitter();

	void setCommandLimits(double min, double max);

	double fit(
		const ValueLogger& learnedCmd,
		const ValueLogger& outsideTorques,
		const Trajectory& traj
	);
private:
	ServoCommandGenerator* m_gen;
	double m_min;
	double m_max;
};

#endif
