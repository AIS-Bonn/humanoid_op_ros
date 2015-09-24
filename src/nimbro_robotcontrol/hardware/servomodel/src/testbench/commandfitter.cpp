// Fits servo model using least squares
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "commandfitter.h"
#include "trajectory.h"
#include <servomodel/servocommandgenerator.h>
#include "valuelogger.h"

#include <float.h>

#include <Eigen/SVD>

using namespace Eigen;

CommandFitter::CommandFitter(ServoCommandGenerator* gen)
 : m_gen(gen)
 , m_min(DBL_MIN)
 , m_max(DBL_MAX)
{
}

CommandFitter::~CommandFitter()
{
}

void CommandFitter::setCommandLimits(double min, double max)
{
	m_min = min;
	m_max = max;
}

double CommandFitter::fit(const ValueLogger& learnedCmd, const ValueLogger& outsideTorques, const Trajectory& traj)
{
	Matrix<double, Dynamic, 4> A(learnedCmd.count(), 4);
	Matrix<double, Dynamic, 1> b(learnedCmd.count());

	int i = 0;
	for(ValueLogger::const_iterator it = learnedCmd.begin(); it.isValid(); ++it)
	{
		// HACK: Ignore the first 0.5s because the deviation is going to be
		// quite high here
		if(it.time() < 0.5)
			continue;

		if(it.value() < m_min || it.value() > m_max)
			continue;

		double time = it.time() + m_gen->latency();

		double pos = traj.position(time);
		double vel = traj.velocity(time);
		double acc = traj.acceleration(time);

		b(i) = (it.value() - pos) / m_gen->currentFactor();

		A.row(i) = m_gen->commandPartsFor(
			pos, vel, acc,
			outsideTorques.valueAtTime(time)
		).transpose();

		++i;
	}

	A = A.topRows(i);
	b = b.head(i);

	MatrixXd coefficients = A.jacobiSvd(ComputeFullU | ComputeFullV).solve(b);

	m_gen->setCoefficients(coefficients);

	// return sum of square errors
	return (A*coefficients - b).norm();
}
