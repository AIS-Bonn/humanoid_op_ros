#ifndef PERCEPT_H_
#define PERCEPT_H_

#include "Action.h"
#include "util/Vec2f.h"

// The percept struct contains the raw sensor data received from the robot / simulation.

namespace indep_cpg_gait
{
	struct Percept
	{
		Vec2f fusedAngle;
		Vec2f DfusedAngle;
		Pose pose;
	};
}
#endif // PERCEPT_H_
