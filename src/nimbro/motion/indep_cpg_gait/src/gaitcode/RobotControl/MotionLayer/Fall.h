#ifndef FALL_H
#define FALL_H

#include "Motion.h"

namespace indep_cpg_gait
{
	class Fall : public Motion
	{
	public:
		Fall();
	~Fall(){};

	protected:
		HeadPose headFunction();
		AbstractArmPose abstractArmFunction(double armSign);
		AbstractLegPose abstractLegFunction(double legSign);
	};
}
#endif
