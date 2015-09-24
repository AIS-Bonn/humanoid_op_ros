#ifndef HALT_H
#define HALT_H

#include "Motion.h"

namespace indep_cpg_gait
{
	class Halt : public Motion
	{
	public:
		Halt();
	~Halt(){};

	void update();

	protected:
		HeadPose headFunction();
		InverseArmPose inverseArmFunction(double armSign);
		InverseLegPose inverseLegFunction(double legSign);
		ArmPose armFunction(double armSign);
	};
}
#endif
