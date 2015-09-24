#ifndef MOTIONTEST_H_
#define MOTIONTEST_H_

#include "Motion.h"

namespace indep_cpg_gait
{
	class MotionTest : public Motion
	{
	public:
		MotionTest();
	~MotionTest(){};

	void init();
	void update();
	void reset();

	private:
	double phase;
	double frequency;

	protected:
		HeadPose headFunction();
		TrunkPose trunkFunction();
		ArmPose armFunction(double armSign);
		LegPose legFunction(double legSign);
		AbstractArmPose abstractArmFunction(double armSign);
		AbstractLegPose abstractLegFunction(double legSign);
		InverseArmPose inverseArmFunction(double armSign);
		InverseLegPose inverseLegFunction(double legSign);
	};
}
#endif
