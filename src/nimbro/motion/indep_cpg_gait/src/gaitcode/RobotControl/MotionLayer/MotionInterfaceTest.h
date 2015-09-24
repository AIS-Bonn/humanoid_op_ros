#ifndef MOTIONINTERFACETEST_H_
#define MOTIONINTERFACETEST_H_

#include "Motion.h"
#include "Config.h"
#include "RobotModel/RobotModel.h"

namespace indep_cpg_gait
{
	class MotionInterfaceTest : public Motion
	{
	public:
		MotionInterfaceTest();
	~MotionInterfaceTest(){};

	bool configChangedIn();

	protected:
		HeadPose headFunction();
		TrunkPose trunkFunction();
		ArmPose armFunction(double armSign);
		LegPose legFunction(double legSign);
		AbstractArmPose abstractArmFunction(double armSign);
		AbstractLegPose abstractLegFunction(double legSign);
		InverseArmPose inverseArmFunction(double armSign);
		InverseLegPose inverseLegFunction(double legSign);

	private:
		Config savedConfig;
		RobotModel robotModel;
	};
}
#endif
