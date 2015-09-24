#ifndef MOTION_H
#define MOTION_H

#include <RobotControl/Action.h>
#include <RobotModel/RobotModel.h>
#include "Config.h"
#include "State.h"

namespace indep_cpg_gait
{
	class Motion
	{
	public:
		Motion();
	~Motion();

	Action actuate();
		virtual void init();
		virtual void update();
		virtual void reset();

	protected:
		virtual HeadPose headFunction();
		virtual TrunkPose trunkFunction();
		virtual ArmPose armFunction(double armSign);
		virtual LegPose legFunction(double legSign);
		virtual AbstractArmPose abstractArmFunction(double armSign);
		virtual AbstractLegPose abstractLegFunction(double legSign);
		virtual InverseArmPose inverseArmFunction(double armSign);
		virtual InverseLegPose inverseLegFunction(double legSign);

	};
}
#endif
