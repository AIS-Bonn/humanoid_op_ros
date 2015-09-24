#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include "Percept.h"
#include "Action.h"
#include "RobotModel/RobotModel.h"
#include "MotionLayer/MotionLayer.h"
// #include "LimpFilter.h"

namespace indep_cpg_gait
{
	class RobotControl
	{
	public:

// 		LimpFilter comFilter;
// 		LimpFilter trunkAngleFilter;
// 		LimpFilter soleAngleFilter;
	RobotModel robotModel;
		MotionLayer motionLayer;

	double lastUpdateTimestamp;
	double lastStartTimestamp;

	RobotControl();
	~RobotControl(){};

	public:

	void init();

	void sense(Percept& percept);
	Pose act();
	void learn();
	};
}
#endif
