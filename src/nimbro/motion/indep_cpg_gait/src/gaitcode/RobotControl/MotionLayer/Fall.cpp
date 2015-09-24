#include "Fall.h"

namespace indep_cpg_gait
{
	Fall::Fall()
	{
	}

	HeadPose Fall::headFunction()
	{
		HeadPose hp;
		hp.neck = state.rxAction.pose.headPose.neck;
		hp.neck.compliance = 1.0;
		return hp;
	}

	AbstractArmPose Fall::abstractArmFunction(double armSign)
	{
		AbstractArmPose ap;
		if (armSign == 1)
		{
			ap.armAngle = state.rxAction.abstractPose.rightArmPose.armAngle;
			ap.armExtension = state.rxAction.abstractPose.rightArmPose.armExtension;
		}
		else
		{
			ap.armAngle = state.rxAction.abstractPose.leftArmPose.armAngle;
			ap.armExtension = state.rxAction.abstractPose.leftArmPose.armExtension;
		}
		ap.compliance = 1.0;
		return ap;
	}

	AbstractLegPose Fall::abstractLegFunction(double legSign)
	{
		AbstractLegPose lp;
		if (legSign == 1)
		{
			lp.legAngle = state.rxAction.abstractPose.rightLegPose.legAngle;
			lp.legExtension = state.rxAction.abstractPose.rightLegPose.legExtension;
			lp.footAngle = state.rxAction.abstractPose.rightLegPose.footAngle;
		}
		else
		{
			lp.legAngle = state.rxAction.abstractPose.leftLegPose.legAngle;
			lp.legExtension = state.rxAction.abstractPose.leftLegPose.legExtension;
			lp.footAngle = state.rxAction.abstractPose.leftLegPose.footAngle;
		}
		lp.compliance = 1.0;
		return lp;
	}

}
