#include "Halt.h"
#include <Globals.h>
#include <boost/concept_check.hpp>

namespace indep_cpg_gait
{
	Halt::Halt()
	{
	}

	void Halt::update()
	{

	}

	HeadPose Halt::headFunction()
	{
		HeadPose hp;
		hp.neck.compliance = config.complianceTrunk;
		return hp;
	}

	InverseArmPose Halt::inverseArmFunction(double armSign)
	{
		InverseArmPose iap;
		iap.handPosition.x = config.armOffsetX;
		iap.handPosition.y = armSign*config.armOffsetY;
		iap.handPosition.z = config.armOffsetZ;
		iap.compliance = config.complianceArm;
		return iap;
	}

	InverseLegPose Halt::inverseLegFunction(double legSign)
	{
		InverseLegPose ilp;
		ilp.footPosition.x = config.footOffsetX;
		ilp.footPosition.y = legSign*config.footOffsetY + config.footShiftY;
		ilp.footPosition.z = config.footOffsetZ;
		ilp.footAngle.x = config.footAngleX;
		ilp.footAngle.y = config.footAngleY;
		ilp.compliance = config.complianceLeg;
		return ilp;
	}

	ArmPose Halt::armFunction(double armSign)
	{
		ArmPose ap;
		ap.shoulder.x = -armSign * config.armOffsetY;

		if (armSign > 0 && config.balanceOffset > 0)
			ap.shoulder.x -= config.balanceOffset;

		if (armSign < 0 && config.balanceOffset < 0)
			ap.shoulder.x -= config.balanceOffset;

		return ap;
	}
}