#include "MotionInterfaceTest.h"

namespace indep_cpg_gait
{
	MotionInterfaceTest::MotionInterfaceTest()
	{
	}

	bool MotionInterfaceTest::configChangedIn()
	{
		// Examine the new config state and find out in which layer the config changed.
		// Inverse, abstract or joint target?

		bool changed = false;

	//	if (savedConfig.motionInterface.pose != config.motionInterface.pose)
	//	{
	//		robotModel.setPose(config.motionInterface.pose);
	//		config.motionInterface = robotModel.getAction();
	//		changed = true;
	//	}
	//
	//	else if (savedConfig.motionInterface.abstractPose != config.motionInterface.abstractPose)
	//	{
	//		robotModel.setAbstractPose(config.motionInterface.abstractPose);
	//		config.motionInterface = robotModel.getAction();
	//		changed = true;
	//	}
	//
	//	else if (savedConfig.motionInterface.inversePose != config.motionInterface.inversePose)
	//	{
	//		robotModel.setInversePose(config.motionInterface.inversePose);
	//		config.motionInterface = robotModel.getAction();
	//		changed = true;
	//	}

		if (changed)
			savedConfig = config;

		return changed;
	}

	HeadPose MotionInterfaceTest::headFunction()
	{
		HeadPose hp;
		hp = config.motionInterface.pose.headPose;
		return hp;
	}

	TrunkPose MotionInterfaceTest::trunkFunction()
	{
		TrunkPose tp;
		tp = config.motionInterface.pose.trunkPose;
		return tp;
	}

	ArmPose MotionInterfaceTest::armFunction(double armSign)
	{
		ArmPose ap;

		if (armSign == 1)
		{
			ap = config.motionInterface.pose.rightArmPose;
		}
		else
		{
			ap = config.motionInterface.pose.leftArmPose;
		}

		return ap;
	}

	LegPose MotionInterfaceTest::legFunction(double legSign)
	{
		LegPose lp;

		if (legSign == 1)
		{
			lp = config.motionInterface.pose.rightLegPose;
		}
		else
		{
			lp = config.motionInterface.pose.leftLegPose;
		}

		return lp;
	}

	AbstractArmPose MotionInterfaceTest::abstractArmFunction(double armSign)
	{
		AbstractArmPose ap;

		if (armSign == 1)
		{
			ap = config.motionInterface.abstractPose.rightArmPose;
		}
		else
		{
			ap = config.motionInterface.abstractPose.leftArmPose;
		}

		return ap;
	}

	AbstractLegPose MotionInterfaceTest::abstractLegFunction(double legSign)
	{
		AbstractLegPose lp;

		if (legSign == 1)
		{
			lp = config.motionInterface.abstractPose.rightLegPose;
		}
		else
		{
			lp = config.motionInterface.abstractPose.leftLegPose;
		}

		return lp;
	}

	InverseArmPose MotionInterfaceTest::inverseArmFunction(double armSign)
	{
		InverseArmPose ap;

		if (armSign == 1)
		{
			ap = config.motionInterface.inversePose.rightArmPose;
		}
		else
		{
			ap = config.motionInterface.inversePose.leftArmPose;
		}

		return ap;
	}

	InverseLegPose MotionInterfaceTest::inverseLegFunction(double legSign)
	{
		InverseLegPose lp;

		if (legSign == 1)
		{
			lp = config.motionInterface.inversePose.rightLegPose;
		}
		else
		{
			lp = config.motionInterface.inversePose.leftLegPose;
		}

		return lp;
	}
}


