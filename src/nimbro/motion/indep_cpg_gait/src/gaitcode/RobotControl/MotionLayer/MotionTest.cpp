#include "MotionTest.h"
#include "Globals.h"

namespace indep_cpg_gait
{
	MotionTest::MotionTest()
	{
		phase = 0;
		frequency = 0;
	}

	void MotionTest::init()
	{
		phase = 0;
		frequency = 0.01;
	}

	void MotionTest::reset()
	{
		phase = 0;
	}

	void MotionTest::update()
	{
		phase = picut(phase + frequency);
	}


	HeadPose MotionTest::headFunction()
	{
		HeadPose hp;
		return hp;
	}

	TrunkPose MotionTest::trunkFunction()
	{
		TrunkPose tp;
		return tp;
	}

	ArmPose MotionTest::armFunction(double armSign)
	{
		ArmPose ap;
		return ap;
	}

	LegPose MotionTest::legFunction(double legSign)
	{
		LegPose lp;
		return lp;
	}

	AbstractArmPose MotionTest::abstractArmFunction(double armSign)
	{
		AbstractArmPose ap;
		return ap;
	}

	AbstractLegPose MotionTest::abstractLegFunction(double legSign)
	{
		AbstractLegPose lp;
		return lp;
	}

	InverseArmPose MotionTest::inverseArmFunction(double armSign)
	{
		InverseArmPose ap;

		ap.handPosition.x = armSign*sin(phase);
	//	ap.handPosition.y = 0.5*sin(0.1*phase);
		ap.handPosition.z = 0.5*sin(phase*phase) + 0.5;

		return ap;
	}

	InverseLegPose MotionTest::inverseLegFunction(double legSign)
	{
		InverseLegPose lp;

		if (legSign == -1)
		{
			lp.footPosition.x = 0.5*sin(phase) + 0.5;
			lp.footPosition.y = 0.5*(0.5*sin(2.0*phase) + 0.5);
			lp.footPosition.z = 0.5*sin(phase*phase) + 0.5;
			lp.footAngle.z = 0.5*sin(phase);
		}
		return lp;
	}
}
