#include "Motion.h"
#include <math.h>

// Motion. A base class for all motions in the motion layer.
// Derive your own motion from this and override the update() function
// and the motion generators for the head, trunk, arms and legs on joint,
// abstract or inverse kinematics level. The update() function gets called
// once per iteration. Use it to maintain internal states of your motion.
// The motion generators are called once per iteration too, the arm and
// leg functions once with 1 and once with -1 as a parameter for right or
// left. You can mix motion commands on different levels, just be aware
// that in the end the motion signals from all levels are summed up.

namespace indep_cpg_gait
{
	Motion::Motion()
	{
	}

	Motion::~Motion()
	{
	}

	void Motion::init()
	{
	}

	void Motion::update()
	{
	}

	void Motion::reset()
	{
	}

	// Calls all the body part functions with both signs and constructs a whole body abstract pose.
	Action Motion::actuate()
	{
		Action action;
		action.pose.headPose = headFunction();
		action.pose.trunkPose = trunkFunction();
		action.pose.leftArmPose = armFunction(-1);
		action.pose.rightArmPose = armFunction(1);
		action.pose.leftLegPose = legFunction(-1);
		action.pose.rightLegPose = legFunction(1);
		action.abstractPose.leftArmPose = abstractArmFunction(-1);
		action.abstractPose.rightArmPose = abstractArmFunction(1);
		action.abstractPose.leftLegPose = abstractLegFunction(-1);
		action.abstractPose.rightLegPose = abstractLegFunction(1);
		action.inversePose.leftArmPose = inverseArmFunction(-1);
		action.inversePose.rightArmPose = inverseArmFunction(1);
		action.inversePose.leftLegPose = inverseLegFunction(-1);
		action.inversePose.rightLegPose = inverseLegFunction(1);

		return action;
	}


	HeadPose Motion::headFunction()
	{
		HeadPose hp;
		return hp;
	}

	TrunkPose Motion::trunkFunction()
	{
		TrunkPose tp;
		return tp;
	}

	ArmPose Motion::armFunction(double armSign)
	{
		ArmPose ap;
		return ap;
	}

	LegPose Motion::legFunction(double legSign)
	{
		LegPose lp;
		return lp;
	}

	AbstractArmPose Motion::abstractArmFunction(double armSign)
	{
		AbstractArmPose ap;
		return ap;
	}

	AbstractLegPose Motion::abstractLegFunction(double legSign)
	{
		AbstractLegPose lp;
		return lp;
	}

	InverseArmPose Motion::inverseArmFunction(double armSign)
	{
		InverseArmPose ap;
		return ap;
	}

	InverseLegPose Motion::inverseLegFunction(double legSign)
	{
		InverseLegPose lp;
		return lp;
	}


}