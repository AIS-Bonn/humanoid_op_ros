#ifndef DYNAMICGAIT_H
#define DYNAMICGAIT_H

#include "Motion.h"

namespace indep_cpg_gait
{
	class DynamicGait : public Motion
	{
	public:
		double firstStepTimer;
		Vec3f gcvUserTarget;
		Vec3f gcvTarget;
		Vec3f gcv;
		Vec3f lastGCV;
		Vec3f gcvAcc;
		Vec3f effectiveGCV;
		double gaitFrequency;
		double lastGaitPhase;
		double gaitPhase;
		double expectedEnergy;
		double currentXOffsetOffset;
		double fusedAngle_sag;

		DynamicGait();
		~DynamicGait(){};

		void init();
		void update();
		void reset();

		enum Leg
		{
			RIGHT_LEG,
			LEFT_LEG
		};

		double supportCoefficient(Leg leg) const;

	protected:
		HeadPose headFunction();
		InverseLegPose inverseLegFunction(double legSign);
		InverseArmPose inverseArmFunction(double armSign);
		AbstractLegPose abstractLegFunction(double legSign);
		AbstractArmPose abstractArmFunction(double armSign);
		ArmPose armFunction(double armSign);

	};
}
#endif
