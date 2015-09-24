#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <QList>
#include <QHash>
#include "RobotControl/Action.h"

namespace indep_cpg_gait
{
	struct Config
	{
		double Cy;
		double dy;
		double alpha;
		double delta;
		double omega;
		double D;
		double Z;
		double sigma;
		double mu;
		double energy;
		double stepDelay;
		double intensity;
		double stepSizeFactorY;
		double minEnergyDev;

		Action motionInterface;

		double armOffsetX;
		double armOffsetY;
		double armOffsetZ;
		double footOffsetX;
		double footOffsetY;
		double footOffsetZ;
		double footAngleX;
		double footAngleY;

		double footShiftY;

		double complianceTrunk;
		double complianceArm;
		double complianceLeg;
		double complianceAnkle;

		double gaitFrequency;
		double gaitFrequencySlopeY;
		double gaitFrequencySlopeX;
		double accelerationBackward;
		double accelerationForward;
		double accelerationRotational;
		double accelerationSideward;
		double decelerationFactor;
		double GCVNormP;
		double footOffsetXAccSlope;
		double stableSagAngleFront;
		double stableSagAngleSlope;

		double pushHeight;
		double pushHeightSlope;
		double stepHeight;
		double stepHeightSlope;
		double swingStartTiming;
		double swingStopTiming;
		double armSwing;
		double armSwingSlope;
		double stepLengthYSlope;
		double stepLengthXPushout;
		double stepLengthYPushout;
		double stepLengthZPushout;
		double stepLengthZSlope;
		double stepLengthZVPushout;
		double stepLengthXSlopeFwd;
		double stepLengthXSlopeBwd;
		double lateralHipSwing;
		double lateralHipSwingSlope;
		double rotationalHipSwingSlope;
		double firstStepDuration;
		double firstStepHipSwing;
		double stoppingVelocity;
		double tiltSpeedBwd;
		double tiltSpeedFwd;
		double tiltRotLean;
		double supportSlope;

		double dampingMinimum;
		double dampingMaximum;
		double dampingLateralSlope;
		double dampingSagittalSlope;
		double dampingSlopeArmX;
		double dampingSlopeArmY;

		double fallProtectionRelaxAngle;
		double systemIterationTime;
		double fusedAngleCorrectionX;
		double fusedAngleCorrectionY;

		double solverIterations;
		double simulationFrequency;

		double balanceOffset;

		Config();
		~Config(){};

		void init();
		void save(QString robotName);
		void load(QString robotName);

		double& operator[](int i);
		double& operator()(int i);
		double& operator[](QString key);
		double& operator()(QString key);

	private:

		// Registers a member variable for index based access.
		void registerMember(QString name, double* member, double sliderFactor)
		{
			memberNames << name;
			memberOffsets[name] = (unsigned char*)member - (unsigned char*)this;
			sliderFactors[name] = sliderFactor;
		}

	QHash<QString, unsigned long int> memberOffsets;

	public:
	QList<QString> memberNames; // Contains the names of the members in the right order.
		QHash<QString, double> sliderFactors; // The factors of all explicitely registered config variables.
	};

	extern Config config;
}
#endif /* CONFIGURATION_H_ */
