#include "Config.h"
#include "Globals.h"
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QDebug>

// The global config object contains application wide configuration variables.
// Basically, the config object is a globally accessible struct with public
// read and write access for everyone (no thread safe!). Yes, even if this
// violates common programming paradigms, it focuses on simplicity of use.
// Just include the config header anywhere and use config.blabla to access a
// configuration parameter. Typically you will not want to write any parameters
// during runtime, only the config slider widget wants to do so.
// Similar to the State object, the Config object provides some basic reflection
// capabilities so that config sliders can be automatically generated for the gui.
// All config variables are declared in this central place. Declare them in the
// config.h header, initialize them in the constructor and optionally register
// them in the init method if you want a slider to be created. Every registered
// config variable gets a name and a slider factor assigned that determines the
// sensitivity of the slider.
// The config object also supports save() and load() functions that serializes
// and unserializes the variable contents in a hand editable text file. The save
// and load functions take a robot name as an argument to support different config
// sets for different robots.

namespace indep_cpg_gait
{
	Config config;

	Config::Config()
	{
		Cy = 0;
		dy = 0;
		alpha = 0;
		delta = 0;
		omega = 0;
		D = 0;
		Z = 0;
		sigma = 0;
		mu = 0;
		energy = 0;
		stepDelay = 0;
		intensity = 0;
		stepSizeFactorY = 0;
		minEnergyDev = 0;

		armOffsetX = 0;
		armOffsetY = 0;
		armOffsetZ = 0;
		footOffsetX = 0;
		footOffsetY = 0;
		footOffsetZ = 0;
		footAngleX = 0;
		footAngleY = 0;

		footShiftY = 0;

		complianceTrunk = 0;
		complianceArm = 0;
		complianceLeg = 0;
		complianceAnkle = 0;

		gaitFrequency = 0;
		gaitFrequencySlopeY = 0;
		gaitFrequencySlopeX = 0;
		accelerationBackward = 0;
		accelerationForward = 0;
		accelerationRotational = 0;
		accelerationSideward = 0;
		decelerationFactor = 0;
		footOffsetXAccSlope = 0;
		GCVNormP = 0;
		pushHeight = 0;
		pushHeightSlope = 0;
		stepHeight = 0;
		stepHeightSlope = 0;
		swingStartTiming = 0;
		swingStopTiming = 0;
		armSwing = 0;
		armSwingSlope = 0;
		stepLengthYSlope = 0;
		stepLengthXPushout = 0;
		stepLengthYPushout = 0;
		stepLengthZPushout = 0;
		stepLengthZSlope = 0;
		stepLengthZVPushout = 0;
		stepLengthXSlopeFwd = 0;
		stepLengthXSlopeBwd = 0;
		lateralHipSwing = 0;
		lateralHipSwingSlope = 0;
		rotationalHipSwingSlope = 0;
		firstStepDuration = 0;
		firstStepHipSwing = 0;
		stoppingVelocity = 0;
		tiltSpeedBwd = 0;
		tiltSpeedFwd = 0;
		tiltRotLean = 0;
		supportSlope = 0;
		dampingMinimum = 0;
		dampingMaximum = 0;
		dampingLateralSlope = 0;
		dampingSagittalSlope = 0;
		dampingSlopeArmX = 0;
		dampingSlopeArmY = 0;
		fallProtectionRelaxAngle = 0;
		systemIterationTime = 0;
		fusedAngleCorrectionX = 0;
		fusedAngleCorrectionY = 0;
		stableSagAngleFront = 0;
		stableSagAngleSlope = 0;

		solverIterations = 0;
		simulationFrequency = 0;

		balanceOffset = 0;
	}

	// The init() method should be called after construction.
	// Here, all config variables are registered to build a descriptor meta
	// structure that allows index and key based access to their values.
	// If you don't want to see a certain member on the gui, there is no
	// need to register it.
	void Config::init()
	{
		registerMember("captureStep.Cy", &Cy, 0.33);
		registerMember("captureStep.dy", &dy, 0.01);
		registerMember("captureStep.alpha", &alpha, 0.01);
		registerMember("captureStep.delta", &delta, 0.01);
		registerMember("captureStep.omega", &omega, 0.01);
		registerMember("captureStep.D", &D, 0.01);
		registerMember("captureStep.Z", &Z, 0.01);
		registerMember("captureStep.mu", &mu, 0.01 * PI);
		registerMember("captureStep.sigma", &sigma, 0.01);
		registerMember("captureStep.energy", &energy, 0.01);
		registerMember("captureStep.stepDelay", &stepDelay, 0.01);
		registerMember("captureStep.intensity", &intensity, 0.1);
		registerMember("captureStep.stepSizeFactorY", &stepSizeFactorY, 0.01);
		registerMember("captureStep.minEnergyDev", &minEnergyDev, 0.01);

		registerMember("motionInterface.inversePose.leftArmPose.handPosition.x", &motionInterface.inversePose.leftArmPose.handPosition.x, 0.01);
		registerMember("motionInterface.inversePose.leftArmPose.handPosition.y", &motionInterface.inversePose.leftArmPose.handPosition.y, 0.01);
		registerMember("motionInterface.inversePose.leftArmPose.handPosition.z", &motionInterface.inversePose.leftArmPose.handPosition.z, 0.01);
		registerMember("motionInterface.inversePose.rightArmPose.handPosition.x", &motionInterface.inversePose.rightArmPose.handPosition.x, 0.01);
		registerMember("motionInterface.inversePose.rightArmPose.handPosition.y", &motionInterface.inversePose.rightArmPose.handPosition.y, 0.01);
		registerMember("motionInterface.inversePose.rightArmPose.handPosition.z", &motionInterface.inversePose.rightArmPose.handPosition.z, 0.01);
		registerMember("motionInterface.inversePose.leftLegPose.footPosition.x", &motionInterface.inversePose.leftLegPose.footPosition.x, 0.01);
		registerMember("motionInterface.inversePose.leftLegPose.footPosition.y", &motionInterface.inversePose.leftLegPose.footPosition.y, 0.01);
		registerMember("motionInterface.inversePose.leftLegPose.footPosition.z", &motionInterface.inversePose.leftLegPose.footPosition.z, 0.01);
		registerMember("motionInterface.inversePose.leftLegPose.footAngle.x", &motionInterface.inversePose.leftLegPose.footAngle.x, 0.01);
		registerMember("motionInterface.inversePose.leftLegPose.footAngle.y", &motionInterface.inversePose.leftLegPose.footAngle.y, 0.01);
		registerMember("motionInterface.inversePose.leftLegPose.footAngle.z", &motionInterface.inversePose.leftLegPose.footAngle.z, 0.01);
		registerMember("motionInterface.inversePose.rightLegPose.footPosition.x", &motionInterface.inversePose.rightLegPose.footPosition.x, 0.01);
		registerMember("motionInterface.inversePose.rightLegPose.footPosition.y", &motionInterface.inversePose.rightLegPose.footPosition.y, 0.01);
		registerMember("motionInterface.inversePose.rightLegPose.footPosition.z", &motionInterface.inversePose.rightLegPose.footPosition.z, 0.01);
		registerMember("motionInterface.inversePose.rightLegPose.footAngle.x", &motionInterface.inversePose.rightLegPose.footAngle.x, 0.01);
		registerMember("motionInterface.inversePose.rightLegPose.footAngle.y", &motionInterface.inversePose.rightLegPose.footAngle.y, 0.01);
		registerMember("motionInterface.inversePose.rightLegPose.footAngle.z", &motionInterface.inversePose.rightLegPose.footAngle.z, 0.01);

		registerMember("motionInterface.abstractPose.leftArmPose.armAngle.x", &motionInterface.abstractPose.leftArmPose.armAngle.x, 0.01 * PI);
		registerMember("motionInterface.abstractPose.leftArmPose.armAngle.y", &motionInterface.abstractPose.leftArmPose.armAngle.y, 0.01 * PI);
		registerMember("motionInterface.abstractPose.leftArmPose.armAngle.z", &motionInterface.abstractPose.leftArmPose.armAngle.z, 0.01 * PI);
		registerMember("motionInterface.abstractPose.leftArmPose.armExtension", &motionInterface.abstractPose.leftArmPose.armExtension, 0.01);
		registerMember("motionInterface.abstractPose.rightArmPose.armAngle.x", &motionInterface.abstractPose.rightArmPose.armAngle.x, 0.01 * PI);
		registerMember("motionInterface.abstractPose.rightArmPose.armAngle.y", &motionInterface.abstractPose.rightArmPose.armAngle.y, 0.01 * PI);
		registerMember("motionInterface.abstractPose.rightArmPose.armAngle.z", &motionInterface.abstractPose.rightArmPose.armAngle.z, 0.01 * PI);
		registerMember("motionInterface.abstractPose.rightArmPose.armExtension", &motionInterface.abstractPose.rightArmPose.armExtension, 0.01);
		registerMember("motionInterface.abstractPose.leftLegPose.legAngle.x", &motionInterface.abstractPose.leftLegPose.legAngle.x, 0.01 * PI);
		registerMember("motionInterface.abstractPose.leftLegPose.legAngle.y", &motionInterface.abstractPose.leftLegPose.legAngle.y, 0.01 * PI);
		registerMember("motionInterface.abstractPose.leftLegPose.legAngle.z", &motionInterface.abstractPose.leftLegPose.legAngle.z, 0.01 * PI);
		registerMember("motionInterface.abstractPose.leftLegPose.legExtension", &motionInterface.abstractPose.leftLegPose.legExtension, 0.01);
		registerMember("motionInterface.abstractPose.leftLegPose.footAngle.x", &motionInterface.abstractPose.leftLegPose.footAngle.x, 0.01 * PI);
		registerMember("motionInterface.abstractPose.leftLegPose.footAngle.y", &motionInterface.abstractPose.leftLegPose.footAngle.y, 0.01 * PI);
		registerMember("motionInterface.abstractPose.rightLegPose.legAngle.x", &motionInterface.abstractPose.rightLegPose.legAngle.x, 0.01 * PI);
		registerMember("motionInterface.abstractPose.rightLegPose.legAngle.y", &motionInterface.abstractPose.rightLegPose.legAngle.y, 0.01 * PI);
		registerMember("motionInterface.abstractPose.rightLegPose.legAngle.z", &motionInterface.abstractPose.rightLegPose.legAngle.z, 0.01 * PI);
		registerMember("motionInterface.abstractPose.rightLegPose.legExtension", &motionInterface.abstractPose.rightLegPose.legExtension, 0.01);
		registerMember("motionInterface.abstractPose.rightLegPose.footAngle.x", &motionInterface.abstractPose.rightLegPose.footAngle.x, 0.01 * PI);
		registerMember("motionInterface.abstractPose.rightLegPose.footAngle.y", &motionInterface.abstractPose.rightLegPose.footAngle.y, 0.01 * PI);

		registerMember("motionInterface.pose.headPose.neck.y", &motionInterface.pose.headPose.neck.y, 0.01 * PI);
		registerMember("motionInterface.pose.headPose.neck.z", &motionInterface.pose.headPose.neck.z, 0.01 * PI);
		registerMember("motionInterface.pose.leftArmPose.shoulder.x", &motionInterface.pose.leftArmPose.shoulder.x, 0.01 * PI);
		registerMember("motionInterface.pose.leftArmPose.shoulder.y", &motionInterface.pose.leftArmPose.shoulder.y, 0.01 * PI);
		registerMember("motionInterface.pose.leftArmPose.shoulder.z", &motionInterface.pose.leftArmPose.shoulder.z, 0.01 * PI);
		registerMember("motionInterface.pose.leftArmPose.elbow.y", &motionInterface.pose.leftArmPose.elbow.y, 0.01 * PI);
		registerMember("motionInterface.pose.leftLegPose.hip.x", &motionInterface.pose.leftLegPose.hip.x, 0.01 * PI);
		registerMember("motionInterface.pose.leftLegPose.hip.y", &motionInterface.pose.leftLegPose.hip.y, 0.01 * PI);
		registerMember("motionInterface.pose.leftLegPose.hip.z", &motionInterface.pose.leftLegPose.hip.z, 0.01 * PI);
		registerMember("motionInterface.pose.leftLegPose.knee.y", &motionInterface.pose.leftLegPose.knee.y, 0.01 * PI);
		registerMember("motionInterface.pose.leftLegPose.ankle.x", &motionInterface.pose.leftLegPose.ankle.x, 0.01 * PI);
		registerMember("motionInterface.pose.leftLegPose.ankle.y", &motionInterface.pose.leftLegPose.ankle.y, 0.01 * PI);
		registerMember("motionInterface.pose.rightArmPose.shoulder.x", &motionInterface.pose.rightArmPose.shoulder.x, 0.01 * PI);
		registerMember("motionInterface.pose.rightArmPose.shoulder.y", &motionInterface.pose.rightArmPose.shoulder.y, 0.01 * PI);
		registerMember("motionInterface.pose.rightArmPose.shoulder.z", &motionInterface.pose.rightArmPose.shoulder.z, 0.01 * PI);
		registerMember("motionInterface.pose.rightArmPose.elbow.y", &motionInterface.pose.rightArmPose.elbow.y, 0.01 * PI);
		registerMember("motionInterface.pose.rightLegPose.hip.x", &motionInterface.pose.rightLegPose.hip.x, 0.01 * PI);
		registerMember("motionInterface.pose.rightLegPose.hip.y", &motionInterface.pose.rightLegPose.hip.y, 0.01 * PI);
		registerMember("motionInterface.pose.rightLegPose.hip.z", &motionInterface.pose.rightLegPose.hip.z, 0.01 * PI);
		registerMember("motionInterface.pose.rightLegPose.knee.y", &motionInterface.pose.rightLegPose.knee.y, 0.01 * PI);
		registerMember("motionInterface.pose.rightLegPose.ankle.x", &motionInterface.pose.rightLegPose.ankle.x, 0.01 * PI);
		registerMember("motionInterface.pose.rightLegPose.ankle.y", &motionInterface.pose.rightLegPose.ankle.y, 0.01 * PI);

		registerMember("haltPosition.armOffsetX", &armOffsetX, 0.01);
		registerMember("haltPosition.armOffsetY", &armOffsetY, 0.01);
		registerMember("haltPosition.armOffsetZ", &armOffsetZ, 0.01);
		registerMember("haltPosition.footOffsetX", &footOffsetX, 0.01);
		registerMember("haltPosition.footOffsetY", &footOffsetY, 0.01);
		registerMember("haltPosition.footOffsetZ", &footOffsetZ, 0.01);
		registerMember("haltPosition.footAngleX", &footAngleX, 0.01);
		registerMember("haltPosition.footAngleY", &footAngleY, 0.01);

		registerMember("haltPosition.footShiftY", &footShiftY, 0.005);

		registerMember("haltPosition.complianceTrunk", &complianceTrunk, 0.01);
		registerMember("haltPosition.complianceArm", &complianceArm, 0.01);
		registerMember("haltPosition.complianceLeg", &complianceLeg, 0.01);
		registerMember("haltPosition.complianceAnkle", &complianceAnkle, 0.01);

		registerMember("dynamicGait.gaitFrequency", &gaitFrequency, 0.1);
		registerMember("dynamicGait.gaitFrequencySlopeY", &gaitFrequencySlopeY, 0.1);
		registerMember("dynamicGait.gaitFrequencySlopeX", &gaitFrequencySlopeX, 0.01);
		registerMember("dynamicGait.accelerationBackward", &accelerationBackward, 0.01);
		registerMember("dynamicGait.accelerationForward", &accelerationForward, 0.01);
		registerMember("dynamicGait.accelerationRotational", &accelerationRotational, 0.01);
		registerMember("dynamicGait.accelerationSideward", &accelerationSideward, 0.1);
		registerMember("dynamicGait.decelerationFactor", &decelerationFactor, 0.1);
		registerMember("dynamicGait.GCVNormP", &GCVNormP, 0.1);
		registerMember("dynamicGait.pushHeight", &pushHeight, 0.01);
		registerMember("dynamicGait.pushHeightSlope", &pushHeightSlope, 0.01);
		registerMember("dynamicGait.stepHeight", &stepHeight, 0.01);
		registerMember("dynamicGait.stepHeightSlope", &stepHeightSlope, 0.01);
		registerMember("dynamicGait.swingStartTiming", &swingStartTiming, 0.01);
		registerMember("dynamicGait.swingStopTiming", &swingStopTiming, 0.01 * PI);
		registerMember("dynamicGait.armSwing", &armSwing, 0.01);
		registerMember("dynamicGait.armSwingSlope", &armSwingSlope, 0.01);
		registerMember("dynamicGait.stepLengthYSlope", &stepLengthYSlope, 0.01);
		registerMember("dynamicGait.stepLengthXPushout", &stepLengthXPushout, 0.01);
		registerMember("dynamicGait.stepLengthYPushout", &stepLengthYPushout, 0.01);
		registerMember("dynamicGait.stepLengthZPushout", &stepLengthZPushout, 0.01);
		registerMember("dynamicGait.stepLengthZSlope", &stepLengthZSlope, 0.01);
		registerMember("dynamicGait.stepLengthZVPushout", &stepLengthZVPushout, 0.05);
		registerMember("dynamicGait.stepLengthXSlopeFwd", &stepLengthXSlopeFwd, 0.01);
		registerMember("dynamicGait.stepLengthXSlopeBwd", &stepLengthXSlopeBwd, 0.01);
		registerMember("dynamicGait.lateralHipSwing", &lateralHipSwing, 0.001);
		registerMember("dynamicGait.lateralHipSwingSlope", &lateralHipSwingSlope, 0.001);
		registerMember("dynamicGait.rotationalHipSwingSlope", &rotationalHipSwingSlope, 0.01);
		registerMember("dynamicGait.firstStepDuration", &firstStepDuration, 0.01);
		registerMember("dynamicGait.firstStepHipSwing", &firstStepHipSwing, 0.01);
		registerMember("dynamicGait.stoppingVelocity", &stoppingVelocity, 0.01);
		registerMember("dynamicGait.tiltSpeedBwd", &tiltSpeedBwd, 0.01);
		registerMember("dynamicGait.tiltSpeedFwd", &tiltSpeedFwd, 0.01);
		registerMember("dynamicGait.tiltRotLean", &tiltRotLean, 0.01);
		registerMember("dynamicGait.supportSlope", &supportSlope, 0.05);
		registerMember("dynamicGait.footOffsetXAccSlope", &footOffsetXAccSlope, 0.001);
		registerMember("dynamicGait.stableSagAngleFront", &stableSagAngleFront, 0.01*PI);
		registerMember("dynamicGait.stableSagAngleSlope", &stableSagAngleSlope, 1);

		registerMember("damping.Minimum", &dampingMinimum, 0.01);
		registerMember("damping.Maximum", &dampingMaximum, 0.01);
		registerMember("damping.LateralSlope", &dampingLateralSlope, 0.01);
		registerMember("damping.SagittalSlope", &dampingSagittalSlope, 0.01);
		registerMember("damping.SlopeArmX", &dampingSlopeArmX, 0.01);
		registerMember("damping.SlopeArmY", &dampingSlopeArmY, 0.01);

		registerMember("fallProtectionRelaxAngle", &fallProtectionRelaxAngle, 1.0);
		registerMember("systemIterationTime", &systemIterationTime, 0.001);
		registerMember("fusedAngleCorrectionX", &fusedAngleCorrectionX, 0.001);
		registerMember("fusedAngleCorrectionY", &fusedAngleCorrectionY, 0.001);

		registerMember("simulation.solverIterations", &solverIterations, 1.0);
		registerMember("simulation.simulationFrequency", &simulationFrequency, 1.0);

		registerMember("dynamicGait.balanceOffset", &balanceOffset, 0.01);
	}

	// Loads the config variables from the .conf file.
	// Unregistered variables are ignored.
	void Config::load(QString robotName)
	{
		QFile file("conf/" + robotName + ".conf");
		if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			qDebug() << "Couldn't load config file" << file.fileName();
			return;
		}

		QTextStream in(&file);
		QString line;
		QStringList list;
		bool ok;
		while (!in.atEnd())
		{
			line = in.readLine().trimmed();
			list = line.split("=");
			if (list.length() == 2 and not line.startsWith("//") and not line.startsWith("#"))
			{
				QString key = list[0].trimmed();
				double value = list[1].trimmed().toDouble(&ok);
				this->operator[](key) = value;
			}
		}

		file.close();
	}

	// Saves the config variables to the .conf file.
	void Config::save(QString robotName)
	{
		QFile file("conf/" + robotName + ".conf");
		if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
		{
			qDebug() << "Couldn't open config file" << file.fileName();
			return;
		}

		QTextStream out(&file);
		foreach (QString key, memberNames)
			out << key << "=" << QString::number(this->operator[](key)) << endl;
		file.close();
	}

	// Returns a reference to the ith member of this object.
	double& Config::operator()(int i)
	{
		return this->operator[](memberNames[i]);
	}

	// Returns a reference to the ith member of this object.
	double& Config::operator[](int i)
	{
		return this->operator[](memberNames[i]);
	}

	// Returns a reference to the member that was registered with the given key.
	double& Config::operator()(QString key)
	{
		return this->operator[](key);
	}

	// Returns a reference to the member that was registered with the given key.
	double& Config::operator[](QString key)
	{
		double* ptr = (double*)((unsigned long int)this+memberOffsets[key]);
		double& rf = *ptr;
		return rf;
	}

}