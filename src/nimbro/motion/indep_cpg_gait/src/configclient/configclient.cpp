//Author: Sebastian Schüller

#include <config_server/ParameterDescription.h>

#include <boost/foreach.hpp>

#include "configclient.h"


const char* parameterList[] = {
		"captureStep.Cy",
		"captureStep.dy",
		"captureStep.alpha",
		"captureStep.delta",
		"captureStep.omega",
		"captureStep.D",
		"captureStep.Z",
		"captureStep.mu",
		"captureStep.sigma",
		"captureStep.energy",
		"captureStep.stepDelay",
		"captureStep.intensity",
		"captureStep.stepSizeFactorY",
		"captureStep.minEnergyDev",

// 		"motionInterface.inversePose.leftArmPose.handPosition.x",
// 		"motionInterface.inversePose.leftArmPose.handPosition.y",
// 		"motionInterface.inversePose.leftArmPose.handPosition.z",
// 		"motionInterface.inversePose.rightArmPose.handPosition.x",
// 		"motionInterface.inversePose.rightArmPose.handPosition.y",
// 		"motionInterface.inversePose.rightArmPose.handPosition.z",
// 		"motionInterface.inversePose.leftLegPose.footPosition.x",
// 		"motionInterface.inversePose.leftLegPose.footPosition.y",
// 		"motionInterface.inversePose.leftLegPose.footPosition.z",
// 		"motionInterface.inversePose.leftLegPose.footAngle.x",
// 		"motionInterface.inversePose.leftLegPose.footAngle.y",
// 		"motionInterface.inversePose.leftLegPose.footAngle.z",
// 		"motionInterface.inversePose.rightLegPose.footPosition.x",
// 		"motionInterface.inversePose.rightLegPose.footPosition.y",
// 		"motionInterface.inversePose.rightLegPose.footPosition.z",
// 		"motionInterface.inversePose.rightLegPose.footAngle.x",
// 		"motionInterface.inversePose.rightLegPose.footAngle.y",
// 		"motionInterface.inversePose.rightLegPose.footAngle.z",
// 
// 		"motionInterface.abstractPose.leftArmPose.armAngle.x",
// 		"motionInterface.abstractPose.leftArmPose.armAngle.y",
// 		"motionInterface.abstractPose.leftArmPose.armAngle.z",
// 		"motionInterface.abstractPose.leftArmPose.armExtension",
// 		"motionInterface.abstractPose.rightArmPose.armAngle.x",
// 		"motionInterface.abstractPose.rightArmPose.armAngle.y",
// 		"motionInterface.abstractPose.rightArmPose.armAngle.z",
// 		"motionInterface.abstractPose.rightArmPose.armExtension",
// 		"motionInterface.abstractPose.leftLegPose.legAngle.x",
// 		"motionInterface.abstractPose.leftLegPose.legAngle.y",
// 		"motionInterface.abstractPose.leftLegPose.legAngle.z",
// 		"motionInterface.abstractPose.leftLegPose.legExtension",
// 		"motionInterface.abstractPose.leftLegPose.footAngle.x",
// 		"motionInterface.abstractPose.leftLegPose.footAngle.y",
// 		"motionInterface.abstractPose.rightLegPose.legAngle.x",
// 		"motionInterface.abstractPose.rightLegPose.legAngle.y",
// 		"motionInterface.abstractPose.rightLegPose.legAngle.z",
// 		"motionInterface.abstractPose.rightLegPose.legExtension",
// 		"motionInterface.abstractPose.rightLegPose.footAngle.x",
// 		"motionInterface.abstractPose.rightLegPose.footAngle.y",
// 
// 		"motionInterface.pose.headPose.neck.y",
// 		"motionInterface.pose.headPose.neck.z",
// 		"motionInterface.pose.leftArmPose.shoulder.x",
// 		"motionInterface.pose.leftArmPose.shoulder.y",
// 		"motionInterface.pose.leftArmPose.shoulder.z",
// 		"motionInterface.pose.leftArmPose.elbow.y",
// 		"motionInterface.pose.leftLegPose.hip.x",
// 		"motionInterface.pose.leftLegPose.hip.y",
// 		"motionInterface.pose.leftLegPose.hip.z",
// 		"motionInterface.pose.leftLegPose.knee.y",
// 		"motionInterface.pose.leftLegPose.ankle.x",
// 		"motionInterface.pose.leftLegPose.ankle.y",
// 		"motionInterface.pose.rightArmPose.shoulder.x",
// 		"motionInterface.pose.rightArmPose.shoulder.y",
// 		"motionInterface.pose.rightArmPose.shoulder.z",
// 		"motionInterface.pose.rightArmPose.elbow.y",
// 		"motionInterface.pose.rightLegPose.hip.x",
// 		"motionInterface.pose.rightLegPose.hip.y",
// 		"motionInterface.pose.rightLegPose.hip.z",
// 		"motionInterface.pose.rightLegPose.knee.y",
// 		"motionInterface.pose.rightLegPose.ankle.x",
// 		"motionInterface.pose.rightLegPose.ankle.y",

		"haltPosition.armOffsetX",
		"haltPosition.armOffsetY",
		"haltPosition.armOffsetZ",
		"haltPosition.footOffsetX",
		"haltPosition.footOffsetY",
		"haltPosition.footOffsetZ",
		"haltPosition.footAngleX",
		"haltPosition.footAngleY",

		"haltPosition.footShiftY",

		"haltPosition.complianceTrunk",
		"haltPosition.complianceArm",
		"haltPosition.complianceLeg",
		"haltPosition.complianceAnkle",

		"dynamicGait.gaitFrequency",
		"dynamicGait.gaitFrequencySlopeY",
		"dynamicGait.gaitFrequencySlopeX",
		"dynamicGait.accelerationBackward",
		"dynamicGait.accelerationForward",
		"dynamicGait.accelerationRotational",
		"dynamicGait.accelerationSideward",
		"dynamicGait.decelerationFactor",
		"dynamicGait.GCVNormP",
		"dynamicGait.pushHeight",
		"dynamicGait.pushHeightSlope",
		"dynamicGait.stepHeight",
		"dynamicGait.stepHeightSlope",
		"dynamicGait.swingStartTiming",
		"dynamicGait.swingStopTiming",
		"dynamicGait.armSwing",
		"dynamicGait.armSwingSlope",
		"dynamicGait.stepLengthYSlope",
		"dynamicGait.stepLengthXPushout",
		"dynamicGait.stepLengthYPushout",
		"dynamicGait.stepLengthZPushout",
		"dynamicGait.stepLengthZSlope",
		"dynamicGait.stepLengthZVPushout",
		"dynamicGait.stepLengthXSlopeFwd",
		"dynamicGait.stepLengthXSlopeBwd",
		"dynamicGait.lateralHipSwing",
		"dynamicGait.lateralHipSwingSlope",
		"dynamicGait.rotationalHipSwingSlope",
		"dynamicGait.firstStepDuration",
		"dynamicGait.firstStepHipSwing",
		"dynamicGait.stoppingVelocity",
		"dynamicGait.tiltSpeedBwd",
		"dynamicGait.tiltSpeedFwd",
		"dynamicGait.tiltRotLean",
		"dynamicGait.supportSlope",
		"dynamicGait.footOffsetXAccSlope",
		"dynamicGait.stableSagAngleFront",
		"dynamicGait.stableSagAngleSlope",

		"damping.Minimum",
		"damping.Maximum",
		"damping.LateralSlope",
		"damping.SagittalSlope",
		"damping.SlopeArmX",
		"damping.SlopeArmY",

		"fallProtectionRelaxAngle",
		"systemIterationTime",
		"fusedAngleCorrectionX",
		"fusedAngleCorrectionY",

		"simulation.solverIterations",
		"simulation.simulationFrequency",

		"dynamicGait.balanceOffset",

		0
};

namespace config_client 
{

inline QString reformat(const char* name)
{ return QString(name).replace(".", "/"); }


ConfigClient::ConfigClient()
{}

ConfigClient::~ConfigClient()
{}

void ConfigClient::init()
{
	indep_cpg_gait::config.init();
	indep_cpg_gait::state.init();

	config_server::ParameterDescription newDescription;
	int i;
	for (i = 0; parameterList[i] != 0; i++)
	{
		newDescription.name = "/indep_cpg_gait/" + reformat(parameterList[i]).toStdString();
		newDescription.default_value = "0";
		newDescription.step = indep_cpg_gait::config.sliderFactors[parameterList[i]];
		newDescription.min = -100 * indep_cpg_gait::config.sliderFactors[parameterList[i]];
		newDescription.max = 100 * indep_cpg_gait::config.sliderFactors[parameterList[i]];
		newDescription.type = "float";
		createParameter(newDescription, QString(parameterList[i]));
	}
}

void ConfigClient::setCallback(const boost::function<void()>& cb)
{
	BOOST_FOREACH(const boost::shared_ptr<GaitConfigItem>& item, m_itemList)
	{
		item->setCallback(cb);
	}
}

void ConfigClient::createParameter(const config_server::ParameterDescription& description, QString configName)
{
	boost::shared_ptr<GaitConfigItem> item (new GaitConfigItem(description, configName));
	assert(item);
	m_itemList.push_back(item);
}


}
