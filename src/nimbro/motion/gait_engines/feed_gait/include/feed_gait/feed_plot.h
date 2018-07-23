// Feedback gait plotting
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_PLOT_H
#define FEED_PLOT_H

// Includes
#include <plot_msgs/plot_manager.h>

// Feedback gait namespace
namespace feed_gait
{
	// Typedefs
	typedef plot_msgs::PlotManagerFS FeedPlotManager;

	// Feedback gait plot manager IDs enumeration
	enum FeedPMIDS
	{
		PM_FG_CURRENT_KIN_TYPE,
		PM_FG_CURRENT_TRAJ_TYPE,
		PM_FG_CURRENT_ODOM_TYPE,
		PM_FG_WALKING,
		PM_FG_GPI_GAITPHASE,
		PM_FG_GPI_ELAPGAITPHASE,
		PM_FG_GPI_REMGAITPHASE,
		PM_FG_GPI_SUPPLEGINDEX,
		PM_FG_GI_NOMINALDT,
		PM_FG_GI_TRUEDT,
		PM_FG_GI_FUSEDYAW,
		PM_FG_GI_FUSEDPITCH,
		PM_FG_GI_FUSEDROLL,
		PM_FG_GI_INPUTGCVX,
		PM_FG_GI_INPUTGCVY,
		PM_FG_GI_INPUTGCVZ,
		PM_FG_TI_DBLSUPPPHASELEN,
		PM_FG_TI_FUSEDPITCHN,
		PM_FG_TI_HIPHEIGHT_MIN,
		PM_FG_TI_HIPHEIGHT_NOM,
		PM_FG_MI_NOMGAITFREQ,
		PM_FG_TC_GAITFREQ,
		PM_FG_TO_GAITFREQ,
		PM_FG_TO_TIMETOSTEP,
		PM_FG_NEWGAITPHASE,
		PM_FG_SSC_USEMODELCMDX,
		PM_FG_SSC_USEMODELCMDY,
		PM_FG_SSC_USEMODELCMDZ,
		PM_FG_SSC_GCVLFX,
		PM_FG_SSC_GCVLFY,
		PM_FG_SSC_GCVLFZ,
		PM_FG_SSC_GCVHFX,
		PM_FG_SSC_GCVHFY,
		PM_FG_SSC_GCVHFZ,
		PM_FG_SSC_GCVEOSX,
		PM_FG_SSC_GCVEOSY,
		PM_FG_SSC_GCVEOSZ,
		PM_FG_SSO_TARGETGCVX,
		PM_FG_SSO_TARGETGCVY,
		PM_FG_SSO_TARGETGCVZ,
		PM_FG_TC_GCVX,
		PM_FG_TC_GCVY,
		PM_FG_TC_GCVZ,
		PM_FG_TC_GCVLFACCX,
		PM_FG_TC_GCVLFACCY,
		PM_FG_TC_GCVLFACCZ,
		PM_FG_TC_FUSEDPITCHS,
		PM_FG_TC_FUSEDROLLS,
		PM_FG_TC_FOOTTILTCTS_A,
		PM_FG_TC_FOOTTILTSUPP_A,
		PM_FG_TC_SWINGOUT_A,
		PM_FG_TC_LEANTILT_A,
		PM_FG_TC_HIPSHIFTX,
		PM_FG_TC_HIPSHIFTY,
		PM_FG_TC_HIPHEIGHTMAX,
		PM_FG_TC_ARMTILT_A,
		PM_FG_TC_ACFLAGS,
		PM_FG_PC_SUPPCOEFF_LEFT,
		PM_FG_PC_SUPPCOEFF_RIGHT,
		PM_FG_HALT_BLENDING,
		PM_FG_HALT_BLEND_FACTOR,

		PM_TPM_SI_PHASEX,
		PM_TPM_SI_PHASEY,
		PM_TPM_SI_EXPPHASEX,
		PM_TPM_SI_EXPPHASEY,
		PM_TPM_SI_SYNCPHASEX,
		PM_TPM_SI_SYNCPHASEY,
		PM_TPM_SI_SYNCPHASEDX,
		PM_TPM_SI_SYNCPHASEDY,
		PM_TPM_PID_DEVIATIONX,
		PM_TPM_PID_DEVIATIONY,
		PM_TPM_PID_MEANFILTEREDX,
		PM_TPM_PID_MEANFILTEREDY,
		PM_TPM_PID_WLBFDERIVX,
		PM_TPM_PID_WLBFDERIVY,
		PM_TPM_PID_INTEGRANDX,
		PM_TPM_PID_INTEGRANDY,
		PM_TPM_PID_INTEGRALX,
		PM_TPM_PID_INTEGRALY,
		PM_TPM_PID_FEEDPHASEPX,
		PM_TPM_PID_FEEDPHASEPY,
		PM_TPM_PID_FEEDPHASEDX,
		PM_TPM_PID_FEEDPHASEDY,
		PM_TPM_PID_FEEDPHASEIX,
		PM_TPM_PID_FEEDPHASEIY,
		PM_TPM_PS_NSDEVIATION_X,
		PM_TPM_PS_NSDEVIATION_Y,
		PM_TPM_PS_NSFEED_X,
		PM_TPM_PS_NSFEED_Y,
		PM_TPM_SO_LIMPENERGY_L,
		PM_TPM_SO_LIMPENERGY_R,
		PM_TPM_SO_CROSSINGENERGY_L,
		PM_TPM_SO_CROSSINGENERGY_R,
		PM_TPM_SO_SWINGOUTX_L,
		PM_TPM_SO_SWINGOUTX_R,
		PM_TPM_SO_SWINGOUTX,
		PM_TPM_SO_SWINGOUTY,
		PM_TPM_HHM_PHASEDEVSPEED,
		PM_TPM_HHM_INSTABILITY,
		PM_TPM_HHM_HIPHEIGHTMAX_TGT,
		PM_TPM_HHM_HIPHEIGHTMAX,
		PM_TPM_TIM_FEEDWEIGHT,
		PM_TPM_TIM_FREQDELTA,
		PM_TPM_SS_THETABM,
		PM_TPM_SS_THETAMF,
		PM_TPM_SS_CROSSENERGY_B,
		PM_TPM_SS_CROSSENERGY_F,
		PM_TPM_SS_GCVDELTAX_B,
		PM_TPM_SS_GCVDELTAX_F,
		PM_TPM_SS_GCVDELTAX,

		PM_KT_CMD_GCVINTERNALX,
		PM_KT_CMD_GCVINTERNALY,
		PM_KT_CMD_GCVINTERNALZ,
		PM_KT_CMD_GCVACCINTERNALX,
		PM_KT_CMD_GCVACCINTERNALY,
		PM_KT_CMD_GCVACCINTERNALZ,
		PM_KT_CMD_LEANPHASEX,
		PM_KT_CMD_LEANPHASEY,

		PM_COUNT
	};

	// Plot manager configuration
	inline void configurePlotManager(FeedPlotManager* PM)
	{
		// Configure feedback gait variables
		PM->setName(PM_FG_CURRENT_KIN_TYPE,   "FeedGait/dynamicObjects/currentKinType");
		PM->setName(PM_FG_CURRENT_TRAJ_TYPE,  "FeedGait/dynamicObjects/currentTrajType");
		PM->setName(PM_FG_CURRENT_ODOM_TYPE,  "FeedGait/dynamicObjects/currentOdomType");
		PM->setName(PM_FG_WALKING,            "FeedGait/walking");
		PM->setName(PM_FG_GPI_GAITPHASE,      "FeedGait/01 gaitPhaseInfo/gaitPhase");
		PM->setName(PM_FG_GPI_ELAPGAITPHASE,  "FeedGait/01 gaitPhaseInfo/elapGaitPhase");
		PM->setName(PM_FG_GPI_REMGAITPHASE,   "FeedGait/01 gaitPhaseInfo/remGaitPhase");
		PM->setName(PM_FG_GPI_SUPPLEGINDEX,   "FeedGait/01 gaitPhaseInfo/suppLegIndex");
		PM->setName(PM_FG_GI_NOMINALDT,       "FeedGait/02 gaitInput/dT/nominaldT");
		PM->setName(PM_FG_GI_TRUEDT,          "FeedGait/02 gaitInput/dT/truedT");
		PM->setName(PM_FG_GI_FUSEDYAW,        "FeedGait/02 gaitInput/robotOrient/fusedYaw");
		PM->setName(PM_FG_GI_FUSEDPITCH,      "FeedGait/02 gaitInput/robotOrient/fusedPitch");
		PM->setName(PM_FG_GI_FUSEDROLL,       "FeedGait/02 gaitInput/robotOrient/fusedRoll");
		PM->setName(PM_FG_GI_INPUTGCVX,       "FeedGait/02 gaitInput/inputGcv/x");
		PM->setName(PM_FG_GI_INPUTGCVY,       "FeedGait/02 gaitInput/inputGcv/y");
		PM->setName(PM_FG_GI_INPUTGCVZ,       "FeedGait/02 gaitInput/inputGcv/z");
		PM->setName(PM_FG_TI_DBLSUPPPHASELEN, "FeedGait/03 trajInfo/dblSuppPhaseLen");
		PM->setName(PM_FG_TI_FUSEDPITCHN,     "FeedGait/03 trajInfo/fusedPitchN");
		PM->setName(PM_FG_TI_HIPHEIGHT_MIN,   "FeedGait/03 trajInfo/hipHeightMin");
		PM->setName(PM_FG_TI_HIPHEIGHT_NOM,   "FeedGait/03 trajInfo/hipHeightNom");
		PM->setName(PM_FG_MI_NOMGAITFREQ,     "FeedGait/04 modelInput/nomGaitFreq");
		PM->setName(PM_FG_TC_GAITFREQ,        "FeedGait/05 timingCommand/gaitFreq");
		PM->setName(PM_FG_TO_GAITFREQ,        "FeedGait/06 timingOutput/gaitFreq");
		PM->setName(PM_FG_TO_TIMETOSTEP,      "FeedGait/06 timingOutput/timeToStep");
		PM->setName(PM_FG_NEWGAITPHASE,       "FeedGait/07 newGaitPhase");
		PM->setName(PM_FG_SSC_USEMODELCMDX,   "FeedGait/08 stepSizeCmd/useModelCmd/x");
		PM->setName(PM_FG_SSC_USEMODELCMDY,   "FeedGait/08 stepSizeCmd/useModelCmd/y");
		PM->setName(PM_FG_SSC_USEMODELCMDZ,   "FeedGait/08 stepSizeCmd/useModelCmd/z");
		PM->setName(PM_FG_SSC_GCVLFX,         "FeedGait/08 stepSizeCmd/gcvLF/x");
		PM->setName(PM_FG_SSC_GCVLFY,         "FeedGait/08 stepSizeCmd/gcvLF/y");
		PM->setName(PM_FG_SSC_GCVLFZ,         "FeedGait/08 stepSizeCmd/gcvLF/z");
		PM->setName(PM_FG_SSC_GCVHFX,         "FeedGait/08 stepSizeCmd/gcvHF/x");
		PM->setName(PM_FG_SSC_GCVHFY,         "FeedGait/08 stepSizeCmd/gcvHF/y");
		PM->setName(PM_FG_SSC_GCVHFZ,         "FeedGait/08 stepSizeCmd/gcvHF/z");
		PM->setName(PM_FG_SSC_GCVEOSX,        "FeedGait/08 stepSizeCmd/gcvEOS/x");
		PM->setName(PM_FG_SSC_GCVEOSY,        "FeedGait/08 stepSizeCmd/gcvEOS/y");
		PM->setName(PM_FG_SSC_GCVEOSZ,        "FeedGait/08 stepSizeCmd/gcvEOS/z");
		PM->setName(PM_FG_SSO_TARGETGCVX,     "FeedGait/09 stepSizeOutput/targetGcv/x");
		PM->setName(PM_FG_SSO_TARGETGCVY,     "FeedGait/09 stepSizeOutput/targetGcv/y");
		PM->setName(PM_FG_SSO_TARGETGCVZ,     "FeedGait/09 stepSizeOutput/targetGcv/z");
		PM->setName(PM_FG_TC_GCVX,            "FeedGait/10 trajCmd/gcv/x");
		PM->setName(PM_FG_TC_GCVY,            "FeedGait/10 trajCmd/gcv/y");
		PM->setName(PM_FG_TC_GCVZ,            "FeedGait/10 trajCmd/gcv/z");
		PM->setName(PM_FG_TC_GCVLFACCX,       "FeedGait/10 trajCmd/gcvLFAcc/x");
		PM->setName(PM_FG_TC_GCVLFACCY,       "FeedGait/10 trajCmd/gcvLFAcc/y");
		PM->setName(PM_FG_TC_GCVLFACCZ,       "FeedGait/10 trajCmd/gcvLFAcc/z");
		PM->setName(PM_FG_TC_FUSEDPITCHS,     "FeedGait/10 trajCmd/fusedPitchS");
		PM->setName(PM_FG_TC_FUSEDROLLS,      "FeedGait/10 trajCmd/fusedRollS");
		PM->setName(PM_FG_TC_FOOTTILTCTS_A,   "FeedGait/10 trajCmd/footTiltCts/tiltAngle");
		PM->setName(PM_FG_TC_FOOTTILTSUPP_A,  "FeedGait/10 trajCmd/footTiltSupp/tiltAngle");
		PM->setName(PM_FG_TC_SWINGOUT_A,      "FeedGait/10 trajCmd/swingOut/tiltAngle");
		PM->setName(PM_FG_TC_LEANTILT_A,      "FeedGait/10 trajCmd/leanTilt/tiltAngle");
		PM->setName(PM_FG_TC_HIPSHIFTX,       "FeedGait/10 trajCmd/hipShift/x");
		PM->setName(PM_FG_TC_HIPSHIFTY,       "FeedGait/10 trajCmd/hipShift/y");
		PM->setName(PM_FG_TC_HIPHEIGHTMAX,    "FeedGait/10 trajCmd/hipHeightMax");
		PM->setName(PM_FG_TC_ARMTILT_A,       "FeedGait/10 trajCmd/armTilt/tiltAngle");
		PM->setName(PM_FG_TC_ACFLAGS,         "FeedGait/10 trajCmd/ACFlags");
		PM->setName(PM_FG_PC_SUPPCOEFF_LEFT,  "FeedGait/11 poseCmd/suppCoeff/left");
		PM->setName(PM_FG_PC_SUPPCOEFF_RIGHT, "FeedGait/11 poseCmd/suppCoeff/right");
		PM->setName(PM_FG_HALT_BLENDING,      "FeedGait/blending/haltBlending");
		PM->setName(PM_FG_HALT_BLEND_FACTOR,  "FeedGait/blending/haltBlendFactor");

		// Configure tilt phase model variables
		PM->setName(PM_TPM_SI_PHASEX,            "TiltPhaseModel/stateInfo/phaseX");
		PM->setName(PM_TPM_SI_PHASEY,            "TiltPhaseModel/stateInfo/phaseY");
		PM->setName(PM_TPM_SI_EXPPHASEX,         "TiltPhaseModel/stateInfo/expectedPhaseX");
		PM->setName(PM_TPM_SI_EXPPHASEY,         "TiltPhaseModel/stateInfo/expectedPhaseY");
		PM->setName(PM_TPM_SI_SYNCPHASEX,        "TiltPhaseModel/stateInfo/syncPhaseX");
		PM->setName(PM_TPM_SI_SYNCPHASEY,        "TiltPhaseModel/stateInfo/syncPhaseY");
		PM->setName(PM_TPM_SI_SYNCPHASEDX,       "TiltPhaseModel/stateInfo/syncPhaseDX");
		PM->setName(PM_TPM_SI_SYNCPHASEDY,       "TiltPhaseModel/stateInfo/syncPhaseDY");
		PM->setName(PM_TPM_PID_DEVIATIONX,       "TiltPhaseModel/PID/deviationX");
		PM->setName(PM_TPM_PID_DEVIATIONY,       "TiltPhaseModel/PID/deviationY");
		PM->setName(PM_TPM_PID_MEANFILTEREDX,    "TiltPhaseModel/PID/meanFilteredX");
		PM->setName(PM_TPM_PID_MEANFILTEREDY,    "TiltPhaseModel/PID/meanFilteredY");
		PM->setName(PM_TPM_PID_WLBFDERIVX,       "TiltPhaseModel/PID/wlbfDerivX");
		PM->setName(PM_TPM_PID_WLBFDERIVY,       "TiltPhaseModel/PID/wlbfDerivY");
		PM->setName(PM_TPM_PID_INTEGRANDX,       "TiltPhaseModel/PID/integrandX");
		PM->setName(PM_TPM_PID_INTEGRANDY,       "TiltPhaseModel/PID/integrandY");
		PM->setName(PM_TPM_PID_INTEGRALX,        "TiltPhaseModel/PID/integralX");
		PM->setName(PM_TPM_PID_INTEGRALY,        "TiltPhaseModel/PID/integralY");
		PM->setName(PM_TPM_PID_FEEDPHASEPX,      "TiltPhaseModel/PID/feedPhasePX");
		PM->setName(PM_TPM_PID_FEEDPHASEPY,      "TiltPhaseModel/PID/feedPhasePY");
		PM->setName(PM_TPM_PID_FEEDPHASEDX,      "TiltPhaseModel/PID/feedPhaseDX");
		PM->setName(PM_TPM_PID_FEEDPHASEDY,      "TiltPhaseModel/PID/feedPhaseDY");
		PM->setName(PM_TPM_PID_FEEDPHASEIX,      "TiltPhaseModel/PID/feedPhaseIX");
		PM->setName(PM_TPM_PID_FEEDPHASEIY,      "TiltPhaseModel/PID/feedPhaseIY");
		PM->setName(PM_TPM_PS_NSDEVIATION_X,     "TiltPhaseModel/planeS/deviationNSX");
		PM->setName(PM_TPM_PS_NSDEVIATION_Y,     "TiltPhaseModel/planeS/deviationNSY");
		PM->setName(PM_TPM_PS_NSFEED_X,          "TiltPhaseModel/planeS/feedNSX");
		PM->setName(PM_TPM_PS_NSFEED_Y,          "TiltPhaseModel/planeS/feedNSY");
		PM->setName(PM_TPM_SO_LIMPENERGY_L,      "TiltPhaseModel/swingOut/limpEnergyL");
		PM->setName(PM_TPM_SO_LIMPENERGY_R,      "TiltPhaseModel/swingOut/limpEnergyR");
		PM->setName(PM_TPM_SO_CROSSINGENERGY_L,  "TiltPhaseModel/swingOut/crossingEnergyL");
		PM->setName(PM_TPM_SO_CROSSINGENERGY_R,  "TiltPhaseModel/swingOut/crossingEnergyR");
		PM->setName(PM_TPM_SO_SWINGOUTX_L,       "TiltPhaseModel/swingOut/swingOutXL");
		PM->setName(PM_TPM_SO_SWINGOUTX_R,       "TiltPhaseModel/swingOut/swingOutXR");
		PM->setName(PM_TPM_SO_SWINGOUTX,         "TiltPhaseModel/swingOut/swingOutX");
		PM->setName(PM_TPM_SO_SWINGOUTY,         "TiltPhaseModel/swingOut/swingOutY");
		PM->setName(PM_TPM_HHM_PHASEDEVSPEED,    "TiltPhaseModel/hipHeightMax/phaseDevSpeed");
		PM->setName(PM_TPM_HHM_INSTABILITY,      "TiltPhaseModel/hipHeightMax/instability");
		PM->setName(PM_TPM_HHM_HIPHEIGHTMAX_TGT, "TiltPhaseModel/hipHeightMax/hipHeightMaxTarget");
		PM->setName(PM_TPM_HHM_HIPHEIGHTMAX,     "TiltPhaseModel/hipHeightMax/hipHeightMax");
		PM->setName(PM_TPM_TIM_FEEDWEIGHT,       "TiltPhaseModel/timing/feedWeight");
		PM->setName(PM_TPM_TIM_FREQDELTA,        "TiltPhaseModel/timing/frequencyDelta");
		PM->setName(PM_TPM_SS_THETABM,           "TiltPhaseModel/stepSize/thetaBM");
		PM->setName(PM_TPM_SS_THETAMF,           "TiltPhaseModel/stepSize/thetaMF");
		PM->setName(PM_TPM_SS_CROSSENERGY_B,     "TiltPhaseModel/stepSize/crossingEnergyB");
		PM->setName(PM_TPM_SS_CROSSENERGY_F,     "TiltPhaseModel/stepSize/crossingEnergyF");
		PM->setName(PM_TPM_SS_GCVDELTAX_B,       "TiltPhaseModel/stepSize/gcvDeltaX/B");
		PM->setName(PM_TPM_SS_GCVDELTAX_F,       "TiltPhaseModel/stepSize/gcvDeltaX/F");
		PM->setName(PM_TPM_SS_GCVDELTAX,         "TiltPhaseModel/stepSize/gcvDeltaX/final");

		// Configure keypoint trajectory generation variables
		PM->setName(PM_KT_CMD_GCVINTERNALX,    "KeypointTraj/cmd/gcvInternal/x");
		PM->setName(PM_KT_CMD_GCVINTERNALY,    "KeypointTraj/cmd/gcvInternal/y");
		PM->setName(PM_KT_CMD_GCVINTERNALZ,    "KeypointTraj/cmd/gcvInternal/z");
		PM->setName(PM_KT_CMD_GCVACCINTERNALX, "KeypointTraj/cmd/gcvAccInternal/x");
		PM->setName(PM_KT_CMD_GCVACCINTERNALY, "KeypointTraj/cmd/gcvAccInternal/y");
		PM->setName(PM_KT_CMD_GCVACCINTERNALZ, "KeypointTraj/cmd/gcvAccInternal/z");
		PM->setName(PM_KT_CMD_LEANPHASEX,      "KeypointTraj/cmd/leanTilt/px");
		PM->setName(PM_KT_CMD_LEANPHASEY,      "KeypointTraj/cmd/leanTilt/py");

		// Check that we have been thorough
		if(!PM->checkNames())
			ROS_ERROR("Please review any warnings above that are related to the naming of plotter variables!");
	}
}

#endif
// EOF