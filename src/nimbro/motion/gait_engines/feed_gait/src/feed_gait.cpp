// Feedback gait
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <feed_gait/feed_gait.h>
#include <feed_gait/kinematics/feed_kinematics.h>
#include <robotcontrol/model/robotmodel.h>
#include <pluginlib/class_list_macros.h>

// Namespaces
using namespace feed_gait;

// ########################
// #### FeedGait class ####
// ########################

//
// General
//

// Constructor
FeedGait::FeedGait() : GaitEngine()
 , config()
 , m_PM(new FeedPlotManager(PM_COUNT, RESOURCE_PATH))
{
	// Reset the gait engine
	FeedGait::reset();

	// Configure the plot manager
	configurePlotManager(m_PM);

	// Configure configuration parameter callbacks
	config.plotData.setCallback(boost::bind(&FeedGait::callbackPlotData, this), true);
}

// Destructor
FeedGait::~FeedGait()
{
	// Delete variables
	delete m_PM;
}

//
// Gait engine overrides
//

// Reset function
void FeedGait::reset()
{
	// Reset the dynamic objects
	resetDynamicObjects();

	// Reset walking
	resetWalking();
}

// Step function
void FeedGait::step()
{
	// Clear the plot manager for a new cycle
	m_PM->clear();

	// Update the halt pose
	updateHaltPose();

	// Check whether we should start walking
	if(in.gaitCmd.walk && !m_walking)
		setWalking(true);

	// Update the gait phase information
	updateGaitPhaseInfo();

	// Save the last commanded gait phase
	double oldGaitPhase = m_gaitPhase;

	// Update the gait engine inputs
	GaitInput gaitInput = calcGaitInputs();

	// Update the odometry object
	OdometryInput odomInput = calcOdomInputs(gaitInput);
	O->callUpdate(odomInput);

	// Get the trajectory generation information
	TrajInfo trajInfo;
	T->getInfo(trajInfo);

	// Update the model objects
	ModelInput modelInput;
	if(m_walking)
	{
		modelInput = calcModelInputs(odomInput, gaitInput, trajInfo);
		for(const FeedModelBasePtr& model : m_models)
			model->callUpdate(m_gaitPhaseInfo, modelInput);
	}

	// Update the gait phase based on the required timing
	TimingOutput timingOutput = updateTiming(); // Note: m_gaitPhase is updated here!

	// Update the halt pose blender
	m_haltBlender.update(timingOutput.gaitPhaseInc);

	// Declare variables
	StepSizeOutput stepSizeOutput;
	TrajCommand trajCmd(trajInfo.fusedPitchN);
	ACFlag trajCmdUse = ACFlag::NONE;
	PoseCommand poseCmd = m_haltPose;

	// Check whether we should stop walking
	double stoppingPhaseTol = rc_utils::coerce(timingOutput.gaitPhaseIncMax * config.stoppingPhaseTol(), 0.0, M_PI_4);
	if(m_walking && !in.gaitCmd.walk && m_currentGcv.norm() <= config.stoppingGcvMag() &&
		((m_gaitPhase >= 0.0 && oldGaitPhase <= 0.0 && (m_gaitPhase <= stoppingPhaseTol || oldGaitPhase >= -stoppingPhaseTol)) ||
		(m_gaitPhase <= 0.0 && oldGaitPhase >= 0.0 && (m_gaitPhase <= stoppingPhaseTol - M_PI || oldGaitPhase >= M_PI - stoppingPhaseTol))))
	{
		setWalking(false);
	}

	// Generate the required gait motion
	if(m_walking)
	{
		// Update the current gait command vector (m_currentGcv) based on the model step sizes
		stepSizeOutput = updateStepSize(gaitInput, timingOutput);

		// Update the current gait command acceleration (m_currentGcvLFAcc) based on the current gait command vector command
		updateGcvAcc();

		// Construct the required trajectory generation command based on the current gait command vector and model actions commands
		trajCmd.gcv = m_currentGcv;
		trajCmd.gcvLFAcc = m_currentGcvLFAcc;
		if(config.cmdUseActions())
		{
			for(const FeedModelBasePtr& model : m_models)
				model->writeActions(trajCmd, trajCmdUse);
		}

		// Regenerate the trajectory and evaluate it at the current gait phase
		T->generate(trajCmd);
		T->evaluate(m_gaitPhase, poseCmd);

		// Start blending to the generated pose command if necessary
		if((!m_haltBlender.blending() && m_haltBlender.currentFactor() != GenBlendFactor) || m_haltBlender.targetFactor() != GenBlendFactor)
			m_haltBlender.setTargetScaled(GenBlendFactor, config.blendingStartPhaseLen());

		// Apply the halt pose blender
		m_haltBlender.setSource(poseCmd);
		m_haltBlender.blend(poseCmd, m_haltPose);
	}
	else
	{
		// Update the current gait command acceleration (m_currentGcvLFAcc) based on the current gait command vector command
		updateGcvAcc();

		// Start blending to the halt pose if necessary
		if((!m_haltBlender.blending() && m_haltBlender.currentFactor() != HaltBlendFactor) || m_haltBlender.targetFactor() != HaltBlendFactor)
			m_haltBlender.setTargetScaled(HaltBlendFactor, config.blendingStopPhaseLen());

		// Apply the halt pose blender
		m_haltBlender.blendSource(m_haltPose, poseCmd);
	}

	// Update the gait engine outputs
	updateGaitOutputs(poseCmd);

	// Plotting
	if(m_PM->getEnabled())
	{
		m_PM->plotScalar(m_walking, PM_FG_WALKING);
		m_PM->plotScalar(m_gaitPhaseInfo.gaitPhase, PM_FG_GPI_GAITPHASE);
		m_PM->plotScalar(m_gaitPhaseInfo.elapGaitPhase, PM_FG_GPI_ELAPGAITPHASE);
		m_PM->plotScalar(m_gaitPhaseInfo.remGaitPhase, PM_FG_GPI_REMGAITPHASE);
		m_PM->plotScalar(hk::limbSignOf(m_gaitPhaseInfo.suppLegIndex), PM_FG_GPI_SUPPLEGINDEX);
		m_PM->plotScalar(gaitInput.nominaldT, PM_FG_GI_NOMINALDT);
		m_PM->plotScalar(gaitInput.truedT, PM_FG_GI_TRUEDT);
		m_PM->plotScalar(gaitInput.robotOrient.fusedYaw, PM_FG_GI_FUSEDYAW);
		m_PM->plotScalar(gaitInput.robotOrient.fusedPitch, PM_FG_GI_FUSEDPITCH);
		m_PM->plotScalar(gaitInput.robotOrient.fusedRoll, PM_FG_GI_FUSEDROLL);
		m_PM->plotVec3d (gaitInput.inputGcv, PM_FG_GI_INPUTGCVX);
		m_PM->plotScalar(trajInfo.dblSuppPhaseLen, PM_FG_TI_DBLSUPPPHASELEN);
		m_PM->plotScalar(trajInfo.fusedPitchN, PM_FG_TI_FUSEDPITCHN);
		m_PM->plotScalar(trajInfo.hipHeightMin, PM_FG_TI_HIPHEIGHT_MIN);
		m_PM->plotScalar(trajInfo.hipHeightNom, PM_FG_TI_HIPHEIGHT_NOM);
		m_PM->plotScalar(modelInput.nomGaitFrequency, PM_FG_MI_NOMGAITFREQ);
		m_PM->plotScalar(timingOutput.gaitFrequency, PM_FG_TO_GAITFREQ);
		m_PM->plotScalar(timingOutput.timeToStep, PM_FG_TO_TIMETOSTEP);
		m_PM->plotScalar(m_gaitPhase, PM_FG_NEWGAITPHASE);
		m_PM->plotVec3d (stepSizeOutput.targetGcv, PM_FG_SSO_TARGETGCVX);
		m_PM->plotVec3d (trajCmd.gcv, PM_FG_TC_GCVX);
		m_PM->plotVec3d (trajCmd.gcvLFAcc, PM_FG_TC_GCVLFACCX);
		m_PM->plotScalar(trajCmd.fusedPitchS, PM_FG_TC_FUSEDPITCHS);
		m_PM->plotScalar(trajCmd.fusedRollS, PM_FG_TC_FUSEDROLLS);
		m_PM->plotScalar(trajCmd.footTiltCts.tiltAngle, PM_FG_TC_FOOTTILTCTS_A);
		m_PM->plotScalar(trajCmd.footTiltSupp.tiltAngle, PM_FG_TC_FOOTTILTSUPP_A);
		m_PM->plotScalar(trajCmd.swingOut.tiltAngle, PM_FG_TC_SWINGOUT_A);
		m_PM->plotScalar(trajCmd.leanTilt.tiltAngle, PM_FG_TC_LEANTILT_A);
		m_PM->plotVec2d (trajCmd.hipShift, PM_FG_TC_HIPSHIFTX);
		m_PM->plotScalar(trajCmd.hipHeightMax, PM_FG_TC_HIPHEIGHTMAX);
		m_PM->plotScalar(trajCmd.armTilt.tiltAngle, PM_FG_TC_ARMTILT_A);
		m_PM->plotScalar(trajCmdUse.value, PM_FG_TC_ACFLAGS);
		m_PM->plotScalar(poseCmd.suppCoeff[hk::LEFT], PM_FG_PC_SUPPCOEFF_LEFT);
		m_PM->plotScalar(poseCmd.suppCoeff[hk::RIGHT], PM_FG_PC_SUPPCOEFF_RIGHT);
		m_PM->plotScalar(m_haltBlender.blending(), PM_FG_HALT_BLENDING);
		m_PM->plotScalar(m_haltBlender.currentFactor(), PM_FG_HALT_BLEND_FACTOR);
	}

	// Publish the plot data
	m_PM->publish();
}

// Halt pose function
void FeedGait::updateHaltPose()
{
	// Get the required halt pose
	T->getHaltPose(m_haltPose);

	// Transcribe the halt pose positions and efforts
	jointPosEffortKinToGait(m_haltPose, haltJointCmd, haltJointEffort);

	// Set whether to use raw joint commands
	haltUseRawJointCmds = !config.useServoModel();

	// Set the halt pose support coefficients
	haltSupportCoeffLeftLeg = m_haltPose.suppCoeff[hk::LEFT];
	haltSupportCoeffRightLeg = m_haltPose.suppCoeff[hk::RIGHT];
}

// Set odometry function
void FeedGait::setOdometry(double posX, double posY, double rotZ)
{
	// Set the odometry
	O->setPose2D(posX, posY, rotZ);
	out.odomJump = true;
}

// Update odometry function
void FeedGait::updateOdometry()
{
	// Transcribe the position odometry information
	Vec3 trunkPos = O->pos3D();
	out.odomPosition[0] = trunkPos.x();
	out.odomPosition[1] = trunkPos.y();
	out.odomPosition[2] = trunkPos.z();

	// Transcribe the orientation odometry information
	Quat trunkRot = O->rot3D();
	out.odomOrientation[0] = trunkRot.w();
	out.odomOrientation[1] = trunkRot.x();
	out.odomOrientation[2] = trunkRot.y();
	out.odomOrientation[3] = trunkRot.z();
}

// Handle joystick button function
void FeedGait::handleJoystickButton(int button)
{
	// Perform custom actions for the joystick buttons as required
	keypoint_traj::KeypointTrajConfig& ktconfig = keypoint_traj::KeypointTrajConfig::getInstance();
	tilt_phase_model::TiltPhaseModelConfig& tpmconfig = tilt_phase_model::TiltPhaseModelConfig::getInstance();
	switch(config.joystickMode())
	{
		case 0:
			if(button == 9) tpmconfig.enable.set(!tpmconfig.enable());
			break;
		case 1:
			if(button == 9)
			{
				bool newValue = !tpmconfig.enableArmTiltY();
				tpmconfig.enableArmTiltX.set(newValue);
				tpmconfig.enableArmTiltY.set(newValue);
				tpmconfig.enableFootTiltSuppX.set(newValue);
				tpmconfig.enableFootTiltSuppY.set(newValue);
			}
			break;
		case 2:
			if(button == 9)
			{
				bool newValue = (tpmconfig.biasFootTiltCtsX() == 0.0 && tpmconfig.biasFootTiltCtsY() == 0.0);
				tpmconfig.biasFootTiltCtsX.set(0.06 * newValue);
				tpmconfig.biasFootTiltCtsY.set(0.045 * newValue);
			}
			else if(button == 11)
			{
				bool newValue = !tpmconfig.enableFootTiltCtsX();
				tpmconfig.enableFootTiltCtsX.set(newValue);
				tpmconfig.enableFootTiltCtsY.set(newValue);
			}
			else if(button == 12)
			{
				bool newValue = !tpmconfig.enableHipShiftX();
				tpmconfig.enableHipShiftX.set(newValue);
				tpmconfig.enableHipShiftY.set(newValue);
			}
			break;
		case 3:
			if(button == 9)
			{
				bool newValue = !tpmconfig.enableLeanTiltY();
				tpmconfig.enableLeanTiltX.set(newValue);
				tpmconfig.enableLeanTiltY.set(newValue);
			}
			else if(button == 11)
			{
				float oldValue = tpmconfig.biasLeanTiltX(), scale;
				if(oldValue > 0.0) scale = -1.0;
				else if(oldValue < 0.0) scale = 0.0;
				else scale = 1.0;
				tpmconfig.biasLeanTiltX.set(0.08 * scale);
			}
			else if(button == 12)
			{
				float oldValue = tpmconfig.biasLeanTiltY(), scale;
				if(oldValue > 0.0) scale = -1.0;
				else if(oldValue < 0.0) scale = 0.0;
				else scale = 1.0;
				tpmconfig.biasLeanTiltY.set(0.08 * scale);
			}
			break;
		case 4:
			if(button == 9)
			{
				bool newValue = !tpmconfig.enableSwingOutX();
				tpmconfig.enableSwingOutX.set(newValue);
				tpmconfig.enableSwingOutY.set(newValue);
			}
			break;
		case 5:
			if(button == 9) tpmconfig.enablePlaneNSY.set(!tpmconfig.enablePlaneNSY());
			break;
		case 6:
			if(button == 9) tpmconfig.enableTiming.set(!tpmconfig.enableTiming());
			break;
		case 7:
			if(button == 9) tpmconfig.enableHipHeightMax.set(!tpmconfig.enableHipHeightMax());
			break;
		case 8:
			if(button == 9) ktconfig.enableLeanFF.set(!ktconfig.enableLeanFF());
			break;
	}
}

//
// Dynamic objects
//

// Reset all objects that are dynamically typed and instantiated based on configuration parameters
void FeedGait::resetDynamicObjects()
{
	// Update the current dynamic object types
	m_currentKinType = kinematicsTypeFrom(config.kinematicsType());
	m_currentTrajType = trajectoryTypeFrom(config.trajectoryType());
	m_currentOdomType = odometryTypeFrom(config.odometryType());

	// Reset the dynamic objects
	KW = createKinematicsWrapper();
	KI = createKinematicsInterface();
	T = createTrajectory();
	O = createOdometry();
	recreateModels();

	// Update the joint information
	updateJointInformation();

	// Update the halt pose
	updateHaltPose();

	// Indicate that the odometry was reset
	out.odomJump = true;

	// Inform the user about the objects that have been created
	ROS_INFO("Updated dynamic objects: %s kinematics, %s trajectory, %s odometry", kinematicsTypeName(m_currentKinType).c_str(), trajectoryTypeName(m_currentTrajType).c_str(), odometryTypeName(m_currentOdomType).c_str());

	// Plotting
	m_PM->plotScalar(m_currentKinType, PM_FG_CURRENT_KIN_TYPE);
	m_PM->plotScalar(m_currentTrajType, PM_FG_CURRENT_TRAJ_TYPE);
	m_PM->plotScalar(m_currentOdomType, PM_FG_CURRENT_ODOM_TYPE);
}

// Kinematics wrapper class factory function
hk::KinematicsWrapperBasePtr FeedGait::createKinematicsWrapper(KinematicsType kinType) const
{
	// Create the required kinematics wrapper object
	if(kinType == KT_SERIAL)
		return std::make_shared<hk::KinematicsWrapper<serial::SerialKinematics>>();
	else if(kinType == KT_PARALLEL)
	{
		ROS_ERROR("Parallel kinematics (parallel::ParallelKinematics) have not been implemented yet => Using serial kinematics (serial::SerialKinematics) instead!");
		return std::make_shared<hk::KinematicsWrapper<serial::SerialKinematics>>();
	}
	else
	{
		ROS_ERROR("Attempted to create kinematics wrapper class of unknown kinematics type %d => Something is wrong!", kinType);
		return std::make_shared<hk::KinematicsWrapper<serial::SerialKinematics>>();
	}
}

// Kinematics interface class factory function
FeedKinematicsBasePtr FeedGait::createKinematicsInterface(KinematicsType kinType) const
{
	// Create the required kinematics interface object
	if(kinType == KT_SERIAL)
		return std::make_shared<FeedKinematics<serial::SerialKinematics>>();
	else if(kinType == KT_PARALLEL)
	{
		ROS_ERROR("Parallel kinematics (parallel::ParallelKinematics) have not been implemented yet => Using serial kinematics (serial::SerialKinematics) instead!");
		return std::make_shared<FeedKinematics<serial::SerialKinematics>>();
	}
	else
	{
		ROS_ERROR("Attempted to create kinematics interface class of unknown kinematics type %d => Something is wrong!", kinType);
		return std::make_shared<FeedKinematics<serial::SerialKinematics>>();
	}
}

// Trajectory generation class factory function
FeedTrajectoryBasePtr FeedGait::createTrajectory(TrajectoryType trajType, KinematicsType kinType) const
{
	// Create the required trajectory generation object
	if(kinType == KT_SERIAL)
		return createTrajectory<serial::SerialKinematics>(trajType);
	else if(kinType == KT_PARALLEL)
	{
		ROS_ERROR("Parallel kinematics (parallel::ParallelKinematics) have not been implemented yet => Using serial kinematics (serial::SerialKinematics) instead!");
		return createTrajectory<serial::SerialKinematics>(trajType);
	}
	else
	{
		ROS_ERROR("Attempted to create trajectory generation class of unknown kinematics type %d => Something is wrong!", kinType);
		return createTrajectory<serial::SerialKinematics>(trajType);
	}
}

// Odometry class factory function
FeedOdometryBasePtr FeedGait::createOdometry(OdometryType odomType, KinematicsType kinType) const
{
	// Create the required odometry object
	if(kinType == KT_SERIAL)
		return createOdometry<serial::SerialKinematics>(odomType);
	else if(kinType == KT_PARALLEL)
	{
		ROS_ERROR("Parallel kinematics (parallel::ParallelKinematics) have not been implemented yet => Using serial kinematics (serial::SerialKinematics) instead!");
		return createOdometry<serial::SerialKinematics>(odomType);
	}
	else
	{
		ROS_ERROR("Attempted to create odometry class of unknown kinematics type %d => Something is wrong!", kinType);
		return createOdometry<serial::SerialKinematics>(odomType);
	}
}

// Model class factory function
FeedModelBasePtr FeedGait::createModel(ModelType modelType, KinematicsType kinType) const
{
	// Create the required model object
	if(kinType == KT_SERIAL)
		return createModel<serial::SerialKinematics>(modelType);
	else if(kinType == KT_PARALLEL)
	{
		ROS_ERROR("Parallel kinematics (parallel::ParallelKinematics) have not been implemented yet => Using serial kinematics (serial::SerialKinematics) instead!");
		return createModel<serial::SerialKinematics>(modelType);
	}
	else
	{
		ROS_ERROR("Attempted to create model class of unknown kinematics type %d => Something is wrong!", kinType);
		return createModel<serial::SerialKinematics>(modelType);
	}
}

// Recreate model classes function
void FeedGait::recreateModels(KinematicsType kinType)
{
	// Clear the current vector of models
	m_models.clear();

	// Create the required new models
	for(int m = MT_FIRST; m < MT_COUNT; m++)
		m_models.emplace_back(createModel((ModelType) m, kinType));
}

// Update the joint information
void FeedGait::updateJointInformation()
{
	// Retrieve the joint information from the kinematics wrapper class
	KW->getJointInfo(m_jointInfo);

	// Regenerate the joint index map from Gait to Kinematics order
	m_jointIndexMapGK.clear();
	for(std::size_t g = 0; g < gait::NUM_JOINTS; g++)
	{
		const std::string& gname = gait::jointName[g];
		JointInfoVec::const_iterator it = std::find_if(m_jointInfo.cbegin(), m_jointInfo.cend(), [&gname](const hk::JointInfo& item) { return item.jointName == gname; });
		if(it == m_jointInfo.end())
		{
			m_jointIndexMapGK[g] = NullJointIndex;
			ROS_WARN("Failed to find a Kinematics joint to map to Gait joint %u (%s)!", (unsigned) g, gname.c_str());
		}
		else
		{
			std::size_t k = std::distance(m_jointInfo.cbegin(), it);
			m_jointIndexMapGK[g] = k;
		}
	}

	// Regenerate the joint index map from Kinematics to Gait order
	m_jointIndexMapKG.clear();
	const std::string* jbegin = std::begin(gait::jointName);
	const std::string* jend = std::end(gait::jointName);
	for(std::size_t k = 0; k < m_jointInfo.size(); k++)
	{
		const hk::JointInfo& jinfo = m_jointInfo[k];
		const std::string& kname = jinfo.jointName;
		const std::string* it = std::find(jbegin, jend, kname);
		if(it == jend)
		{
			m_jointIndexMapKG[k] = NullJointIndex;
			if(jinfo.limbType == hk::LT_LEG || jinfo.limbType == hk::LT_ARM)
				ROS_WARN("Failed to find a Gait joint to map to Kinematics joint %u (%s)!", (unsigned) k, kname.c_str());
		}
		else
		{
			std::size_t g = std::distance(jbegin, it);
			m_jointIndexMapKG[k] = g;
		}
	}
}

// Convert joint positions from Gait to Kinematics order
void FeedGait::jointPosGaitToKin(const double (&gaitPos)[gait::NUM_JOINTS], std::vector<double>& kinPos) const
{
	// Convert the joint positions as required
	std::size_t numKin = numKinJoints();
	kinPos.resize(numKin);
	for(std::size_t k = 0; k < numKin; k++)
	{
		std::size_t g = m_jointIndexMapKG.at(k);
		kinPos[k] = (g != NullJointIndex && g < gait::NUM_JOINTS ? gaitPos[g] : 0.0);
	}
}

// Convert joint positions from Kinematics to Gait order
void FeedGait::jointPosKinToGait(const std::vector<double>& kinPos, double (&gaitPos)[gait::NUM_JOINTS]) const
{
	// Convert the joint positions as required
	for(std::size_t g = 0; g < gait::NUM_JOINTS; g++)
	{
		std::size_t k = m_jointIndexMapGK.at(g);
		gaitPos[g] = (k != NullJointIndex && k < kinPos.size() ? kinPos[k] : 0.0);
	}
}

// Convert joint efforts from Gait to Kinematics order
void FeedGait::jointEffortGaitToKin(const double (&gaitEffort)[gait::NUM_JOINTS], std::vector<double>& kinEffort) const
{
	// Convert the joint efforts as required
	std::size_t numKin = numKinJoints();
	kinEffort.resize(numKin);
	for(std::size_t k = 0; k < numKin; k++)
	{
		std::size_t g = m_jointIndexMapKG.at(k);
		kinEffort[k] = (g != NullJointIndex && g < gait::NUM_JOINTS ? gaitEffort[g] : config.effortDefault());
	}
}

// Convert joint efforts from Kinematics to Gait order
void FeedGait::jointEffortKinToGait(const std::vector<double>& kinEffort, double (&gaitEffort)[gait::NUM_JOINTS]) const
{
	// Convert the joint efforts as required
	for(std::size_t g = 0; g < gait::NUM_JOINTS; g++)
	{
		std::size_t k = m_jointIndexMapGK.at(g);
		gaitEffort[g] = (k != NullJointIndex && k < kinEffort.size() ? kinEffort[k] : config.effortDefault());
	}
}

// Convert joint positions and efforts from Gait to Kinematics order
void FeedGait::jointPosEffortGaitToKin(const double (&gaitPos)[gait::NUM_JOINTS], const double (&gaitEffort)[gait::NUM_JOINTS], std::vector<double>& kinPos, std::vector<double>& kinEffort) const
{
	// Convert the joint positions and efforts as required
	std::size_t numKin = numKinJoints();
	kinPos.resize(numKin);
	kinEffort.resize(numKin);
	for(std::size_t k = 0; k < numKin; k++)
	{
		std::size_t g = m_jointIndexMapKG.at(k);
		if(g != NullJointIndex && g < gait::NUM_JOINTS)
		{
			kinPos[k] = gaitPos[g];
			kinEffort[k] = gaitEffort[g];
		}
		else
		{
			kinPos[k] = 0.0;
			kinEffort[k] = config.effortDefault();
		}
	}
}

// Convert joint positions and efforts from Kinematics to Gait order
void FeedGait::jointPosEffortKinToGait(const std::vector<double>& kinPos, const std::vector<double>& kinEffort, double (&gaitPos)[gait::NUM_JOINTS], double (&gaitEffort)[gait::NUM_JOINTS]) const
{
	// Convert the joint positions and efforts as required
	for(std::size_t g = 0; g < gait::NUM_JOINTS; g++)
	{
		std::size_t k = m_jointIndexMapGK.at(g);
		gaitPos[g] = (k != NullJointIndex && k < kinPos.size() ? kinPos[k] : 0.0);
		gaitEffort[g] = (k != NullJointIndex && k < kinEffort.size() ? kinEffort[k] : config.effortDefault());
	}
}

//
// Step functions
//

// Calculate gait inputs function
GaitInput FeedGait::calcGaitInputs() const
{
	// Declare variables
	GaitInput gaitInput;

	// Transcribe the time increment variables
	gaitInput.truedT = in.truedT;
	gaitInput.nominaldT = in.nominaldT;
	gaitInput.timestamp = in.timestamp;

	// Calculate the joint positions
	jointPosGaitToKin(in.jointPos, gaitInput.jointPos);

	// Calculate the robot orientation
	gaitInput.robotOrient.fusedYaw = model->robotFYawPR();
	gaitInput.robotOrient.fusedPitch = model->robotFPitchPR() + config.fusedPitchOffset();
	gaitInput.robotOrient.fusedRoll = model->robotFRollPR() + config.fusedRollOffset();

	// Calculate the gait command vector variables
	if(in.gaitCmd.walk)
		gaitInput.inputGcv << in.gaitCmd.linVelX, in.gaitCmd.linVelY, in.gaitCmd.angVelZ;
	else
		gaitInput.inputGcv.setZero();

	// Return the required gait inputs
	return gaitInput;
}

// Calculate odometry inputs function
OdometryInput FeedGait::calcOdomInputs(const GaitInput& gaitInput) const
{
	// Declare variables
	OdometryInput odomInput;

	// Calculate the odometry inputs as required
	odomInput.CommonInput::operator=(gaitInput);

	// Return the required odometry inputs
	return odomInput;
}

// Calculate model inputs function
ModelInput FeedGait::calcModelInputs(const OdometryInput& odomInput, const GaitInput& gaitInput, const TrajInfo& trajInfo) const
{
	// Declare variables
	ModelInput modelInput;

	// Calculate the model inputs as required
	modelInput.OdometryInput::operator=(odomInput);
	modelInput.inputGcv = gaitInput.inputGcv;
	modelInput.nomGaitFrequency = config.gaitFrequency();
	modelInput.trajInfo = trajInfo;

	// Return the required model inputs
	return modelInput;
}

// Update timing function
TimingOutput FeedGait::updateTiming()
{
	// Declare variables
	TimingOutput timingOutput;
	TimingCommand timingCmd;

	// Calculate the required gait frequency
	double gaitFrequency = config.gaitFrequency();
	if(m_walking && config.cmdUseTiming())
	{
		for(const FeedModelBasePtr& model : m_models)
		{
			if(!model->useTiming()) continue;
			timingCmd = model->getTiming();
			gaitFrequency = timingCmd.gaitFrequency;
		}
	}

	// Coerce the gait frequency to the allowed range
	timingOutput.gaitFrequency = rc_utils::coerce<double>(gaitFrequency, MinGaitFrequency, config.gaitFrequencyMax());

	// Calculate the required time to step
	timingOutput.timeToStep = rc_utils::coerceMin(m_gaitPhaseInfo.remGaitPhase / (M_PI * timingOutput.gaitFrequency), 0.0);

	// Update the gait phase with the appropriate phase increment
	timingOutput.gaitPhaseInc = M_PI * in.nominaldT * timingOutput.gaitFrequency;
	timingOutput.gaitPhaseIncMax = M_PI * in.nominaldT * config.gaitFrequencyMax();
	m_gaitPhase = rc_utils::picut(m_gaitPhase + timingOutput.gaitPhaseInc);

	// Plotting
	if(m_PM->getEnabled())
		m_PM->plotScalar(timingCmd.gaitFrequency, PM_FG_TC_GAITFREQ);

	// Return the required timing outputs
	return timingOutput;
}

// Update step size function
StepSizeOutput FeedGait::updateStepSize(const GaitInput& gaitInput, const TimingOutput& timingOutput)
{
	// Declare variables
	StepSizeOutput stepSizeOutput;

	// Initialise the step size command to low frequency slope limiting of the input gait command vector
	StepSizeCommand stepSizeCommand;
	stepSizeCommand.X.set(gaitInput.inputGcv.x(), 0.0, 0.0);
	stepSizeCommand.Y.set(gaitInput.inputGcv.y(), 0.0, 0.0);
	stepSizeCommand.Z.set(gaitInput.inputGcv.z(), 0.0, 0.0);

	// Calculate the required step size
	StepSizeCommand stepSizeCommandModel;
	if(config.cmdUseStepSize())
	{
		for(const FeedModelBasePtr& model : m_models)
		{
			if(model->useStepSize())
				model->getStepSize().writeTo(stepSizeCommandModel);
		}
	}
	if(!config.cmdUseStepSizeX()) stepSizeCommandModel.X.use = false;
	if(!config.cmdUseStepSizeY()) stepSizeCommandModel.Y.use = false;
	if(!config.cmdUseStepSizeZ()) stepSizeCommandModel.Z.use = false;
	stepSizeCommandModel.writeTo(stepSizeCommand);

	// Update the low frequency slope limited gait command vector
	double D = config.gcvSlopeLFDecToAcc();
	if(m_currentGcvCmd.gcvLF.x() >= 0.0) m_currentGcvCmd.gcvLF.x() += rc_utils::coerce(stepSizeCommand.X.gcvLF - m_currentGcvCmd.gcvLF.x(), -in.truedT*config.gcvSlopeLFForwards()*D, in.truedT*config.gcvSlopeLFForwards()   );
	else                                 m_currentGcvCmd.gcvLF.x() += rc_utils::coerce(stepSizeCommand.X.gcvLF - m_currentGcvCmd.gcvLF.x(), -in.truedT*config.gcvSlopeLFBackwards() , in.truedT*config.gcvSlopeLFBackwards()*D);
	if(m_currentGcvCmd.gcvLF.y() >= 0.0) m_currentGcvCmd.gcvLF.y() += rc_utils::coerce(stepSizeCommand.Y.gcvLF - m_currentGcvCmd.gcvLF.y(), -in.truedT*config.gcvSlopeLFSidewards()*D, in.truedT*config.gcvSlopeLFSidewards()  );
	else                                 m_currentGcvCmd.gcvLF.y() += rc_utils::coerce(stepSizeCommand.Y.gcvLF - m_currentGcvCmd.gcvLF.y(), -in.truedT*config.gcvSlopeLFSidewards()  , in.truedT*config.gcvSlopeLFSidewards()*D);
	if(m_currentGcvCmd.gcvLF.z() >= 0.0) m_currentGcvCmd.gcvLF.z() += rc_utils::coerce(stepSizeCommand.Z.gcvLF - m_currentGcvCmd.gcvLF.z(), -in.truedT*config.gcvSlopeLFRotational()*D, in.truedT*config.gcvSlopeLFRotational()  );
	else                                 m_currentGcvCmd.gcvLF.z() += rc_utils::coerce(stepSizeCommand.Z.gcvLF - m_currentGcvCmd.gcvLF.z(), -in.truedT*config.gcvSlopeLFRotational()  , in.truedT*config.gcvSlopeLFRotational()*D);

	// Update the high frequency slope limited gait command vector
	m_currentGcvCmd.gcvHF.x() += rc_utils::coerceAbs(stepSizeCommand.X.gcvHF - m_currentGcvCmd.gcvHF.x(), in.truedT*config.gcvSlopeHFForwards());
	m_currentGcvCmd.gcvHF.y() += rc_utils::coerceAbs(stepSizeCommand.Y.gcvHF - m_currentGcvCmd.gcvHF.y(), in.truedT*config.gcvSlopeHFSidewards());
	m_currentGcvCmd.gcvHF.z() += rc_utils::coerceAbs(stepSizeCommand.Z.gcvHF - m_currentGcvCmd.gcvHF.z(), in.truedT*config.gcvSlopeHFRotational());

	// Update the high frequency slope limited end of step gait command vector
	double u = (timingOutput.timeToStep > in.nominaldT ? in.nominaldT / timingOutput.timeToStep : 1.0);
	m_currentGcvCmd.gcvEOS.x() += rc_utils::coerceAbs(u*(stepSizeCommand.X.gcvEOS - m_currentGcvCmd.gcvEOS.x()), in.truedT*config.gcvSlopeHFForwards());
	m_currentGcvCmd.gcvEOS.y() += rc_utils::coerceAbs(u*(stepSizeCommand.Y.gcvEOS - m_currentGcvCmd.gcvEOS.y()), in.truedT*config.gcvSlopeHFSidewards());
	m_currentGcvCmd.gcvEOS.z() += rc_utils::coerceAbs(u*(stepSizeCommand.Z.gcvEOS - m_currentGcvCmd.gcvEOS.z()), in.truedT*config.gcvSlopeHFRotational());

	// Update the current gait command vector
	m_currentGcv = m_currentGcvCmd.totalGcv();

	// Calculate the step size outputs
	stepSizeOutput.targetGcv = stepSizeCommand.targetGcv();

	// Plotting
	if(m_PM->getEnabled())
	{
		m_PM->plotScalar(stepSizeCommandModel.X.use, PM_FG_SSC_USEMODELCMDX);
		m_PM->plotScalar(stepSizeCommandModel.Y.use, PM_FG_SSC_USEMODELCMDY);
		m_PM->plotScalar(stepSizeCommandModel.Z.use, PM_FG_SSC_USEMODELCMDZ);
		m_PM->plotVec3d (m_currentGcvCmd.gcvLF, PM_FG_SSC_GCVLFX);
		m_PM->plotVec3d (m_currentGcvCmd.gcvHF, PM_FG_SSC_GCVHFX);
		m_PM->plotVec3d (m_currentGcvCmd.gcvEOS, PM_FG_SSC_GCVEOSX);
	}

	// Return the required step size output
	return stepSizeOutput;
}

// Update gait command acceleration function
void FeedGait::updateGcvAcc()
{
	// Resize the gait command acceleration filter
	std::size_t gcvLFAccFilterN = rc_utils::coerceMin(config.gcvLFAccFilterN(), 1);
	if(m_gcvLFAccFilter.len() != gcvLFAccFilterN) m_gcvLFAccFilter.resize(gcvLFAccFilterN);

	// Update the current gait command acceleration
	m_gcvLFAccFilter.addPW(in.timestamp, m_currentGcvCmd.gcvLF, 1.0).update();
	m_currentGcvLFAcc = m_gcvLFAccFilter.deriv();
}

// Update gait outputs function
void FeedGait::updateGaitOutputs(const PoseCommand& poseCmd)
{
	// Transcribe the commanded positions and efforts
	jointPosEffortKinToGait(poseCmd, out.jointCmd, out.jointEffort);

	// Transcribe whether to use raw joint commands
	out.useRawJointCmds = haltUseRawJointCmds;

	// Transcribe the commanded support coefficients
	out.supportCoeffLeftLeg = poseCmd.suppCoeff[hk::LEFT];
	out.supportCoeffRightLeg = poseCmd.suppCoeff[hk::RIGHT];

	// Set whether the robot is walking
	out.walking = (m_walking || m_haltBlender.blending());

	// Update the odometry outputs
	updateOdometry();
}

//
// Walking
//

// Reset function for walking
void FeedGait::resetWalking()
{
	// Set walking off
	setWalking(false);

	// Reset the odometry
	O->reset();
	out.odomJump = true;

	// Reset the halt pose blender
	m_haltBlender.reset(HaltBlendFactor, m_haltPose);
}

// Set function for walking
void FeedGait::setWalking(bool walking)
{
	// Set the walking flags
	m_walking = walking;

	// Set the gait phase variables
	m_leftLegFirst = config.leftLegFirst();
	m_gaitPhase = (m_leftLegFirst ? 0.0 : M_PI);
	updateGaitPhaseInfo();

	// Set the gait command vector variables
	m_currentGcvCmd.reset();
	m_currentGcv.setZero();

	// Set the gait command acceleration variables
	m_gcvLFAccFilter.resetAll();
	m_currentGcvLFAcc.setZero();

	// Reset the odometry if walking is starting
	if(walking)
	{
		O->reset();
		out.odomJump = true;
	}

	// Reset the models
	for(const FeedModelBasePtr& model : m_models)
		model->reset();
}

//
// Plotting
//

// Callback for when the plotData parameter is updated
void FeedGait::callbackPlotData()
{
	// Enable or disable plotting as required
	if(config.plotData())
		m_PM->enable();
	else
		m_PM->disable();
}

PLUGINLIB_EXPORT_CLASS(feed_gait::FeedGait, gait::GaitEngine)
// EOF