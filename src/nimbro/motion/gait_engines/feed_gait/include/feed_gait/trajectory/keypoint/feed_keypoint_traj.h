// Feedback gait keypoint trajectory generation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_KEYPOINT_TRAJ_H
#define FEED_KEYPOINT_TRAJ_H

// Includes
#include <feed_gait/feed_common.h>
#include <feed_gait/trajectory/feed_trajectory_base.h>
#include <feed_gait/trajectory/keypoint/feed_keypoint_traj_common.h>
#include <feed_gait/trajectory/keypoint/feed_kinematics_kt.h>
#include <humanoid_kinematics/robot_kinematics.h>
#include <config_server/parameter.h>
#include <type_traits>

// Feedback gait namespace
namespace feed_gait
{
	/**
	* @namespace keypoint_traj
	* 
	* @brief Keypoint trajectory generation namespace.
	**/
	namespace keypoint_traj
	{
		/**
		* @class KeypointTrajConfig
		*
		* @brief Keypoint trajectory generation configuration parameters class.
		**/
		class KeypointTrajConfig
		{
		private:
			// Constructor
			KeypointTrajConfig()
				: TYPE_NAME(trajectoryTypeName(TT_KEYPOINT))
				, CONFIG_PARAM_PATH(TRAJ_CONFIG_PARAM_PATH + TYPE_NAME + "/")

				, enableGcv             (CONFIG_PARAM_PATH + "enable/enableGcv", false)
				, enableGcvAcc          (CONFIG_PARAM_PATH + "enable/enableGcvAcc", false)
				, enableFusedPitchS     (CONFIG_PARAM_PATH + "enable/enableFusedPitchS", false)
				, enableFusedRollS      (CONFIG_PARAM_PATH + "enable/enableFusedRollS", false)
				, enableFootTilt        (CONFIG_PARAM_PATH + "enable/enableFootTilt", false)
				, enableSwingOut        (CONFIG_PARAM_PATH + "enable/enableSwingOut", false)
				, enableLean            (CONFIG_PARAM_PATH + "enable/enableLean", false)
				, enableLeanFF          (CONFIG_PARAM_PATH + "enable/enableLeanFF", false)
				, enableHipXYZ          (CONFIG_PARAM_PATH + "enable/enableHipXYZ", false)
				, enableArmTilt         (CONFIG_PARAM_PATH + "enable/enableArmTilt", false)

				, gcvBiasLinVelX        (CONFIG_PARAM_PATH + "gcvBias/linVelX", -0.4, 0.01, 0.4, 0.0)
				, gcvBiasLinVelY        (CONFIG_PARAM_PATH + "gcvBias/linVelY", -0.4, 0.01, 0.4, 0.0)
				, gcvBiasAngVelZ        (CONFIG_PARAM_PATH + "gcvBias/angVelZ", -0.4, 0.01, 0.4, 0.0)
				, gcvPrescalerLinVelX   (CONFIG_PARAM_PATH + "gcvPrescaler/linVelX", 0.0, 0.05, 4.0, 1.0)
				, gcvPrescalerLinVelY   (CONFIG_PARAM_PATH + "gcvPrescaler/linVelY", 0.0, 0.05, 4.0, 1.0)
				, gcvPrescalerAngVelZ   (CONFIG_PARAM_PATH + "gcvPrescaler/angVelZ", 0.0, 0.05, 4.0, 1.0)
				, gcvInternalMaxLinVelX (CONFIG_PARAM_PATH + "gcvInternalMax/linVelX", 0.0, 0.1, 6.0, 2.0)
				, gcvInternalMaxLinVelY (CONFIG_PARAM_PATH + "gcvInternalMax/linVelY", 0.0, 0.1, 6.0, 2.0)
				, gcvInternalMaxAngVelZ (CONFIG_PARAM_PATH + "gcvInternalMax/angVelZ", 0.0, 0.1, 6.0, 2.0)

				, leanFFGainGcvX        (CONFIG_PARAM_PATH + "leanFeedForward/gainGcvX", 0.0, 0.01, 1.0, 0.0)
				, leanFFGainGcvAbsZ     (CONFIG_PARAM_PATH + "leanFeedForward/gainGcvAbsZ", 0.0, 0.01, 1.0, 0.0)
				, leanFFGainGcvAccX     (CONFIG_PARAM_PATH + "leanFeedForward/gainGcvAccX", 0.0, 0.01, 1.0, 0.0)
				, leanFFPhaseMaxAbsY    (CONFIG_PARAM_PATH + "leanFeedForward/phaseMaxAbsY", 0.0, 0.01, 0.5, 0.1)
				, leanFFPhaseBuf        (CONFIG_PARAM_PATH + "leanFeedForward/phaseBuf", 0.0, 0.002, 0.2, 0.02)

				, phaseDoubleSupportLen (CONFIG_PARAM_PATH + "phase/doubleSupportLen", 0.1, 0.01, 1.5, 0.6)
				, phaseOffsetDE         (CONFIG_PARAM_PATH + "phase/offsetDE", 0.05, 0.01, 0.8, 0.3)
				, phaseOffsetAG         (CONFIG_PARAM_PATH + "phase/offsetAG", 0.05, 0.01, 0.8, 0.3)

				, fusedPitchN           (CONFIG_PARAM_PATH + "general/fusedPitchN", -0.2, 0.005, 0.2, 0.0)
				, ssgType               (CONFIG_PARAM_PATH + "stepSizeGenerator/Type", SSGT_FIRST, 1, SSGT_COUNT - 1, SSGT_DEFAULT)
				, abmType               (CONFIG_PARAM_PATH + "armBaseMotion/Type", ABMT_FIRST, 1, ABMT_COUNT - 1, ABMT_DEFAULT)

				, legPlaneRatioS        (CONFIG_PARAM_PATH + "leg/planeRatio/S", 0.0, 0.01, 1.0, 1.0)
				, legPlaneRatioI        (CONFIG_PARAM_PATH + "leg/planeRatio/I", 0.0, 0.01, 1.0, 0.5)
				, legPlaneRatioJ        (CONFIG_PARAM_PATH + "leg/planeRatio/J", 0.0, 0.01, 1.0, 0.5)
				, legMCLHipCentreRatio  (CONFIG_PARAM_PATH + "leg/MCLHipCentreRatio", 0.0, 0.01, 1.0, 0.0)
				, legACAdjustABRatio    (CONFIG_PARAM_PATH + "leg/ACAdjustABRatio", 0.0, 0.01, 1.0, 0.5)
				, legFootTiltIsRelRatio (CONFIG_PARAM_PATH + "leg/footTiltIsRelRatio", 0.0, 0.01, 1.0, 0.5)
				, legFOverNRatio        (CONFIG_PARAM_PATH + "leg/FOverNRatio", 0.0, 0.01, 1.0, 0.6)
				, legHeightRatioE       (CONFIG_PARAM_PATH + "leg/heightRatio/E", 0.0, 0.01, 1.0, 0.2)
				, legHeightRatioG       (CONFIG_PARAM_PATH + "leg/heightRatio/G", 0.0, 0.01, 1.0, 0.2)
				, legSwingOutMaxIwd     (CONFIG_PARAM_PATH + "leg/swingOut/maxIwd", 0.0, 0.001, 0.1, 0.03)
				, legSwingOutBuf        (CONFIG_PARAM_PATH + "leg/swingOut/buf", 0.0, 0.001, 0.1, 0.03)
				, legHipHeightMin       (CONFIG_PARAM_PATH + "leg/hipHeight/min", 0.3, 0.01, 0.9, 0.5)
				, legHipHeightNom       (CONFIG_PARAM_PATH + "leg/hipHeight/nom", 0.5, 0.01, 1.0, 0.95)
				, legSuppCoeffPhaseExtra(CONFIG_PARAM_PATH + "leg/suppCoeff/phaseExtra", 0.0, 0.01, 1.0, 0.25)
				, legSuppCoeffRange     (CONFIG_PARAM_PATH + "leg/suppCoeff/range", 0.0, 0.01, 1.0, 1.0)

				, armArmTiltAngleMaxAbs (CONFIG_PARAM_PATH + "arm/armTiltAngle/maxAbs", 0.5, 0.01, 2.0, 1.5)
				, armArmTiltAngleBuf    (CONFIG_PARAM_PATH + "arm/armTiltAngle/buf", 0.0, 0.005, 0.3, 0.15)
				, armCoMRayTiltMaxP     (CONFIG_PARAM_PATH + "arm/CoMRayTilt/maxP", 0.5, 0.01, 1.5, 1.4)
				, armCoMRayTiltMaxR     (CONFIG_PARAM_PATH + "arm/CoMRayTilt/maxR", 0.5, 0.01, 1.5, 1.2)
				, armCoMRayTiltBuf      (CONFIG_PARAM_PATH + "arm/CoMRayTilt/buf", 0.0, 0.005, 0.3, 0.15)
				, armCoMRayYMaxIwd      (CONFIG_PARAM_PATH + "arm/CoMRayY/maxIwd", -0.2, 0.005, 0.1, 0.0)
				, armCoMRayYBuf         (CONFIG_PARAM_PATH + "arm/CoMRayY/buf", 0.0, 0.005, 0.3, 0.1)

				, tuningForceEvalHalt   (CONFIG_PARAM_PATH + "tuning/forceEvalHaltPose", false)
				, tuningNoLegs          (CONFIG_PARAM_PATH + "tuning/noLegs", false)
				, tuningNoLegTrajectory (CONFIG_PARAM_PATH + "tuning/noLegTrajectory", false)
				, tuningNoLegEfforts    (CONFIG_PARAM_PATH + "tuning/noLegEfforts", false)
				, tuningNoLegSuppCoeff  (CONFIG_PARAM_PATH + "tuning/noLegSuppCoeff", false)
				, tuningNoArms          (CONFIG_PARAM_PATH + "tuning/noArms", false)
				, tuningNoArmTrajectory (CONFIG_PARAM_PATH + "tuning/noArmTrajectory", false)
				, tuningNoArmEfforts    (CONFIG_PARAM_PATH + "tuning/noArmEfforts", false)

				, debugPrintGenLeg      (CONFIG_PARAM_PATH + "debug/printGenLeg", false)
				, debugPrintEvalArm     (CONFIG_PARAM_PATH + "debug/printEvalArm", false)
				, debugPrintPrecision   (CONFIG_PARAM_PATH + "debug/printPrecision", 0, 1, 16, 4)
			{
				// Configure gait command vector bias callback
				boost::function<void (const float&)> gcvBiasCBFunc = boost::bind(&KeypointTrajConfig::gcvBiasCB, this);
				gcvBiasLinVelX.setCallback(gcvBiasCBFunc);
				gcvBiasLinVelY.setCallback(gcvBiasCBFunc);
				gcvBiasAngVelZ.setCallback(gcvBiasCBFunc);
				gcvBiasCB();

				// Configure the double support phase length callback
				phaseDoubleSupportLen.setCallback(boost::bind(&KeypointTrajConfig::phaseDoubleSupportLenCB, this), true);
			}

			// Ensure class remains a singleton
			KeypointTrajConfig(const KeypointTrajConfig&) = delete;
			KeypointTrajConfig& operator=(const KeypointTrajConfig&) = delete;

		public:
			// Get singleton instance of class
			static KeypointTrajConfig& getInstance() { static thread_local KeypointTrajConfig ktconfig; return ktconfig; }

			// Static constants
			static const double MinPhaseSep;

			// Constants
			const std::string TYPE_NAME;
			const std::string CONFIG_PARAM_PATH;

			// Enable parameters
			config_server::Parameter<bool>  enableGcv;              //!< @brief Boolean flag whether to enable the gcv command input (otherwise zero)
			config_server::Parameter<bool>  enableGcvAcc;           //!< @brief Boolean flag whether to enable the gcv acceleration command input (otherwise zero)
			config_server::Parameter<bool>  enableFusedPitchS;      //!< @brief Boolean flag whether to enable the pitch component of the swing ground plane input (otherwise same as nominal plane)
			config_server::Parameter<bool>  enableFusedRollS;       //!< @brief Boolean flag whether to enable the roll component of the swing ground plane input (otherwise same as nominal plane, i.e. zero)
			config_server::Parameter<bool>  enableFootTilt;         //!< @brief Boolean flag whether to enable foot tilting (otherwise zero)
			config_server::Parameter<bool>  enableSwingOut;         //!< @brief Boolean flag whether to enable swing out (otherwise zero)
			config_server::Parameter<bool>  enableLean;             //!< @brief Boolean flag whether to enable leaning (otherwise zero)
			config_server::Parameter<bool>  enableLeanFF;           //!< @brief Boolean flag whether to enable the feedforward component of leaning (otherwise zero)
			config_server::Parameter<bool>  enableHipXYZ;           //!< @brief Boolean flag whether to enable hip shifting and height limiting (otherwise zero shift and no limiting)
			config_server::Parameter<bool>  enableArmTilt;          //!< @brief Boolean flag whether to enable arm tilt (otherwise zero)

			// Gait command vector parameters
			config_server::Parameter<float> gcvBiasLinVelX;         //!< @brief Bias added to the linear x-velocity component of the gait command vector
			config_server::Parameter<float> gcvBiasLinVelY;         //!< @brief Bias added to the linear y-velocity component of the gait command vector
			config_server::Parameter<float> gcvBiasAngVelZ;         //!< @brief Bias added to the angular z-velocity component of the gait command vector
			config_server::Parameter<float> gcvPrescalerLinVelX;    //!< @brief Prescaler for the gcv linear velocity X (allows easy scaling of the dynamic range of the gait)
			config_server::Parameter<float> gcvPrescalerLinVelY;    //!< @brief Prescaler for the gcv linear velocity Y (allows easy scaling of the dynamic range of the gait)
			config_server::Parameter<float> gcvPrescalerAngVelZ;    //!< @brief Prescaler for the gcv angular velocity Z (allows easy scaling of the dynamic range of the gait)
			config_server::Parameter<float> gcvInternalMaxLinVelX;  //!< @brief Absolute maximum allowed value for the used internal linear x-velocity component of the gait command vector
			config_server::Parameter<float> gcvInternalMaxLinVelY;  //!< @brief Absolute maximum allowed value for the used internal linear y-velocity component of the gait command vector
			config_server::Parameter<float> gcvInternalMaxAngVelZ;  //!< @brief Absolute maximum allowed value for the used internal angular z-velocity component of the gait command vector

			// Feedforward leaning parameters
			config_server::Parameter<float> leanFFGainGcvX;         //!< @brief Gain for the feedforward leaning component based on gcv X
			config_server::Parameter<float> leanFFGainGcvAbsZ;      //!< @brief Gain for the feedforward leaning component based on the absolute gcv Z
			config_server::Parameter<float> leanFFGainGcvAccX;      //!< @brief Gain for the feedforward leaning component based on the gcv acceleration X
			config_server::Parameter<float> leanFFPhaseMaxAbsY;     //!< @brief Maximum absolute feedforward leaning in the phase Y direction
			config_server::Parameter<float> leanFFPhaseBuf;         //!< @brief Soft coercion buffer for the feedforward leaning phase

			// Phase parameters
			config_server::Parameter<float> phaseDoubleSupportLen;  //!< @brief Phase length of the assumed double support phase
			config_server::Parameter<float> phaseOffsetDE;          //!< @brief Phase offset from D forwards to E
			config_server::Parameter<float> phaseOffsetAG;          //!< @brief Phase offset from A backwards to G

			// General parameters
			config_server::Parameter<float> fusedPitchN;            //!< @brief Nominal fused pitch of the robot relative to the ground while walking
			config_server::Parameter<int>   ssgType;                //!< @brief Step size generator to use as the basis of the generated trajectory
			config_server::Parameter<int>   abmType;                //!< @brief Arm base motion to use as the basis of the generated trajectory

			// Leg trajectory parameters
			config_server::Parameter<float> legPlaneRatioS;         //!< @brief Ratio to use for interpolation from N to the raw S to define the swing ground plane S (0 = N, 1 = Raw S, for ACAC)
			config_server::Parameter<float> legPlaneRatioI;         //!< @brief Ratio to use for interpolation from N to S to define the intermediate ground plane I (0 = N, 1 = S, for BDBD)
			config_server::Parameter<float> legPlaneRatioJ;         //!< @brief Ratio to use for interpolation from N to S to define the support ground plane J (0 = N, 1 = S, for NN)
			config_server::Parameter<float> legMCLHipCentreRatio;   //!< @brief Ratio of whether the MCL points in the positive z-direction of the N plane or towards the hip centre point (0 = BzN, 1 = Hip centre point)
			config_server::Parameter<float> legACAdjustABRatio;     //!< @brief Ratio of the AC adjustment to be done by AB (0 => On average all AC adjustment is done by CD, 1 => On average all AC adjustment is done by AB)
			config_server::Parameter<float> legFootTiltIsRelRatio;  //!< @brief Ratio of whether foot tilts are considered to be relative or absolute (0 => Absolute, 1 => Relative)
			config_server::Parameter<float> legFOverNRatio;         //!< @brief Ratio of whether the adjusted F is S-above the reconciled N or F points (0 => Adjusted F is S-above the reconciled F point, 1 => Adjusted F is S-above the reconciled N point)
			config_server::Parameter<float> legHeightRatioE;        //!< @brief Ratio of the S-height of E above D (0 => E is at same S-height as D, 1 => E is at same S-height as F)
			config_server::Parameter<float> legHeightRatioG;        //!< @brief Ratio of the S-height of G above A (0 => G is at same S-height as A, 1 => G is at same S-height as F)
			config_server::Parameter<float> legSwingOutMaxIwd;      //!< @brief Maximum allowed inwards swing out in units of tilt phase
			config_server::Parameter<float> legSwingOutBuf;         //!< @brief Soft coercion buffer for the limiting of swing out in units of tilt phase
			config_server::Parameter<float> legHipHeightMin;        //!< @brief Minimum hip height to use (in units of tip leg scale)
			config_server::Parameter<float> legHipHeightNom;        //!< @brief Nominal hip height to use (in units of tip leg scale)
			config_server::Parameter<float> legSuppCoeffPhaseExtra; //!< @brief Total amount of extra phase in addition to the double support phases AB and CD, applied symmetrically to define the support coefficient transition period (i.e. the extra phase is applied half before and half after double support)
			config_server::Parameter<float> legSuppCoeffRange;      //!< @brief Required difference between the symmetric support coefficients during walking (e.g. if this is 0.8 the support coefficients transition between 0.1 and 0.9)

			// Arm trajectory parameters
			config_server::Parameter<float> armArmTiltAngleMaxAbs;  //!< @brief Maximum allowed tilt angle component of the arm tilt by absolute value
			config_server::Parameter<float> armArmTiltAngleBuf;     //!< @brief Soft coercion buffer for the tilt angle component of the arm tilt
			config_server::Parameter<float> armCoMRayTiltMaxP;      //!< @brief Maximum allowed tilt of the CoM ray (relative to straight down) by absolute value if the tilt is purely in the pitch direction (defines the major axis of the tilt limiting ellipse)
			config_server::Parameter<float> armCoMRayTiltMaxR;      //!< @brief Maximum allowed tilt of the CoM ray (relative to straight down) by absolute value if the tilt is purely in the roll direction (defines the minor axis of the tilt limiting ellipse)
			config_server::Parameter<float> armCoMRayTiltBuf;       //!< @brief Soft coercion buffer for the limiting of the tilt of the CoM ray
			config_server::Parameter<float> armCoMRayYMaxIwd;       //!< @brief Maximum allowed inwards normalised y component of the CoM ray
			config_server::Parameter<float> armCoMRayYBuf;          //!< @brief Soft coercion buffer for the limiting of the normalised y component of the CoM ray

			// Tuning parameters
			config_server::Parameter<bool>  tuningForceEvalHalt;    //!< @brief Force evaluation of the true halt pose in each cycle
			config_server::Parameter<bool>  tuningNoLegs;           //!< @brief Disable all motions of the legs
			config_server::Parameter<bool>  tuningNoLegTrajectory;  //!< @brief Disable the leg trajectory
			config_server::Parameter<bool>  tuningNoLegEfforts;     //!< @brief Disable leg efforts
			config_server::Parameter<bool>  tuningNoLegSuppCoeff;   //!< @brief Disable leg support coefficients
			config_server::Parameter<bool>  tuningNoArms;           //!< @brief Disable all motions of the arms
			config_server::Parameter<bool>  tuningNoArmTrajectory;  //!< @brief Disable the arm trajectory
			config_server::Parameter<bool>  tuningNoArmEfforts;     //!< @brief Disable arm efforts

			// Debug parameters
			config_server::Parameter<bool>  debugPrintGenLeg;       //!< @brief Boolean flag whether to print debug information about the leg component of the trajectory generate function
			config_server::Parameter<bool>  debugPrintEvalArm;      //!< @brief Boolean flag whether to print debug information about the arm component of the trajectory evaluate function
			config_server::Parameter<int>   debugPrintPrecision;    //!< @brief Fixed precision with which to print debug information

			// Derived configuration parameters
			Vec3 gcvBias;                                           //!< @brief Bias added to the gait command vector
			double D;                                               //!< @brief Sanity-checked phase length of the assumed double support phase, in the range [MinPhaseSep,pi/2] (see `phaseDoubleSupportLen`)

		private:
			// Configuration parameter callbacks
			void gcvBiasCB() { gcvBias << gcvBiasLinVelX(), gcvBiasLinVelY(), gcvBiasAngVelZ(); }
			void phaseDoubleSupportLenCB() { D = rc_utils::coerce<double>(phaseDoubleSupportLen(), MinPhaseSep, M_PI_2); }
		};

		//! Spline coefficients struct (x(t) = a0 + a1*t + a2*t^2 + a3*t^3)
		struct SplineCoeff
		{
			// Reset function
			void reset();

			// Set functions
			void set(double T, double x0, double xT, double v0, double vT);
			void setMV(double T, double x0, double vbar, double v0, double vT);

			// Evaluation functions
			double evalX(double t) const { return a0 + t*(a1 + t*(a2 + t*a3)); }
			double evalV(double t) const { return a1 + t*(2.0*a2 + t*3.0*a3); }
			double evalA(double t) const { return 2.0*a2 + t*6.0*a3; }
			double evalJ(double t) const { return 6.0*a3; }

			// Data members
			double T;  // Nominal range of t: [0,T]
			double a0; // Coefficient of 1
			double a1; // Coefficient of t
			double a2; // Coefficient of t^2
			double a3; // Coefficient of t^3
		};

		// Stream insertion operator for the spline coefficients struct
		inline std::ostream& operator<<(std::ostream& os, const SplineCoeff& coeff) { return os << "S(" << coeff.T << ", " << coeff.a0 << ", " << coeff.a1 << ", " << coeff.a2 << ", " << coeff.a3 << ")"; }

		//! Support coefficient variables struct
		struct SuppCoeffVars
		{
			// Reset function
			void reset();

			// Data members
			double centrePtRise;
			double centrePtFall;
			double transitionSlope;
			double transitionMargin;
		};

		/**
		* @class FeedKeypointTraj
		*
		* @brief Feedback gait keypoint trajectory generation class.
		**/
		template<class Kinematics> class FeedKeypointTraj : public FeedTrajectoryBase
		{
		private:
			// Static assertions
			static_assert(std::is_base_of<humanoid_kinematics::RobotKinematics, Kinematics>::value, "The Kinematics template parameter must derive from humanoid_kinematics::RobotKinematics");

		public:
			// Typedefs
			typedef typename Kinematics::JointPose JointPose;
			typedef typename Kinematics::JointArmPose JointArmPose;
			typedef typename Kinematics::JointEffort JointEffort;
			typedef typename Kinematics::JointLegEffort JointLegEffort;
			typedef typename Kinematics::JointArmEffort JointArmEffort;
			typedef typename Kinematics::AbsPose AbsPose;
			typedef typename Kinematics::AbsLegPose AbsLegPose;
			typedef typename Kinematics::AbsArmPose AbsArmPose;
			typedef typename Kinematics::AbsLegPoseVel AbsLegPoseVel;
			typedef typename Kinematics::InvPose InvPose;
			typedef typename Kinematics::InvLegPose InvLegPose;
			typedef typename Kinematics::LegTipPose LegTipPose;
			typedef typename Kinematics::LegTipPoseVel LegTipPoseVel;
			typedef FeedKinematicsKT<Kinematics> KinematicsInterface;
			typedef ConvenienceArrays<Kinematics> ConvArrays;

			// Constructor/destructor
			explicit FeedKeypointTraj(FeedPlotManager* PM) : FeedTrajectoryBase(PM), ktconfig(KeypointTrajConfig::getInstance()), KI(ktconfig), K(KI.K), RK(K), m_cmd(ktconfig.fusedPitchN()) { init(); }
			virtual ~FeedKeypointTraj() = default;

			// Configuration parameters
			const KeypointTrajConfig& ktconfig;

			// Kinematics interface
			const KinematicsInterface KI;
			const Kinematics& K;
			const hk::RobotKinematics& RK;

			// Information function
			virtual void getInfo(TrajInfo& trajInfo) const override;

			// Gait halt pose function
			virtual void getHaltPose(PoseCommand& haltPose) const override;

			// Trajectory generation function
			virtual void generate(const TrajCommand& trajCmd) override;

			// Evaluate function
			virtual void evaluate(double gaitPhase, PoseCommand& poseCmd) const override;

		protected:
			// Constants
			static const int NUM_ABS_LEG = ConvArrays::NUM_ABS_LEG;

			// Typedefs
			typedef InvLegPosesT<Kinematics> InvLegPoses;
			typedef double LegPhases[NUM_LR];
			typedef double ArmPhases[NUM_LR];
			typedef rot_conv::AbsTiltRot FootTilts[NUM_LR][NUM_KEYS];
			typedef rot_conv::AngVel FootAngVels[NUM_LR][NUM_KEYS];
			typedef AbsLegPose AbsLegPoses[NUM_LR][NUM_KEYS];
			typedef AbsArmPose AbsArmPoses[NUM_LR];
			typedef double AbsLegSplineData[NUM_LR][NUM_KEYS][NUM_ABS_LEG];
			typedef SplineCoeff AbsLegSplineCoeffs[NUM_LR][NUM_KEYS][NUM_ABS_LEG];

			// Evaluate gait halt pose function
			void evaluateHaltPose();
			bool m_evaluatingHaltPose;

			// Initialisation function
			void init();

			// Update functions
			void updateTrajCommand(const TrajCommand& trajCmd);
			void updateCommonVars();
			void updateBasePose();

			// Leg trajectory generation functions
			void generateStepSizes(const AbsPose& baseAP, LimbPhases& limbPhase, LegTipPoints& LTP, MotionCentre& MC, FootYaws& footYaw, NominalFootTilts& nomFootTilt, double& stepHeightDist) const;
			void projectKeypoints(const LegTipPoints& LTP, const MotionCentre& MC, LegTipPoints& relLTP) const;
			void rotateKeypoints(LegTipPoints& relLTP) const;
			void reconcileKeypoints(LegTipPoints& relLTP, FootYaws& footYaw) const;
			void adjustKeypoints(LegTipPoints& relLTP, const MotionCentre& MC, const NominalFootTilts& nomFootTilt, const double& stepHeightDist, LegTipPoints& LTP, FootYaws& footYaw, FootTilts& footTilt) const;
			void calcSuppLinVel(const LimbPhases& limbPhase, const LegTipPoints& LTP, LegTipPointVels& LTPVel) const;
			void calcSwingLinVel(const LimbPhases& limbPhase, LegTipPoints& LTP, LegTipPointVels& LTPVel) const;
			void calcAngVel(const LimbPhases& limbPhase, FootYaws& footYaw, FootTilts& footTilt, FootAngVels& footAngVel) const;
			void consolidateKeypoints(const LegTipPoints& LTP, const FootYaws& footYaw, const FootTilts& footTilt, InvLegPoses& ILP) const;
			void leanKeypoints(InvLegPoses& ILP, MotionCentre& MC, LegTipPointVels& LTPVel, FootAngVels& footAngVel) const;
			void shiftKeypoints(InvLegPoses& ILP, MotionCentre& MC) const;
			void finaliseKeypoints(InvLegPoses& ILP, AbsLegPoses& ALP, bool& exactKin, MotionCentre& MC) const;
			void calcLegTrajectory(const LimbPhases& limbPhase, const AbsLegPoses& ALP, const LegTipPointVels& LTPVel, const FootAngVels& footAngVel, AbsLegSplineCoeffs& absCoeff) const;
			void calcSuppCoeffVars(const LimbPhases& limbPhase, SuppCoeffVars& suppCoeffVars) const;

			// Leg evaluation functions
			void evalLegTrajectory(const LegPhases& legPhase, const LimbPhases& limbPhase, const AbsLegSplineCoeffs& absCoeff, AbsLegPose& legL, AbsLegPose& legR) const;
			void evalLegEfforts(const LegPhases& legPhase, const JointEffort& baseJE, JointLegEffort& legL, JointLegEffort& legR) const;
			void evalLegSuppCoeff(const LegPhases& legPhase, const PoseCommand::SuppCoeff& baseSC, const SuppCoeffVars& suppCoeffVars, PoseCommand::SuppCoeff& suppCoeff) const;

			// Arm evaluation functions
			void evalArmTrajectory(const ArmPhases& armPhase, const AbsPose& baseAP, const LimbPhases& limbPhase, AbsArmPose& armL, AbsArmPose& armR) const;
			void evalArmEfforts(const ArmPhases& armPhase, const JointEffort& baseJE, JointArmEffort& armL, JointArmEffort& armR) const;

			// Arm trajectory functions
			void genArmBaseMotion(double phase, const LimbPhases& limbPhase, AbsArmPose& AAP, JointArmPose& JAP, Vec3& CoM) const;
			void tiltArms(JointArmPose& JAP, Vec3& CoM) const;
			void limitArms(JointArmPose& JAP) const;

			// Convenience arrays
			ConvArrays CA;

			// Trajectory command
			TrajCommand m_cmd;

			// Base pose variables (updated in every cycle)
			AbsPose m_basePoseAP;
			JointEffort m_basePoseJE;
			PoseCommand::SuppCoeff m_basePoseSC;

			// Halt pose variables (calculated once at construction)
			PoseCommand m_haltPose;
			AbsPose m_haltPoseAP;
			JointEffort m_haltPoseJE;

			// Common variables
			CommonVars CV;

			// Generated trajectory
			LimbPhases m_limbPhase;
			AbsLegSplineCoeffs m_absCoeff;
			SuppCoeffVars m_suppCoeffVars;

			// Debugging variables
			mutable unsigned int m_genCount;
			mutable unsigned int m_evalCount;
		};
	}
}

// Include implementations that should occur in the header
#include <feed_gait/trajectory/keypoint/feed_keypoint_traj_impl.h>

#endif
// EOF