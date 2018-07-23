// Feedback gait model common
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_MODEL_COMMON_H
#define FEED_MODEL_COMMON_H

// Includes
#include <feed_gait/feed_common.h>
#include <feed_gait/odometry/feed_odometry_common.h>
#include <humanoid_kinematics/kinematics_common.h>
#include <rc_utils/math_funcs.h>

// Feedback gait namespace
namespace feed_gait
{
	/**
	* @struct TimingCommand
	*
	* @brief Timing command struct.
	**/
	struct TimingCommand
	{
		// Constructor and reset
		TimingCommand() { reset(); }
		void reset() { gaitFrequency = 0.0; }

		// Validate function
		void validate(bool& useTiming) const
		{
			if(!isFinite(gaitFrequency)) useTiming = false;
		}

		// Data members
		double gaitFrequency;
	};

	/**
	* @struct StepSizeAxisCommand
	*
	* @brief Step size along a particular axis command struct.
	**/
	struct StepSizeAxisCommand
	{
		// Constructor and reset
		StepSizeAxisCommand() { reset(); }
		void reset()
		{
			gcvLF = gcvHF = gcvEOS = 0.0;
			use = false;
		}

		// Validate function
		void validate(bool& useStepSize) const
		{
			if(use && !(isFinite(gcvLF) && isFinite(gcvHF) && isFinite(gcvEOS))) useStepSize = false; // Non-finite values are only a problem if they're supposed to be used...
		}

		// Set function
		void set(double gcvLF, double gcvHF, double gcvEOS)
		{
			this->gcvLF = gcvLF;
			this->gcvHF = gcvHF;
			this->gcvEOS = gcvEOS;
			use = true;
		}

		// Write function
		void writeTo(StepSizeAxisCommand& cmd) const
		{
			if(use) cmd = *this;
		}

		// Get functions
		double targetGcv() const { return gcvLF + gcvHF + gcvEOS; }

		// Data members
		double gcvLF;
		double gcvHF;
		double gcvEOS;
		bool use;
	};

	/**
	* @struct StepSizeCommand
	*
	* @brief Step size command struct.
	**/
	struct StepSizeCommand
	{
		// Constructor and reset
		StepSizeCommand() { reset(); }
		void reset()
		{
			X.reset();
			Y.reset();
			Z.reset();
		}

		// Validate function
		void validate(bool& useStepSize) const
		{
			X.validate(useStepSize);
			Y.validate(useStepSize);
			Z.validate(useStepSize);
		}

		// Set function
		void set(const Vec3& gcvLF, const Vec3& gcvHF, const Vec3& gcvEOS)
		{
			X.set(gcvLF.x(), gcvHF.x(), gcvEOS.x());
			Y.set(gcvLF.y(), gcvHF.y(), gcvEOS.y());
			Z.set(gcvLF.z(), gcvHF.z(), gcvEOS.z());
		}

		// Write function
		void writeTo(StepSizeCommand& cmd) const
		{
			X.writeTo(cmd.X);
			Y.writeTo(cmd.Y);
			Z.writeTo(cmd.Z);
		}

		// Get functions
		Vec3 targetGcv() const { return Vec3(X.targetGcv(), Y.targetGcv(), Z.targetGcv()); }

		// Data members
		StepSizeAxisCommand X;
		StepSizeAxisCommand Y;
		StepSizeAxisCommand Z;
	};

	/**
	* @struct ACFlag
	* 
	* @brief Actions command flag enumeration wrapper struct.
	**/
	struct ACFlag
	{
		// Actions command flag enumeration
		enum
		{
			NONE         = 0x0000,
			FUSEDPITCHS  = 0x0001, // 1
			FUSEDROLLS   = 0x0002, // 2
			FOOTTILTCTS  = 0x0004, // 4
			FOOTTILTSUPP = 0x0008, // 8
			SWINGOUT     = 0x0010, // 16
			LEANTILT     = 0x0020, // 32
			HIPSHIFT     = 0x0040, // 64
			HIPHEIGHTMAX = 0x0080, // 128
			ARMTILT      = 0x0100, // 256
			ALL          = 0x01FF
		};

		// Constructors
		ACFlag() { value = NONE; }
		ACFlag(int flags) { value = (flags & ALL); }

		// Flag manipulation functions
		void reset() { value = NONE; }
		void setFlags(int flags) { value |= (flags & ALL); }
		void setFlags(const ACFlag& flags) { setFlags(flags.value); }
		void unsetFlags(int flags) { value &= ~(flags & ALL); }
		void unsetFlags(const ACFlag& flags) { unsetFlags(flags.value); }

		// Flag test functions
		bool hasFlag(int flag) const { return (value & flag) != 0; }

		// Operators
		ACFlag& operator=(int flags) { value = (flags & ALL); return *this; }
		bool operator==(const ACFlag& other) const { return value == other.value; }
		bool operator!=(const ACFlag& other) const { return value != other.value; }
		int& operator()() { return value; }

		// Mask function
		void applyMask() { value &= ALL; }

		// Data members
		int value;
	};

	/**
	* @struct ActionsCommand
	*
	* @brief Actions command struct.
	**/
	struct ActionsCommand
	{
		// Constructor and reset
		explicit ActionsCommand(double fusedPitchN) { resetMembers(fusedPitchN); }
		virtual void reset(double fusedPitchN) { resetMembers(fusedPitchN); }

		// Validate function
		void validate(ACFlag& flags) const
		{
			if(!isFinite(fusedPitchS))  flags.unsetFlags(ACFlag::FUSEDPITCHS);
			if(!isFinite(fusedRollS))   flags.unsetFlags(ACFlag::FUSEDROLLS);
			if(!isFinite(footTiltCts))  flags.unsetFlags(ACFlag::FOOTTILTCTS);
			if(!isFinite(footTiltSupp)) flags.unsetFlags(ACFlag::FOOTTILTSUPP);
			if(!isFinite(swingOut))     flags.unsetFlags(ACFlag::SWINGOUT);
			if(!isFinite(leanTilt))     flags.unsetFlags(ACFlag::LEANTILT);
			if(!isFinite(hipShift))     flags.unsetFlags(ACFlag::HIPSHIFT);
			if(!isFinite(hipHeightMax)) flags.unsetFlags(ACFlag::HIPHEIGHTMAX);
			if(!isFinite(armTilt))      flags.unsetFlags(ACFlag::ARMTILT);
		}

		// Set functions
		void setFrom(const ActionsCommand& actionsCmd, const ACFlag& flags) { setFrom(actionsCmd, flags.value); }
		void setFrom(const ActionsCommand& actionsCmd, int flags)
		{
			if(flags & ACFlag::FUSEDPITCHS)  fusedPitchS = actionsCmd.fusedPitchS;
			if(flags & ACFlag::FUSEDROLLS)   fusedRollS = actionsCmd.fusedRollS;
			if(flags & ACFlag::FOOTTILTCTS)  footTiltCts = actionsCmd.footTiltCts;
			if(flags & ACFlag::FOOTTILTSUPP) footTiltSupp = actionsCmd.footTiltSupp;
			if(flags & ACFlag::SWINGOUT)     swingOut = actionsCmd.swingOut;
			if(flags & ACFlag::LEANTILT)     leanTilt = actionsCmd.leanTilt;
			if(flags & ACFlag::HIPSHIFT)     hipShift = actionsCmd.hipShift;
			if(flags & ACFlag::HIPHEIGHTMAX) hipHeightMax = actionsCmd.hipHeightMax;
			if(flags & ACFlag::ARMTILT)      armTilt = actionsCmd.armTilt;
		}

		// Leg data members
		double fusedPitchS;                //!< @brief Fused pitch of the body-fixed axes B relative to the swing ground plane S
		double fusedRollS;                 //!< @brief Fused roll of the body-fixed axes B relative to the swing ground plane S
		rot_conv::AbsTiltRot footTiltCts;  //!< @brief Continuous foot tilt relative to the nominal ground plane N (absolute or relative)
		rot_conv::AbsTiltRot footTiltSupp; //!< @brief Support foot tilt relative to the nominal ground plane N (always absolute)
		rot_conv::AbsTiltRot swingOut;     //!< @brief Peak swing leg tilt relative to the nominal ground plane N
		rot_conv::AbsTiltRot leanTilt;     //!< @brief Upper body leaning tilt relative to the nominal ground plane N
		Vec2 hipShift;                     //!< @brief Normalised xy cartesian shift (in units of inverse leg scale) of the robot's hips relative to the leaned ground plane L
		double hipHeightMax;               //!< @brief Maximum normalised hip height to use (in units of tip leg scale) relative to the leaned ground plane L

		// Arm data members
		rot_conv::AbsTiltRot armTilt;      //!< @brief Additive tilt rotation of the arms relative to the leaned ground plane L

	private:
		// Reset members function
		void resetMembers(double fusedPitchN)
		{
			fusedPitchS = fusedPitchN;
			fusedRollS = 0.0;
			footTiltCts.setIdentity();
			footTiltSupp.setIdentity();
			swingOut.setIdentity();
			leanTilt.setIdentity();
			hipShift.setZero();
			hipHeightMax = INFINITY;
			armTilt.setIdentity();
		}
	};

	/**
	* @struct GaitPhaseInfo
	* 
	* @brief Gait phase information struct.
	**/
	struct GaitPhaseInfo
	{
		// Constants
		static constexpr double MinTimeToStep = 1e-8;

		// Constructor and reset
		GaitPhaseInfo() { reset(); }
		void reset() { setGaitPhase(0.0); }

		// Set functions
		void setGaitPhase(double gaitPhase)
		{
			this->gaitPhase = rc_utils::picut(gaitPhase);
			if(this->gaitPhase > 0.0)
			{
				suppLegIndex = hk::RIGHT;
				elapGaitPhase = this->gaitPhase;
				remGaitPhase = M_PI - this->gaitPhase;
			}
			else
			{
				suppLegIndex = hk::LEFT;
				elapGaitPhase = M_PI + this->gaitPhase;
				remGaitPhase = 0.0 - this->gaitPhase;
			}
		}

		// Calculate gait frequency from a time to step
		double gaitFreqFromTimeToStep(double timeToStep, hk::LimbIndex suppLegIndex) const
		{
			double actualRemGaitPhase = (suppLegIndex != this->suppLegIndex ? remGaitPhase + M_PI : remGaitPhase);
			return actualRemGaitPhase / (M_PI * rc_utils::coerceMin(timeToStep, MinTimeToStep));
		}

		// Data members
		double gaitPhase;           //!< @brief Current gait phase in the range (-pi,pi], where positive values correspond to the right support phase and negative values correspond to the left support phase
		hk::LimbIndex suppLegIndex; //!< @brief Current commanded support leg index, based solely on the gait phase (this toggles at the instant of touchdown, i.e. at the start of each double support phase)
		double elapGaitPhase;       //!< @brief Gait phase in the range (0,pi] since the last change of support leg index
		double remGaitPhase;        //!< @brief Gait phase in the range [0,pi) until the next change of support leg index
	};

	/**
	* @struct TrajInfo
	*
	* @brief Trajectory generation information struct.
	**/
	struct TrajInfo
	{
		// Constructor and reset
		TrajInfo() { reset(); }
		void reset()
		{
			dblSuppPhaseLen = 0.5;
			fusedPitchN = 0.0;
			hipHeightMin = 0.0;
			hipHeightNom = 1.0;
		}

		// Data members
		double dblSuppPhaseLen; //!< @brief Length of the double support phase (in units of radians)
		double fusedPitchN;     //!< @brief Fused pitch of the body-fixed axes B relative to the nominal ground plane N
		double hipHeightMin;    //!< @brief Minimum normalised hip height to use (in units of tip leg scale)
		double hipHeightNom;    //!< @brief Nominal normalised hip height to use (in units of tip leg scale)
	};

	/**
	* @struct ModelInput
	*
	* @brief Model input struct.
	**/
	struct ModelInput : public OdometryInput
	{
		// Constructor and reset
		ModelInput() : OdometryInput() {  resetMembers(); }
		virtual void reset() override { OdometryInput::reset(); resetMembers(); }

		// Data members
		Vec3 inputGcv;
		double nomGaitFrequency;
		TrajInfo trajInfo;

	private:
		// Reset members function
		void resetMembers()
		{
			inputGcv.setZero();
			nomGaitFrequency = 0.0;
			trajInfo.reset();
		}
	};

	/**
	* @struct ModelOutput
	*
	* @brief Model output struct.
	**/
	struct ModelOutput
	{
		// Constructor and reset
		ModelOutput() : actionsCmd(0.0) { reset(); }
		void reset()
		{
			timingCmd.reset();
			useTiming = false;
			stepSizeCmd.reset();
			useStepSize = false;
			actionsCmd.reset(0.0); // A default fusedPitchN of zero is okay, as by default no actions are used due to useActions anyway
			useActions = ACFlag::NONE;
		}

		// Validate function (returns true if the commands were all valid)
		bool validate()
		{
			bool oldUseTiming = useTiming;
			bool oldUseStepSize = useStepSize;
			ACFlag oldUseActions = useActions;
			timingCmd.validate(useTiming);
			stepSizeCmd.validate(useStepSize);
			actionsCmd.validate(useActions);
			return (useTiming == oldUseTiming && useStepSize == oldUseStepSize && useActions == oldUseActions);
		}

		// Data members
		TimingCommand timingCmd;
		bool useTiming;
		StepSizeCommand stepSizeCmd;
		bool useStepSize;
		ActionsCommand actionsCmd;
		ACFlag useActions;
	};
}

#endif
// EOF