// Feedback gait common definitions
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_COMMON_H
#define FEED_COMMON_H

// Includes
#include <rot_conv/rot_conv.h>
#include <rot_conv/rot_conv_extras.h>
#include <rot_conv/rot_conv_frames.h>
#include <humanoid_kinematics/kinematics_common.h>
#include <string>
#include <vector>
#include <cmath>

// Feedback gait namespace
namespace feed_gait
{
	// Namespaces
	namespace hk = humanoid_kinematics;

	// Using declarations
	using rot_conv::Vec2;
	using rot_conv::Vec3;
	using rot_conv::Quat;
	using rot_conv::Rotmat;
	using rot_conv::Frame;

	// Constants
	const std::string RESOURCE_PATH = "feed_gait/";
	const std::string ROOT_CONFIG_PARAM_PATH = "/feed_gait/";
	const std::string KIN_CONFIG_PARAM_EXT = "kinematics/";
	const std::string KIN_CONFIG_PARAM_PATH = ROOT_CONFIG_PARAM_PATH + KIN_CONFIG_PARAM_EXT;
	const std::string TRAJ_CONFIG_PARAM_PATH = ROOT_CONFIG_PARAM_PATH + "trajectory/";
	const std::string ODOM_CONFIG_PARAM_PATH = ROOT_CONFIG_PARAM_PATH + "odometry/";
	const std::string MODEL_CONFIG_PARAM_PATH = ROOT_CONFIG_PARAM_PATH + "model/";

	// Kinematics type enumeration
	enum KinematicsType
	{
		KT_FIRST = 0,
		KT_SERIAL = KT_FIRST,  // Serial kinematics
		KT_PARALLEL,           // Parallel kinematics
		KT_COUNT,
		KT_DEFAULT = KT_SERIAL
	};
	const std::string KinematicsTypeName[KT_COUNT] = {
		"serial",
		"parallel"
	};
	inline bool kinematicsTypeValid(int type) { return (type >= KT_FIRST && type < KT_COUNT); }
	inline bool kinematicsTypeValid(KinematicsType type) { return kinematicsTypeValid((int) type); }
	inline KinematicsType kinematicsTypeFrom(int type) { return (kinematicsTypeValid(type) ? (KinematicsType) type : KT_DEFAULT); }
	inline const std::string& kinematicsTypeName(KinematicsType type) { return (kinematicsTypeValid(type) ? KinematicsTypeName[type] : KinematicsTypeName[KT_DEFAULT]); }

	// Trajectory type enumeration
	enum TrajectoryType
	{
		TT_FIRST = 0,
		TT_TRIVIAL = TT_FIRST,  // Trivial trajectory generation
		TT_KEYPOINT,            // Keypoint trajectory generation
		TT_COUNT,
		TT_DEFAULT = TT_TRIVIAL
	};
	const std::string TrajectoryTypeName[TT_COUNT] = {
		"trivial",
		"keypoint"
	};
	inline bool trajectoryTypeValid(int type) { return (type >= TT_FIRST && type < TT_COUNT); }
	inline bool trajectoryTypeValid(TrajectoryType type) { return trajectoryTypeValid((int) type); }
	inline TrajectoryType trajectoryTypeFrom(int type) { return (trajectoryTypeValid(type) ? (TrajectoryType) type : TT_DEFAULT); }
	inline const std::string& trajectoryTypeName(TrajectoryType type) { return (trajectoryTypeValid(type) ? TrajectoryTypeName[type] : TrajectoryTypeName[TT_DEFAULT]); }

	// Odometry type enumeration
	enum OdometryType
	{
		OT_FIRST = 0,
		OT_TRIVIAL = OT_FIRST,  // Trivial odometry
		OT_SIMPLE,              // Simple odometry
		OT_COUNT,
		OT_DEFAULT = OT_TRIVIAL
	};
	const std::string OdometryTypeName[OT_COUNT] = {
		"trivial",
		"simple"
	};
	inline bool odometryTypeValid(int type) { return (type >= OT_FIRST && type < OT_COUNT); }
	inline bool odometryTypeValid(OdometryType type) { return odometryTypeValid((int) type); }
	inline OdometryType odometryTypeFrom(int type) { return (odometryTypeValid(type) ? (OdometryType) type : OT_DEFAULT); }
	inline const std::string& odometryTypeName(OdometryType type) { return (odometryTypeValid(type) ? OdometryTypeName[type] : OdometryTypeName[OT_DEFAULT]); }

	// Model type enumeration
	enum ModelType
	{
		MT_FIRST = 0,
		MT_TRIVIAL = MT_FIRST,  // Trivial model
		MT_TILT_PHASE,          // Tilt phase model
		MT_COUNT,
		MT_DEFAULT = MT_TRIVIAL
	};
	const std::string ModelTypeName[MT_COUNT] = {
		"trivial",
		"tilt_phase"
	};
	inline bool modelTypeValid(int type) { return (type >= MT_FIRST && type < MT_COUNT); }
	inline bool modelTypeValid(ModelType type) { return modelTypeValid((int) type); }
	inline ModelType modelTypeFrom(int type) { return (modelTypeValid(type) ? (ModelType) type : MT_DEFAULT); }
	inline const std::string& modelTypeName(ModelType type) { return (modelTypeValid(type) ? ModelTypeName[type] : ModelTypeName[MT_DEFAULT]); }

	// Check if finite function
	inline bool isFinite(double value) { return std::isfinite(value); }
	inline bool isFinite(const Vec2& vec) { return std::isfinite(vec.x()) && std::isfinite(vec.y()); }
	inline bool isFinite(const Vec3& vec) { return std::isfinite(vec.x()) && std::isfinite(vec.y()) && std::isfinite(vec.z()); }
	inline bool isFinite(const rot_conv::AbsTiltRot& tilt) { return std::isfinite(tilt.absTiltAxisAngle) && std::isfinite(tilt.tiltAngle); }

	/**
	* @struct CommonInput
	*
	* @brief Common input struct.
	**/
	struct CommonInput
	{
		// Constructor and reset
		CommonInput() { resetMembers(); }
		virtual void reset() { resetMembers(); }

		// Data members
		double truedT;
		double nominaldT;
		double timestamp;
		std::vector<double> jointPos;
		rot_conv::FusedAngles robotOrient;

	private:
		// Reset members function
		void resetMembers()
		{
			truedT = nominaldT = timestamp = 0.0;
			jointPos.clear();
			robotOrient.setIdentity();
		}
	};

	/**
	* @struct PoseCommand
	*
	* @brief Pose command struct.
	**/
	struct PoseCommand
	{
		// Typedefs
		typedef std::vector<double> DblVec;
		typedef double SuppCoeff[hk::NUM_LR];

		// Constructor and reset
		PoseCommand() { reset(); }
		void reset()
		{
			pos.clear();
			effort.clear();
			suppCoeff[hk::LEFT] = suppCoeff[hk::RIGHT] = 0.5;
		}

		// Set functions
		void setPos(const DblVec& pos) { this->pos = pos; }
		void setEffort(const DblVec& effort) { this->effort = effort; }
		void setSuppCoeff(const SuppCoeff& suppCoeff)
		{
			this->suppCoeff[hk::LEFT] = suppCoeff[hk::LEFT];
			this->suppCoeff[hk::RIGHT] = suppCoeff[hk::RIGHT];
		}

		// Mean functions
		void setMeanOf(const PoseCommand& PCA, const PoseCommand& PCB) { meanOf(PCA, PCB, *this); }
		static void meanOf(const PoseCommand& PCA, const PoseCommand& PCB, PoseCommand& PC)
		{
			meanOf(PCA.pos, PCB.pos, PC.pos);
			meanOf(PCA.effort, PCB.effort, PC.effort);
			meanOf(PCA.suppCoeff, PCB.suppCoeff, PC.suppCoeff);
		}
		static void meanOf(const DblVec& vecA, const DblVec& vecB, DblVec& vec)
		{
			std::size_t NA = vecA.size(), NB = vecB.size(), N = std::max(NA, NB);
			vec.resize(N);
			for(std::size_t i = 0; i < N; i++)
			{
				if(i >= NA)
					vec[i] = vecB[i];
				else if(i >= NB)
					vec[i] = vecA[i];
				else
					vec[i] = 0.5*(vecA[i] + vecB[i]);
			}
		}
		static void meanOf(const SuppCoeff& suppCoeffA, const SuppCoeff& suppCoeffB, SuppCoeff& suppCoeff)
		{
			suppCoeff[hk::LEFT] = 0.5*(suppCoeffA[hk::LEFT] + suppCoeffB[hk::LEFT]);
			suppCoeff[hk::RIGHT] = 0.5*(suppCoeffA[hk::RIGHT] + suppCoeffB[hk::RIGHT]);
		}

		// Data members
		DblVec pos;
		DblVec effort;
		SuppCoeff suppCoeff;
	};

	// Stream insertion operator for the PoseCommand type
	std::ostream& operator<<(std::ostream& os, const PoseCommand& poseCmd);
}

#endif
// EOF