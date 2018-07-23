// Feedback gait simple odometry
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_SIMPLE_ODOM_H
#define FEED_SIMPLE_ODOM_H

// Includes
#include <feed_gait/feed_common.h>
#include <feed_gait/odometry/feed_odometry_base.h>
#include <humanoid_kinematics/robot_kinematics.h>
#include <config_server/parameter.h>
#include <type_traits>

// Feedback gait namespace
namespace feed_gait
{
	/**
	* @namespace simple_odom
	* 
	* @brief Simple odometry namespace.
	**/
	namespace simple_odom
	{
		/**
		* @class SimpleOdomConfig
		* 
		* @brief Simple odometry configuration parameters class.
		**/
		class SimpleOdomConfig
		{
		private:
			// Constructor
			SimpleOdomConfig()
				: TYPE_NAME(odometryTypeName(OT_SIMPLE))
				, CONFIG_PARAM_PATH(ODOM_CONFIG_PARAM_PATH + TYPE_NAME + "/")
				, footHeightHysteresis(CONFIG_PARAM_PATH + "footHeightHysteresis", 0.0, 0.001, 0.1, 0.005)
			{}

			// Ensure class remains a singleton
			SimpleOdomConfig(const SimpleOdomConfig&) = delete;
			SimpleOdomConfig& operator=(const SimpleOdomConfig&) = delete;

		public:
			// Get singleton instance of class
			static const SimpleOdomConfig& getInstance() { static thread_local SimpleOdomConfig soconfig; return soconfig; }

			// Constants
			const std::string TYPE_NAME;
			const std::string CONFIG_PARAM_PATH;

			// Configuration parameters
			config_server::Parameter<float> footHeightHysteresis; //!< @brief The minimum required foot height difference to unlock the possibility of a future support exchange
		};

		//! Foot state struct
		struct FootState
		{
			// Constructor
			FootState() = default;

			// Set functions
			void setPos(const Vec3& pos)
			{
				this->pos = pos;
			}
			void setRot(const Quat& rot)
			{
				this->rot = rot;
				yaw = rot_conv::FYawOfQuat(this->rot);
				yawRot = rot_conv::QuatFromAxis(rot_conv::Z_AXIS, yaw);
			}
			void set(const Vec3& pos, const Quat& rot)
			{
				setPos(pos);
				setRot(rot);
			}

			// Data members
			Vec3 pos;
			Quat rot;
			Quat yawRot;
			double yaw;
		};

		/**
		* @class FeedSimpleOdom
		*
		* @brief Feedback gait simple odometry class.
		**/
		template<class Kinematics> class FeedSimpleOdom : public FeedOdometryBase
		{
		private:
			// Static assertions
			static_assert(std::is_base_of<humanoid_kinematics::RobotKinematics, Kinematics>::value, "The Kinematics template parameter must derive from humanoid_kinematics::RobotKinematics");

		public:
			// Typedefs
			typedef typename Kinematics::JointPose JointPose;
			typedef typename Kinematics::LegTipPose LegTipPose;

			// Constructor/destructor
			explicit FeedSimpleOdom(FeedPlotManager* PM) : FeedOdometryBase(PM), soconfig(SimpleOdomConfig::getInstance()), m_LTP({hk::INDEX0, hk::INDEX1}) { OdometryInput odomInput; resetMembers(odomInput); }
			virtual ~FeedSimpleOdom() = default;

			// Configuration parameters
			const SimpleOdomConfig& soconfig;

			// Kinematics
			const Kinematics K;

			// Reset function
			virtual void reset(const OdometryInput& odomInput) override { FeedOdometryBase::reset(odomInput); resetMembers(odomInput); }

			// Set function for the 2D pose
			virtual void setPose2D(double posX, double posY, double rotZ) override;

			// Update function
			virtual void update(const OdometryInput& odomInput) override;

		protected:
			// Update functions
			void updateJointPose(const std::vector<double>& jointPos);
			void updatePoses(const std::vector<double>& jointPos);
			void updatePoses();
			void updateOdometryState();
			void updateSupportState();
			void updateOutputs();

			// Set functions
			void setSupportLeg(hk::LimbIndex limbIndex);

			// Robot pose variables
			JointPose m_JP;
			LegTipPose m_LTP[hk::NUM_LR];
			Vec3 m_hipFootVec[hk::NUM_LR];

			// Robot odometry state
			Frame m_trunk;
			double m_trunkYaw;
			Vec3 m_hipCentre;
			Vec3 m_hip[hk::NUM_LR];
			FootState m_footState[hk::NUM_LR];

			// Support condition variables
			hk::LRLimb m_supportLeg;
			bool m_supportExchangeLock;

		private:
			// Reset members function
			void resetMembers(const OdometryInput& odomInput);
		};
	}
}

// Include implementations that should occur in the header
#include <feed_gait/odometry/simple/feed_simple_odom_impl.h>

#endif
// EOF