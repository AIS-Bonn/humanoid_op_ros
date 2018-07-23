// Feedback gait simple odometry
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_SIMPLE_ODOM_IMPL_H
#define FEED_SIMPLE_ODOM_IMPL_H

// Includes
#include <feed_gait/odometry/simple/feed_simple_odom.h>

// Feedback gait namespace
namespace feed_gait
{
	// Simple odometry namespace
	namespace simple_odom
	{
		//
		// FeedSimpleOdom class
		//

		// Update joint pose function
		template<class Kinematics> void FeedSimpleOdom<Kinematics>::updateJointPose(const std::vector<double>& jointPos)
		{
			// Update the joint pose
			m_JP.fromVector(jointPos);
		}

		// Update poses function
		template<class Kinematics> void FeedSimpleOdom<Kinematics>::updatePoses(const std::vector<double>& jointPos)
		{
			// Update the joint pose
			updateJointPose(jointPos);

			// Update the pose variables
			updatePoses();
		}

		// Update poses function
		template<class Kinematics> void FeedSimpleOdom<Kinematics>::updatePoses()
		{
			// Update the leg tip poses
			K.TipFromJoint(m_JP.legL, m_LTP[hk::LEFT]);
			K.TipFromJoint(m_JP.legR, m_LTP[hk::RIGHT]);

			// Update the hip to foot vectors
			m_hipFootVec[hk::LEFT] = m_LTP[hk::LEFT].pos + K.originLeg(hk::LEFT);
			m_hipFootVec[hk::RIGHT] = m_LTP[hk::RIGHT].pos + K.originLeg(hk::RIGHT);
		}

		// Update odometry state function (recalculates the remaining odometry state based on the trunk state)
		template<class Kinematics> void FeedSimpleOdom<Kinematics>::updateOdometryState()
		{
			// Update the hip centre point and hip points
			m_hipCentre = m_trunk.pos - m_trunk.rot * K.rconfig.trunkLinkOffset;
			Vec3 hipOffsetLeft = (hk::LS_LEFT * K.rconfig.H) * rot_conv::AxisYFromQuat(m_trunk.rot);
			m_hip[hk::LEFT] = m_hipCentre + hipOffsetLeft;
			m_hip[hk::RIGHT] = m_hipCentre - hipOffsetLeft;

			// Update the foot states
			m_footState[hk::LEFT].set(m_hip[hk::LEFT] + m_trunk.rot * m_hipFootVec[hk::LEFT], m_trunk.rot * m_LTP[hk::LEFT].rot);
			m_footState[hk::RIGHT].set(m_hip[hk::RIGHT] + m_trunk.rot * m_hipFootVec[hk::RIGHT], m_trunk.rot * m_LTP[hk::RIGHT].rot);
		}

		// Update support state function
		template<class Kinematics> void FeedSimpleOdom<Kinematics>::updateSupportState()
		{
			// Reset the support leg sign and adjust the robot vertically to have the corresponding foot on the floor
			if(m_footState[hk::LEFT].pos.z() <= m_footState[hk::RIGHT].pos.z())
				setSupportLeg(hk::LEFT);
			else
				setSupportLeg(hk::RIGHT);
			m_supportExchangeLock = true;
		}

		// Update outputs function
		template<class Kinematics> void FeedSimpleOdom<Kinematics>::updateOutputs()
		{
			// Update the odometry output struct
			m_out.pos2D = m_trunk.pos.head<2>();
			m_out.pos3D = m_trunk.pos;
			m_out.rot2D = m_trunkYaw;
			m_out.rot3D = m_trunk.rot;
			m_out.supportLeg = m_supportLeg;
		}

		// Set support leg function
		template<class Kinematics> void FeedSimpleOdom<Kinematics>::setSupportLeg(hk::LimbIndex limbIndex)
		{
			// Set the support leg
			m_supportLeg.set(limbIndex);

			// Adjust the vertical position of the robot to ensure that the support leg is on the floor
			double adjust = m_footState[limbIndex].pos.z();
			m_trunk.pos.z() -= adjust;
			m_hipCentre.z() -= adjust;
			m_hip[hk::LEFT].z() -= adjust;
			m_hip[hk::RIGHT].z() -= adjust;
			m_footState[hk::LEFT].pos.z() -= adjust;
			m_footState[hk::RIGHT].pos.z() -= adjust;
		}

		// Reset members function
		template<class Kinematics> void FeedSimpleOdom<Kinematics>::resetMembers(const OdometryInput& odomInput)
		{
			// Reset the robot pose variables
			updatePoses(odomInput.jointPos);

			// Reset the trunk state
			m_trunk.pos.setZero();
			m_trunk.rot = rot_conv::QuatFromFused(odomInput.robotOrient.fusedPitch, odomInput.robotOrient.fusedRoll);
			m_trunkYaw = 0.0;

			// Reset the remaining odometry state
			updateOdometryState();

			// Reset the support state
			updateSupportState();

			// Update the odometry outputs
			updateOutputs();
		}

		// Set function for the 2D pose
		template<class Kinematics> void FeedSimpleOdom<Kinematics>::setPose2D(double posX, double posY, double rotZ)
		{
			// Update the robot pose variables
			updatePoses();

			// Update the trunk state
			m_trunk.pos.x() = posX;
			m_trunk.pos.y() = posY;
			m_trunk.rot = rot_conv::QuatWithFYaw(m_trunk.rot, rotZ);
			m_trunkYaw = rotZ;

			// Update the remaining odometry state
			updateOdometryState();

			// Update the support state
			updateSupportState();

			// Update the odometry outputs
			updateOutputs();
		}

		// Update function
		template<class Kinematics> void FeedSimpleOdom<Kinematics>::update(const OdometryInput& odomInput)
		{
			// Update the robot pose variables
			updatePoses(odomInput.jointPos);

			// Get the current support/free leg indices and signs
			hk::LimbIndex S = m_supportLeg.index;
			hk::LimbIndex F = m_supportLeg.otherIndex();
			hk::LimbSign signF = m_supportLeg.otherSign();

			// Aliases for the support/free hip positions
			Vec3& hipS = m_hip[S];
			Vec3& hipF = m_hip[F];

			// Aliases for the support/free foot states
			FootState& footStateS = m_footState[S];
			FootState& footStateF = m_footState[F];

			// Get the tilt component of the rotation of the robot
			Quat qGBtilt = rot_conv::QuatFromFused(odomInput.robotOrient.fusedPitch, odomInput.robotOrient.fusedRoll);

			// Calculate the yaw rotation required to be applied to the entire robot so that the support foot yaw is as required
			double deltaYaw = footStateS.yaw - rot_conv::FYawOfQuat(qGBtilt * m_LTP[S].rot);

			// Calculate the rotation of the robot
			rot_conv::QuatRotGlobalZ(qGBtilt, deltaYaw, m_trunk.rot);
			m_trunkYaw = rot_conv::FYawOfQuat(m_trunk.rot);

			// Calculate the foot rotations
			footStateS.rot = m_trunk.rot * m_LTP[S].rot; // Note: By design this should be consistent with the footStateS yaw and yawRot members
			footStateF.setRot(m_trunk.rot * m_LTP[F].rot);

			// Calculate the remaining positions of the hips and free foot
			Vec3 HvecStoF = (signF * K.rconfig.H) * rot_conv::AxisYFromQuat(m_trunk.rot);
			hipS = footStateS.pos - m_trunk.rot * m_hipFootVec[S];
			m_hipCentre = hipS + HvecStoF;
			m_trunk.pos = m_hipCentre + m_trunk.rot * K.rconfig.trunkLinkOffset;
			hipF = m_hipCentre + HvecStoF;
			footStateF.pos = hipF + m_trunk.rot * m_hipFootVec[F];

			// Calculate the height of the free foot above the support foot
			double freeFootHeight = footStateF.pos.z() - footStateS.pos.z(); // Positive if the free foot is higher than the support foot

			// Allow a support exchange if the vertical foot separation has exceeded a configured threshold since the last support exchange
			if(m_supportExchangeLock && fabs(freeFootHeight) >= soconfig.footHeightHysteresis())
				m_supportExchangeLock = false;

			// Perform a support exchange if needed
			if(freeFootHeight < 0.0 && !m_supportExchangeLock)
			{
				setSupportLeg(F);
				m_supportExchangeLock = true;
			}

			// Update the odometry outputs
			updateOutputs();
		}
	}
}

#endif
// EOF