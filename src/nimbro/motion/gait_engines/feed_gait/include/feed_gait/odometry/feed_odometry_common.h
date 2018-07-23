// Feedback gait odometry common
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_ODOMETRY_COMMON_H
#define FEED_ODOMETRY_COMMON_H

// Includes
#include <feed_gait/feed_common.h>

// Feedback gait namespace
namespace feed_gait
{
	/**
	* @struct OdometryInput
	*
	* @brief Odometry input struct.
	**/
	struct OdometryInput : public CommonInput
	{
		// Constructor and reset
		OdometryInput() : CommonInput() { resetMembers(); }
		virtual void reset() override { CommonInput::reset(); resetMembers(); }

	private:
		// Reset members function
		void resetMembers() {}
	};

	/**
	* @struct OdometryOutput
	*
	* @brief Odometry output struct.
	**/
	struct OdometryOutput
	{
		// Constructor and reset
		OdometryOutput() { reset(); }
		void reset()
		{
			pos2D.setZero();
			pos3D.setZero();
			rot2D = 0.0;
			rot3D.setIdentity();
			supportLeg.set(hk::LEFT);
		}

		// Data members
		Vec2 pos2D;
		Vec3 pos3D;
		double rot2D; // Note: This is to be interpreted as a fused yaw
		Quat rot3D;
		hk::LRLimb supportLeg;
	};
}

#endif
// EOF