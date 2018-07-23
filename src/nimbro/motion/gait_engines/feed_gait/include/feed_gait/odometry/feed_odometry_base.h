// Feedback gait odometry base class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_ODOMETRY_BASE_H
#define FEED_ODOMETRY_BASE_H

// Includes
#include <feed_gait/feed_common.h>
#include <feed_gait/feed_plot.h>
#include <feed_gait/odometry/feed_odometry_common.h>
#include <config_server/parameter.h>

// Feedback gait namespace
namespace feed_gait
{
	/**
	* @class CommonOdomConfig
	* 
	* @brief Common odometry configuration parameters class.
	**/
	class CommonOdomConfig
	{
	private:
		// Constructor
		CommonOdomConfig()
			: CONFIG_PARAM_PATH(ODOM_CONFIG_PARAM_PATH + "common/")
		{}

		// Ensure class remains a singleton
		CommonOdomConfig(const CommonOdomConfig&) = delete;
		CommonOdomConfig& operator=(const CommonOdomConfig&) = delete;

	public:
		// Get singleton instance of class
		static const CommonOdomConfig& getInstance() { static thread_local CommonOdomConfig coconfig; return coconfig; }

		// Constants
		const std::string CONFIG_PARAM_PATH;
	};

	/**
	* @class FeedOdometryBase
	*
	* @brief Feedback gait odometry base class.
	**/
	class FeedOdometryBase
	{
	public:
		// Constructor/destructor
		explicit FeedOdometryBase(FeedPlotManager* PM) : coconfig(CommonOdomConfig::getInstance()), m_PM(PM) { OdometryInput odomInput; resetMembers(odomInput); }
		virtual ~FeedOdometryBase() = default;

		// Configuration parameters
		const CommonOdomConfig& coconfig;

		// Reset functions
		void reset() { OdometryInput odomInput; reset(odomInput); }
		virtual void reset(const OdometryInput& odomInput) { resetMembers(odomInput); } // This function should call the base implementation, then reset all members of the class (including the odometry output to zero) such that calling update(odomInput) with the same odomInput straight afterwards would not change anything

		// Set functions for the 2D pose (Note: rotZ is to be interpreted as a fused yaw)
		void resetPose2D() { setPose2D(0.0, 0.0, 0.0); }
		void setPose2D(const Vec3& pose2D) { setPose2D(pose2D.x(), pose2D.y(), pose2D.z()); }
		void setPose2D(const Vec2& pos2D, double rotZ) { setPose2D(pos2D.x(), pos2D.y(), rotZ); }
		virtual void setPose2D(double posX, double posY, double rotZ) = 0; // This function should update the odometry output struct with a new 2D pose, while maintaining the remaining robot state

		// Update functions
		void callUpdate(const OdometryInput& odomInput);
		virtual void update(const OdometryInput& odomInput) = 0; // This function should update the odometry output struct based on the odometry inputs and current state

		// Output function
		OdometryOutput output() const { return m_out; }

		// Output functions for specific members
		Vec2 pos2D() const { return m_out.pos2D; }
		Vec3 pos3D() const { return m_out.pos3D; }
		double rot2D() const { return m_out.rot2D; }
		Quat rot3D() const { return m_out.rot3D; }
		hk::LRLimb supportLeg() const { return m_out.supportLeg; }
		bool supportLegIsLeft() const { return m_out.supportLeg.isLeft; }
		hk::LimbIndex supportLegIndex() const { return m_out.supportLeg.index; }
		hk::LimbSign supportLegSign() const { return m_out.supportLeg.sign; }

		// Dependent output functions
		Vec3 pose2D() const { return Vec3(m_out.pos2D.x(), m_out.pos2D.y(), m_out.rot2D); }

	protected:
		// Plot manager
		FeedPlotManager* const m_PM;

		// Odometry output
		OdometryOutput m_out;

	private:
		// Reset members function
		void resetMembers(const OdometryInput& odomInput) { m_inited = false; m_out.reset(); }

		// Initialised flag
		bool m_inited;
	};

	// Typedefs
	typedef std::shared_ptr<FeedOdometryBase> FeedOdometryBasePtr;
}

#endif
// EOF