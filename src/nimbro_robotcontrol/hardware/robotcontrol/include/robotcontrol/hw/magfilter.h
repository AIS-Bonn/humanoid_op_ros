// Magnetometer filter that performs a hard-iron correction
// File: magfilter.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef MAGFILTER_H
#define MAGFILTER_H

// Includes
#include <string>
#include <vector>
#include <ros/time.h>
#include <Eigen/Core>
#include <std_srvs/Empty.h>
#include <robotcontrol/MagCalib.h>
#include <robotcontrol/WarpAddPoint.h>
#include <config_server/parameter.h>
#include <visualization_msgs/Marker.h>

// State estimation namespace
namespace stateestimation
{
	/**
	* @class CyclicWarp
	* 
	* @brief Applies a monotonically increasing piecewise linear transform to a cyclic value.
	**/
	class CyclicWarp
	{
	public:
		// Constructor
		explicit CyclicWarp(double modulus = RADIANS) { reset(modulus); }
		
		// Constants
		static const double UNIT;
		static const double RADIANS;
		static const double DEGREES;
		
		// Reset functions
		void reset(double modulus = RADIANS);
		void clear();
		
		// Get/set functions
		double getModulus() const { return m_M; }
		void setModulus(double modulus) { m_M = fabs(modulus); }
		void getRefValues(std::vector<double>& raw, std::vector<double>& warped);
		bool setRefValues(const std::vector<double>& raw, const std::vector<double>& warped);
		
		// Transformation functions
		double wrap(double value) const { return value - m_M * floor(value / m_M); } // Wraps a value to [0,M)
		double warp(double rawValue) const { return transform(m_raw, m_warped, rawValue); }
		double unwarp(double warpedValue) const { return transform(m_warped, m_raw, warpedValue); }
		
	private:
		// Helper functions
		double transform(const std::vector<double>& in, const std::vector<double>& out, double inValue) const;
		
		// Internal members
		double m_M;                   // Modulus of the cyclic range [0,M)
		size_t m_N;                   // Number of reference values
		std::vector<double> m_raw;    // Raw reference values (must be cyclically monotonically increasing in [0,M))
		std::vector<double> m_warped; // Corresponding desired warped reference values (must be cyclically monotonically increasing in [0,M))
	};
	
	/**
	* @class MagFilter
	*
	* @brief Processes magnetometer readings and accounts for hard-iron effects.
	*
	* The filter/correction parameters can be adjusted by performing a calibration.
	* This is done via the start and stop calibration service calls.
	**/
	class MagFilter
	{
	public:
		// Constructor/destructor
		explicit MagFilter(const std::string& paramPath = "magFilter"); //!< @brief Default constructor (nominally the @p paramPath shouldn't end in a `/`).
		virtual ~MagFilter(); //!< @brief Default destructor.

		// Update functions
		void update(const Eigen::Vector3d& mag); //!< @brief Update the magnetometer filter with a new measurement (updates the corrected value that is returned by subsequent calls to `value()`).
		void update(double magX, double magY, double magZ); //!< @brief Update the magnetometer filter with a new measurement (updates the corrected value that is returned by subsequent calls to `value()`).

		// Get function for current hard iron calibration value (this value is subtracted from any measured magnetometer value to correct it)
		Eigen::Vector3d hardIron() const { return Eigen::Vector3d(m_hard_x(), m_hard_y(), m_hard_z()); } //!< @brief Returns the current hard iron calibration offset.
		double hardIronX() const { return m_hard_x(); } //!< @brief Returns the x-component of the current hard iron calibration offset.
		double hardIronY() const { return m_hard_y(); } //!< @brief Returns the y-component of the current hard iron calibration offset.
		double hardIronZ() const { return m_hard_z(); } //!< @brief Returns the z-component of the current hard iron calibration offset.

		// Get function for corrected magnetometer measurements
		Eigen::Vector3d value() const { return m_value; } //!< @brief Returns the corrected value of the last measurement passed to either of the update functions.
		double valueX() const { return m_value.x(); } //!< @brief Returns the x-component of the latest corrected magnetometer measurement.
		double valueY() const { return m_value.y(); } //!< @brief Returns the y-component of the latest corrected magnetometer measurement.
		double valueZ() const { return m_value.z(); } //!< @brief Returns the z-component of the latest corrected magnetometer measurement.
		Eigen::Vector3d valuePreWarp() const { return m_valuePreWarp; } //!< @brief Returns the last measurement passed to either of the update functions, as it was just prior to (if enabled) warping.

	private:
		// ROS service servers and service handlers
		bool startCalibration(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
		bool stopCalibration2D(robotcontrol::MagCalibRequest& req, robotcontrol::MagCalibResponse& resp);
		bool stopCalibration3D(robotcontrol::MagCalibRequest& req, robotcontrol::MagCalibResponse& resp);
		bool warpClearPoints(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
		bool warpAddPoint(robotcontrol::WarpAddPointRequest& req, robotcontrol::WarpAddPointResponse& resp);
		ros::ServiceServer m_srv_startCalibration;
		ros::ServiceServer m_srv_stopCalibration2D;
		ros::ServiceServer m_srv_stopCalibration3D;
		ros::ServiceServer m_srv_warpClearPoints;
		ros::ServiceServer m_srv_warpAddPoint;

		// Plotting of magnetometer calibration data
		config_server::Parameter<bool> m_plot_calib_data;
		visualization_msgs::Marker m_marker;
		ros::Publisher m_pub_marker;
		void updatePlotCalibData();

		// Hard iron offset parameters
		config_server::Parameter<bool> m_enable_hard;
		config_server::Parameter<float> m_hard_x;
		config_server::Parameter<float> m_hard_y;
		config_server::Parameter<float> m_hard_z;

		// Calibration measurements vector array
		std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_calibrationMeasurements;

		// Angle warping
		config_server::Parameter<bool> m_enable_warp;
		config_server::Parameter<std::string> m_warpParamString;
		config_server::Parameter<bool> m_warpUseDegrees;
		void handleWarpParamString();
		CyclicWarp m_warp;

		// Internal variables
		Eigen::Vector3d m_value;
		Eigen::Vector3d m_valuePreWarp;
		bool m_calibrating;
	};
}

#endif
// EOF