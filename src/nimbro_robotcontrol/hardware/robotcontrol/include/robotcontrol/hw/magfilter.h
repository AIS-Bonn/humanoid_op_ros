// Magnetometer filter that performs a hard-iron correction
// File: magfilter.h
// Authors: Max Schwarz <max.schwarz@uni-bonn.de>
//          Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef MAGFILTER_H
#define MAGFILTER_H

// Includes
#include <ros/time.h>
#include <Eigen/Core>
#include <std_srvs/Empty.h>
#include <robotcontrol/MagCalib.h>
#include <config_server/parameter.h>
#include <visualization_msgs/Marker.h>

// State estimation namespace
namespace stateestimation
{
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
		Eigen::Vector3d hardIron() const { return Eigen::Vector3d(m_hard_x(), m_hard_y(), m_hard_z()); }
		double hardIronX() const { return m_hard_x(); }
		double hardIronY() const { return m_hard_y(); }
		double hardIronZ() const { return m_hard_z(); }

		// Get function for corrected magnetometer measurements
		Eigen::Vector3d value() const { return m_value; } //!< @brief Returns the corrected value of the last measurement passed to either of the update functions.
		double valueX() const { return m_value.x(); } //!< @brief Returns the x-component of the latest corrected magnetometer measurement.
		double valueY() const { return m_value.y(); } //!< @brief Returns the y-component of the latest corrected magnetometer measurement.
		double valueZ() const { return m_value.z(); } //!< @brief Returns the z-component of the latest corrected magnetometer measurement.

	private:
		// ROS service servers and service handlers
		bool startCalibration(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
		bool stopCalibration2D(robotcontrol::MagCalibRequest& req, robotcontrol::MagCalibResponse& resp);
		bool stopCalibration3D(robotcontrol::MagCalibRequest& req, robotcontrol::MagCalibResponse& resp);
		ros::ServiceServer m_srv_startCalibration;
		ros::ServiceServer m_srv_stopCalibration2D;
		ros::ServiceServer m_srv_stopCalibration3D;

		// Plotting of magnetometer calibration data
		config_server::Parameter<bool> m_plot_calib_data;
		visualization_msgs::Marker m_marker;
		ros::Publisher m_pub_marker;
		void updatePlotCalibData();

		// Config server parameters
		config_server::Parameter<float> m_hard_x;
		config_server::Parameter<float> m_hard_y;
		config_server::Parameter<float> m_hard_z;

		// Calibration measurements vector array
		std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_calibrationMeasurements;

		// Internal variables
		Eigen::Vector3d m_value;
		bool m_calibrating;
	};
}

#endif
// EOF