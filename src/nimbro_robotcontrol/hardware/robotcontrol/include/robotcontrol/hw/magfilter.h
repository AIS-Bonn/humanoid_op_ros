// Magnetometer filter that performs data correction
// File: magfilter.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef MAGFILTER_H
#define MAGFILTER_H

// Includes
#include <boost/circular_buffer.hpp>
#include <boost/circular_buffer/base.hpp>
#include <config_server/parameter.h>
#include <vis_utils/marker_manager.h>
#include <rc_utils/limited_low_pass.h>
#include <robotcontrol/WarpAddPoint.h>
#include <robotcontrol/MagCalibShow.h>
#include <robotcontrol/MagCalib2D.h>
#include <robotcontrol/MagCalib3D.h>
#include <plot_msgs/plot_manager.h>
#include <rc_utils/math_funcs.h>
#include <rc_utils/cyclicwarp.h>
#include <rc_utils/conicfit.h>
#include <std_srvs/Empty.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <ros/ros.h>
#include <vector>
#include <string>

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
		MagFilter(const std::string& resourcePath, const std::string& configParamPath, double cycleTime); //!< @brief Default constructor (the @p resourcePath and @p configParamPath arguments should end in a `/`).
		virtual ~MagFilter() {} //!< @brief Default destructor.

		// Get/set cycle time
		double getCycleTime() const { return m_cycleTime; }
		void setCycleTime(double cycleTime);

		// Get/set assumed global orientation (used for auto-calib data selection only, fused yaw component is disregarded, quaternion should be pure fused yaw when the sensor is level and upright)
		Eigen::Quaterniond getOrientation() const { return m_quat; }
		void setOrientation(const Eigen::Quaterniond& quat) { m_quat = quat.normalized(); m_haveQuat = true; } // Important: The provided quaternion should be the identity (plus/minus fused yaw) when the sensor is level and upright!
		void clearOrientation() { m_quat.setIdentity(); m_haveQuat = false; }

		// Update functions
		void update(const Eigen::Vector3d& mag); //!< @brief Update the magnetometer filter with a new measurement (updates the corrected value that is returned by subsequent calls to `value()`).
		void update(double magX, double magY, double magZ) { update(Eigen::Vector3d(magX, magY, magZ)); } //!< @brief Update the magnetometer filter with a new measurement (updates the corrected value that is returned by subsequent calls to `value()`).

		// Get function for current hard iron calibration (this value is subtracted from any measured magnetometer value to correct it)
		Eigen::Vector3d hardIronOffset() const { return m_hardOffset; } //!< @brief Returns the current hard iron calibration offset.
		double hardIronOffsetX() const { return m_hardOffset.x(); } //!< @brief Returns the x-component of the current hard iron calibration offset.
		double hardIronOffsetY() const { return m_hardOffset.y(); } //!< @brief Returns the y-component of the current hard iron calibration offset.
		double hardIronOffsetZ() const { return m_hardOffset.z(); } //!< @brief Returns the z-component of the current hard iron calibration offset.
		double hardIronField() const { return m_hardField; } //!< @brief Returns the current estimated field strength based on the hard iron calibration.

		// Get function for current soft iron calibration
		Eigen::Vector3d softIronOffset() const { return m_softOffset; } //!< @brief Returns the current soft iron calibration offset.
		Eigen::Matrix3d softIronMatrix() const { return m_softMatrix; } //!< @brief Returns the current soft iron calibration matrix (transforms raw data to data on a unit sphere).
		double softIronField() const { return m_softField; } //!< @brief Returns the current estimated field strength based on the soft iron calibration.

		// Get function for corrected magnetometer measurements
		Eigen::Vector3d value() const { return m_value; } //!< @brief Returns the corrected value of the last measurement passed to either of the update functions.
		double valueX() const { return m_value.x(); } //!< @brief Returns the x-component of the latest corrected magnetometer measurement.
		double valueY() const { return m_value.y(); } //!< @brief Returns the y-component of the latest corrected magnetometer measurement.
		double valueZ() const { return m_value.z(); } //!< @brief Returns the z-component of the latest corrected magnetometer measurement.
		Eigen::Vector3d valuePreWarp() const { return m_valuePreWarp; } //!< @brief Returns the last measurement passed to either of the update functions, as it was just prior to (if enabled) warping.

	private:
		// Cycle time
		double m_cycleTime;

		// ROS service servers and service handlers
		bool startCalibration(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
		bool stopCalibration2D(robotcontrol::MagCalib2DRequest& req, robotcontrol::MagCalib2DResponse& resp);
		bool stopCalibration3D(robotcontrol::MagCalib3DRequest& req, robotcontrol::MagCalib3DResponse& resp);
		bool abortCalibration(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
		bool showCalibration(robotcontrol::MagCalibShowRequest& req, robotcontrol::MagCalibShowResponse& resp);
		bool clearCalibrationMarkers(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
		bool warpClearPoints(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
		bool warpAddPoint(robotcontrol::WarpAddPointRequest& req, robotcontrol::WarpAddPointResponse& resp);
		ros::ServiceServer m_srv_startCalibration;
		ros::ServiceServer m_srv_stopCalibration2D;
		ros::ServiceServer m_srv_stopCalibration3D;
		ros::ServiceServer m_srv_abortCalibration;
		ros::ServiceServer m_srv_showCalibration;
		ros::ServiceServer m_srv_clearCalibrationMarkers;
		ros::ServiceServer m_srv_warpClearPoints;
		ros::ServiceServer m_srv_warpAddPoint;

		// Calibration measurements vector
		config_server::Parameter<float> m_calibDataDebounce;
		std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_calibrationMeasurements;

		// Runtime auto-calibration
		typedef boost::circular_buffer<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > ACBuffer;
		config_server::Parameter<bool> m_autoCalibEnable;
		config_server::Parameter<bool> m_autoCalibClearData;
		config_server::Parameter<bool> m_autoCalibShowAllData;
		config_server::Parameter<bool> m_autoCalibShowDebug;
		config_server::Parameter<int> m_autoCalibBufSize;
		config_server::Parameter<int> m_autoCalibCountForUpdate;
		config_server::Parameter<float> m_autoCalibTimeForUpdate;
		config_server::Parameter<float> m_autoCalibTiltBound;  // The tilt (tilt angle/alpha) in the assumed quaternion for which the auto-calibration weight is just 0, where for zero tilt the weight is 1
		config_server::Parameter<float> m_autoCalibTiltBoundZ; // The rotation in radians (+ or -) of the nominal 2D calibration circle that produces Z values that have precisely zero weight in further auto-calibrations (i.e. BoundZ = AutoR*sin(TiltBoundZ), and new data points with z values of exactly AutoZ +- BoundZ will have weight 0, whereas data points with a z value of AutoZ will have weight 1)
		config_server::Parameter<bool> m_autoCalibUseTiltBoundZ;
		ACBuffer m_autoCalibBuf;
		int m_autoCalibBufCount;
		bool m_autoCalibClearDataBuf;
		ros::Time m_autoCalibLastUpdate;
		Eigen::Vector4d m_autoCalibFilterTarget;
		double m_autoCalibFilterTargetConf;
		double m_autoCalibOldWeight;
		void updateAutoCalibEnable();
		void updateAutoCalibBufSize() { m_autoCalibBuf.rset_capacity(m_autoCalibBufSize()); }
		void updateAutoCalibClearData() { if(m_autoCalibClearData()) { autoCalibClearBufData(); m_autoCalibClearData.set(false); } }
		void autoCalibClearBufData() { m_autoCalibClearDataBuf = true; }
		bool computeAutoCalib();
		config_server::Parameter<float> m_autoCalibMeanDistSmall;     // The largest 2D distance from the auto-calib XY centre to the weighted mean of the data, that still has a confidence of 0 for updating based on the ray towards the mean (in units of current auto-calib R)
		config_server::Parameter<float> m_autoCalibMeanDistLarge;     // The smallest 2D distance from the auto-calib XY centre to the weighted mean of the data, that has a confidence of 1 for updating based on the ray towards the mean (in units of current auto-calib R)
		config_server::Parameter<float> m_autoCalibSpreadAngleMin;    // The minimum allowed required spread angle for a spread angle confidence factor of 0.5
		config_server::Parameter<float> m_autoCalibSpreadAngleTol;    // The percent plus/minus tolerance on the required spread angle for a spread angle confidence factor of 0.5, that gives confidence factors of 1.0 and 0.0 respectively (e.g. 0.20 means that +-20% of the required 0.5 spread angle gives the required 1.0 and 0.0 spread angles)
		config_server::Parameter<float> m_autoCalibSpreadErrorMin;    // The circular fitting error for which a spread angle of SpreadAngleMin is required for a spread angle confidence factor of 0.5
		config_server::Parameter<float> m_autoCalibSpreadErrorMax;    // The circular fitting error for which a spread angle of 2*pi is required for a spread angle confidence factor of 0.5
		config_server::Parameter<float> m_autoCalibPlanarSpreadSmall; // The largest planar spread factor of the data (effective 2D standard deviation around the weighted mean), that still has a confidence of 1 for updating based on the ray towards the mean (in units of current auto-calib R)
		config_server::Parameter<float> m_autoCalibPlanarSpreadLarge; // The smallest planar spread factor of the data (effective 2D standard deviation around the weighted mean), that has a confidence of 0 for updating based on the ray towards the mean (in units of current auto-calib R)
		config_server::Parameter<float> m_autoCalibRadiusErrLimit;    // The allowed deviation between the expected auto-calib radius R and the fitted radius, above which the confidence of updating based on the fitted centre is zero due to excessive radial mismatch (in units of current auto-calib R)
		config_server::Parameter<float> m_autoCalibPreferRadius;      // A dimensionless parameter that configures how much more important low radial error is over high angular spread, in calculation of the confidence of the fitted target (0 => Equal importance, 1 => Radius somewhat more important than angular spread)
		config_server::Parameter<float> m_autoCalibReqFittedConf;     // Rescaling factor used for the fitted target confidence to make it comparable to the mean target confidence (0.5 => The fitted target confidences are doubled and coerced to [0,1], 1.0 => The fitted target confidences are not scaled)
		rc_utils::ConicFit::WeightedPoints2D m_autoCalibXYW;
		rc_utils::LimitedLowPass m_autoCalibFilterX;
		rc_utils::LimitedLowPass m_autoCalibFilterY;
		rc_utils::LimitedLowPass m_autoCalibFilterZ;
		rc_utils::LimitedLowPass m_autoCalibFilterR;
		config_server::Parameter<bool>  m_autoCalibResetToHard;   // Flag whether the current auto-calib filter values should be reset to the currently configured hard iron offset
		config_server::Parameter<bool>  m_autoCalibEnableXY;      // Flag whether to enable updates to the XY filter
		config_server::Parameter<bool>  m_autoCalibEnableZ;       // Flag whether to enable updates to the Z filter
		config_server::Parameter<bool>  m_autoCalibEnableR;       // Flag whether to enable updates to the R filter
		config_server::Parameter<float> m_autoCalibFilterTsXY;    // 90% settling time of the XY filter before the effects of the slope limiting
		config_server::Parameter<float> m_autoCalibFilterTsZ;     // 90% settling time of the Z filter before the effects of the slope limiting
		config_server::Parameter<float> m_autoCalibFilterTsR;     // 90% settling time of the R filter before the effects of the slope limiting
		config_server::Parameter<float> m_autoCalibFilterDeltaXY; // Maximum delta per second of the XY filter
		config_server::Parameter<float> m_autoCalibFilterDeltaZ;  // Maximum delta per second of the Z filter
		config_server::Parameter<float> m_autoCalibFilterDeltaR;  // Maximum delta per second of the R filter
		void updateAutoCalibFilterEnable();
		double getAutoCalibFilterScaler() { return m_cycleTime * rc_utils::coerce(m_autoCalibFilterTargetConf, 0.0, 1.0); }
		void handleAutoCalibFilterXY();
		void handleAutoCalibFilterZ();
		void handleAutoCalibFilterR();
		ros::ServiceServer m_srv_testAutoCalib;
		bool handleTestAutoCalib(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
		bool m_autoCalibTest;

		// Hard iron calibration parameters
		void updateHardOffset(bool resetAutoCalib);
		void setHardIronFromAutoCalib();
		config_server::Parameter<bool> m_enable_hard;
		config_server::Parameter<bool> m_hardSetFromAutoCalib;
		config_server::Parameter<float> m_hard_x;  //     | x |
		config_server::Parameter<float> m_hard_y;  // V = | y |
		config_server::Parameter<float> m_hard_z;  //     | z |
		config_server::Parameter<float> m_hard_B;  // Bc = Bm - V (B is just an estimate of the field strength and is not involved in the undistortion of magnetometer measurements)
		config_server::Parameter<float> m_hard_R;  // R is an estimate of the radius of the 2D xy circle produced by a pure yaw z-rotation of the sensor
		config_server::Parameter<float> m_hard_zR; // zR is the hard iron z-offset of the 2D xy circle (more precisely, of the centre of the fitted circle)
		Eigen::Vector3d m_hardOffset;
		double m_hardField;
		double m_hardRadius;
		double m_hardOffset2DZ;

		// Soft iron calibration parameters
		void updateSoftOffset() { m_softOffset << m_soft_hx(), m_soft_hy(), m_soft_hz(); }
		void updateSoftMatrix() { m_softMatrix << m_soft_w1(), m_soft_w2(), m_soft_w3(), m_soft_w2(), m_soft_w4(), m_soft_w5(), m_soft_w3(), m_soft_w5(), m_soft_w6(); m_softField = m_soft_B(); }
		config_server::Parameter<bool> m_enable_soft;
		config_server::Parameter<float> m_soft_hx; //     | hx |
		config_server::Parameter<float> m_soft_hy; // V = | hy |
		config_server::Parameter<float> m_soft_hz; //     | hz |
		config_server::Parameter<float> m_soft_w1; //
		config_server::Parameter<float> m_soft_w2; //        | w1 w2 w3 |
		config_server::Parameter<float> m_soft_w3; // Winv = | w2 w4 w5 | (symmetric)
		config_server::Parameter<float> m_soft_w4; //        | w3 w5 w6 |
		config_server::Parameter<float> m_soft_w5; //
		config_server::Parameter<float> m_soft_w6; // Bc = B*Winv*(Bm - V) is the formula to undistort magnetometer measurements (soft-iron)
		config_server::Parameter<float> m_soft_B;  //
		Eigen::Vector3d m_softOffset;
		Eigen::Matrix3d m_softMatrix;
		double m_softField;

		// Angle warping calibration parameters
		config_server::Parameter<bool> m_enable_warp;
		config_server::Parameter<std::string> m_warpParamString;
		config_server::Parameter<bool> m_warpUseDegrees;
		void handleWarpParamString();
		rc_utils::CyclicWarp m_warp;

		// Internal variables
		Eigen::Vector3d m_value;
		Eigen::Vector3d m_valueRaw;
		Eigen::Vector3d m_valueRawAuto;
		Eigen::Vector3d m_valuePreWarp;
		Eigen::Quaterniond m_quat;
		bool m_haveQuat;
		bool m_haveValueRaw;
		bool m_haveValueRawAuto;
		bool m_calibrating;

		// Visualisation marker manager
		config_server::Parameter<bool> m_showCalib;
		config_server::Parameter<bool> m_showAutoCalib;
		config_server::Parameter<bool> m_clearCalibMarkers;
		void handleShowCalib();
		void handleShowAutoCalib();
		void handleClearCalibMarkers() { if(m_clearCalibMarkers()) clearCalibMarkers(); }
		void clearCalibMarkers();
		void updateFixedPoints();
		void updateHardVis3D();
		void updateSoftVis3D();
		class MarkerMan : public vis_utils::MarkerManager
		{
		public:
			MarkerMan(const std::string& topicName, int publishInterval, bool enabled, const std::string& refFrame, const std::string& markerNs);
			void reset() { clearRawDataPoints(); clearAutoDataPoints(); clearFixedDataPoints(); hideAll(); }
			void addRawDataPoint(const Eigen::Vector3d& point) { addDataPoint(&RawData, point.x(), point.y(), point.z()); }
			void addRawDataPoint(double x, double y, double z) { addDataPoint(&RawData, x, y, z); }
			void clearRawDataPoints() { clearDataPoints(&RawData); }
			void addAutoDataPoint(const Eigen::Vector3d& point) { addDataPoint(&AutoData, point.x(), point.y(), point.z()); }
			void addAutoDataPoint(double x, double y, double z) { addDataPoint(&AutoData, x, y, z); }
			void addAutoDataColour(double r, double g, double b) { addDataColour(&AutoData, r, g, b); }
			void clearAutoDataPoints() { clearDataPoints(&AutoData); }
			void addFixedDataPoint(const Eigen::Vector3d& point) { addDataPoint(&FixedData, point.x(), point.y(), point.z()); }
			void addFixedDataPoint(double x, double y, double z) { addDataPoint(&FixedData, x, y, z); }
			void clearFixedDataPoints() { clearDataPoints(&FixedData); }
			void addDataPoint(vis_utils::GenMarker* marker, double x, double y, double z);
			void setDataPoint(vis_utils::GenMarker* marker, size_t index, double x, double y, double z);
			void setDataPoint(vis_utils::GenMarker* marker, size_t index, const Eigen::Vector3d& point) { setDataPoint(marker, index, point.x(), point.y(), point.z()); }
			void addDataColour(vis_utils::GenMarker* marker, double r, double g, double b) { std_msgs::ColorRGBA c; c.r = r; c.g = g; c.b = b; c.a = 1.0; marker->marker.colors.push_back(c); }
			void setDataColour(vis_utils::GenMarker* marker, size_t index, double r, double g, double b) { std_msgs::ColorRGBA& c = marker->marker.colors[index]; c.r = r; c.g = g; c.b = b; c.a = 1.0; }
			void clearDataPoints(vis_utils::GenMarker* marker) { marker->marker.points.clear(); marker->marker.colors.clear(); }
			void updateArrow(vis_utils::ArrowMarker* marker, const Eigen::Vector3d& from, const Eigen::Vector3d& vec);
			void updatePosition(vis_utils::GenMarker* marker, const Eigen::Vector3d& position) { updatePosition(marker, position.x(), position.y(), position.z()); }
			void updatePosition(vis_utils::GenMarker* marker, double x, double y, double z);
			void updateOrientation(vis_utils::GenMarker* marker, const Eigen::Quaterniond& orientation) { updateOrientation(marker, orientation.w(), orientation.x(), orientation.y(), orientation.z()); }
			void updateOrientation(vis_utils::GenMarker* marker, double w, double x, double y, double z);
			void updateScale(vis_utils::GenMarker* marker, double scale) { updateScale(marker, scale, scale, scale); }
			void updateScale(vis_utils::GenMarker* marker, double scaleX, double scaleY, double scaleZ);
			void setCirclePoints(vis_utils::GenMarker* marker, const Eigen::Vector3d& centre, double radius) { setEllipsePoints(marker, centre, radius, radius, 0.0); }
			void setEllipsePoints(vis_utils::GenMarker* marker, const Eigen::Vector3d& centre, double radiusX, double radiusY, double angle);
			void setAllLifetimes(double lifetime);
			void publishMarkers();
			void hideAll();
			bool visible;
			bool hideAuto;
			vis_utils::GenMarker    RawData;           // Input data points
			vis_utils::GenMarker    AutoData;          // Auto
			vis_utils::GenMarker    FixedData;         // Soft 3D
			vis_utils::GenMarker    HardCircle;        // Hard 2D
			vis_utils::SphereMarker HardCircleC;       // Hard 2D
			vis_utils::SphereMarker HardSphere;        // Hard 2D & 3D
			vis_utils::SphereMarker HardSphereC;       // Hard 2D & 3D
			vis_utils::ArrowMarker  HardPlaneNormal;   // Hard 2D
			vis_utils::GenMarker    SoftEllipse;       // Soft 2D
			vis_utils::SphereMarker SoftEllipseC;      // Soft 2D
			vis_utils::SphereMarker SoftEllipsoid;     // Soft 3D
			vis_utils::SphereMarker SoftEllipsoidC;    // Soft 3D
			vis_utils::SphereMarker SoftFixedSphere;   // Soft 3D
			vis_utils::SphereMarker SoftFixedSphereC;  // Soft 3D
			vis_utils::GenMarker    AutoDataVar;       // Auto
			vis_utils::SphereMarker AutoDataMean;      // Auto
			vis_utils::TextMarker   AutoDataText;      // Auto
			vis_utils::GenMarker    AutoMeanCircle;    // Auto
			vis_utils::SphereMarker AutoMeanCircleC;   // Auto
			vis_utils::TextMarker   AutoMeanText;      // Auto
			vis_utils::GenMarker    AutoFittedCircle;  // Auto
			vis_utils::SphereMarker AutoFittedCircleC; // Auto
			vis_utils::TextMarker   AutoFittedText;    // Auto
			vis_utils::GenMarker    AutoTargetCircle;  // Auto
			vis_utils::SphereMarker AutoTargetCircleC; // Auto
			vis_utils::TextMarker   AutoTargetText;    // Auto
			vis_utils::GenMarker    AutoCurCircle;     // Auto
			vis_utils::SphereMarker AutoCurCircleC;    // Auto
			vis_utils::TextMarker   AutoCurText;       // Auto
		};
		MarkerMan MM;

		// Plot manager
		config_server::Parameter<bool> m_plotData;
		plot_msgs::PlotManagerFS PM;
		void configurePlotManager();
		void callbackPlotData() { PM.setEnabled(m_plotData()); }
		enum PMIDS
		{
			PM_MAG_INPUT_X,
			PM_MAG_INPUT_Y,
			PM_MAG_INPUT_Z,
			PM_MAG_OUTPUT_X,
			PM_MAG_OUTPUT_Y,
			PM_MAG_OUTPUT_Z,
			PM_MAG_DEB_X,
			PM_MAG_DEB_Y,
			PM_MAG_DEB_Z,
			PM_MAG_DEB_W,
			PM_AUTO_COUNT,
			PM_AUTO_TIME,
			PM_HARD_X,
			PM_HARD_Y,
			PM_HARD_Z,
			PM_HARD_R,
			PM_HARD_ZR,
			PM_HARDTGT_X,
			PM_HARDTGT_Y,
			PM_HARDTGT_Z,
			PM_HARDTGT_R,
			PM_HARDTGT_ZR,
			PM_COUNT
		};
	};
}

#endif
// EOF