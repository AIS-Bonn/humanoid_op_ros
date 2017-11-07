// Magnetometer filter that performs data correction
// File: magfilter.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <robotcontrol/hw/magfilter.h>
#include <visualization_msgs/MarkerArray.h>
#include <rc_utils/math_vec_mat.h>
#include <rc_utils/planefit.h>
#include <rot_conv/rot_conv.h>
#include <sstream>
#include <cmath>

// Defines
#define MM_SCALE             2.0
#define MM_OFFSET_Z          2.0
#define MM_LIFE_SHORT        0.05
#define MM_LIFE_ETERNITY     0.0
#define MM_POINT_SIZE        0.02
#define MM_LINE_SIZE         0.02
#define MM_CENTRE_SIZE       0.05
#define MM_ELLIPSE_POINTS    40
#define MM_TEXT_SIZE         0.08
#define MM_TEXT_OFFSET_ZP    0.04
#define MM_TEXT_OFFSET_ZN    -0.09
#define MM_ARROW_SHAFT_DIAM  0.02
#define MM_ARROW_HEAD_DIAM   0.04
#define MM_ARROW_HEAD_LEN    0.07

// Namespaces
using namespace stateestimation;
using namespace rc_utils;

//
// MagFilter class
//

// Constructor/destructor
MagFilter::MagFilter(const std::string& resourcePath, const std::string& configParamPath, double cycleTime)
	: m_cycleTime(0.010)
	, m_calibDataDebounce(configParamPath + "calibDataDebounce", 0.0, 0.0005, 0.02, 0.008)
	, m_autoCalibEnable(configParamPath + "autoCalib/enable", false)
	, m_autoCalibClearData(configParamPath + "autoCalib/clearBufData", false)
	, m_autoCalibShowAllData(configParamPath + "autoCalib/showAllData", false)
	, m_autoCalibShowDebug(configParamPath + "autoCalib/showDebugMessages", false)
	, m_autoCalibBufSize(configParamPath + "autoCalib/bufSize", 10, 5, 500, 100)
	, m_autoCalibCountForUpdate(configParamPath + "autoCalib/countForUpdate", 10, 5, 500, 100)
	, m_autoCalibTimeForUpdate(configParamPath + "autoCalib/timeForUpdate", 1.0, 0.5, 60.0, 10.0)
	, m_autoCalibTiltBound(configParamPath + "autoCalib/tiltBound", 0.002, 0.002, 0.2, 0.05)
	, m_autoCalibTiltBoundZ(configParamPath + "autoCalib/tiltBoundZ", 0.005, 0.005, 0.4, 0.2)
	, m_autoCalibUseTiltBoundZ(configParamPath + "autoCalib/useTiltBoundZ", true)
	, m_autoCalibBuf()
	, m_autoCalibBufCount(0)
	, m_autoCalibClearDataBuf(false)
	, m_autoCalibLastUpdate(0, 0)
	, m_autoCalibFilterTarget(Eigen::Vector4d::Zero())
	, m_autoCalibFilterTargetConf(0.0)
	, m_autoCalibOldWeight(0.0)
	, m_autoCalibMeanDistSmall(configParamPath + "autoCalib/confidences/meanDistSmall", 0.0, 0.01, 1.0, 0.15)
	, m_autoCalibMeanDistLarge(configParamPath + "autoCalib/confidences/meanDistLarge", 0.0, 0.01, 1.0, 0.50)
	, m_autoCalibSpreadAngleMin(configParamPath + "autoCalib/confidences/spreadAngleMin", 0.4, 0.01, 1.0, 0.7)
	, m_autoCalibSpreadAngleTol(configParamPath + "autoCalib/confidences/spreadAngleTol", 0.01, 0.01, 0.4, 0.2)
	, m_autoCalibSpreadErrorMin(configParamPath + "autoCalib/confidences/spreadErrorMin", 0.0, 0.001, 0.1, 0.02)
	, m_autoCalibSpreadErrorMax(configParamPath + "autoCalib/confidences/spreadErrorMax", 0.05, 0.002, 0.25, 0.13)
	, m_autoCalibPlanarSpreadSmall(configParamPath + "autoCalib/confidences/planarSpreadSmall", 0.0, 0.01, 1.0, 0.1)
	, m_autoCalibPlanarSpreadLarge(configParamPath + "autoCalib/confidences/planarSpreadLarge", 0.0, 0.01, 1.0, 0.3)
	, m_autoCalibRadiusErrLimit(configParamPath + "autoCalib/confidences/radiusErrLimit", 0.01, 0.01, 1.0, 0.15)
	, m_autoCalibPreferRadius(configParamPath + "autoCalib/confidences/preferRadius", 0.0, 0.02, 2.0, 0.5)
	, m_autoCalibReqFittedConf(configParamPath + "autoCalib/confidences/requiredFittedConf", 0.1, 0.01, 1.0, 1.0)
	, m_autoCalibResetToHard(configParamPath + "autoCalib/resetToHardIron", false)
	, m_autoCalibEnableXY(configParamPath + "autoCalib/enableFilter/enableFilterXY", false)
	, m_autoCalibEnableZ(configParamPath + "autoCalib/enableFilter/enableFilterZ", false)
	, m_autoCalibEnableR(configParamPath + "autoCalib/enableFilter/enableFilterR", false)
	, m_autoCalibFilterTsXY(configParamPath + "autoCalib/filter/TsXY", 5.0, 0.5, 60.0, 20.0)
	, m_autoCalibFilterTsZ(configParamPath + "autoCalib/filter/TsZ", 10.0, 0.5, 100.0, 50.0)
	, m_autoCalibFilterTsR(configParamPath + "autoCalib/filter/TsR", 10.0, 0.5, 100.0, 50.0)
	, m_autoCalibFilterDeltaXY(configParamPath + "autoCalib/filter/deltaXY", 0.0, 0.0005, 0.05, 0.005)
	, m_autoCalibFilterDeltaZ(configParamPath + "autoCalib/filter/deltaZ", 0.0, 0.0002, 0.02, 0.002)
	, m_autoCalibFilterDeltaR(configParamPath + "autoCalib/filter/deltaR", 0.0, 0.0002, 0.02, 0.002)
	, m_autoCalibTest(false)
	, m_enable_hard(configParamPath + "hard/enableHardIron", false)
	, m_hardSetFromAutoCalib(configParamPath + "hard/setFromAutoCalib", false)
	, m_hard_x(configParamPath + "hard/x", -3.0, 0.01, 3.0, 0.0)
	, m_hard_y(configParamPath + "hard/y", -3.0, 0.01, 3.0, 0.0)
	, m_hard_z(configParamPath + "hard/z", -3.0, 0.01, 3.0, 0.0)
	, m_hard_B(configParamPath + "hard/B", 0.0, 0.02, 3.0, 1.0)
	, m_hard_R(configParamPath + "hard/R", 0.0, 0.02, 3.0, 1.0)
	, m_hard_zR(configParamPath + "hard/zR", -3.0, 0.01, 3.0, 0.0)
	, m_hardOffset(Eigen::Vector3d::Zero())
	, m_hardField(1.0)
	, m_hardRadius(1.0)
	, m_hardOffset2DZ(0.0)
	, m_enable_soft(configParamPath + "soft/enableSoftIron", false)
	, m_soft_hx(configParamPath + "soft/hx", -3.0, 0.01, 3.0, 0.0)
	, m_soft_hy(configParamPath + "soft/hy", -3.0, 0.01, 3.0, 0.0)
	, m_soft_hz(configParamPath + "soft/hz", -3.0, 0.01, 3.0, 0.0)
	, m_soft_w1(configParamPath + "soft/w1", -10.0, 0.05, 10.0, 1.0)
	, m_soft_w2(configParamPath + "soft/w2", -10.0, 0.05, 10.0, 0.0)
	, m_soft_w3(configParamPath + "soft/w3", -10.0, 0.05, 10.0, 0.0)
	, m_soft_w4(configParamPath + "soft/w4", -10.0, 0.05, 10.0, 1.0)
	, m_soft_w5(configParamPath + "soft/w5", -10.0, 0.05, 10.0, 0.0)
	, m_soft_w6(configParamPath + "soft/w6", -10.0, 0.05, 10.0, 1.0)
	, m_soft_B(configParamPath + "soft/B", 0.0, 0.02, 3.0, 1.0)
	, m_softOffset(Eigen::Vector3d::Zero())
	, m_softMatrix(Eigen::Matrix3d::Identity())
	, m_softField(1.0)
	, m_enable_warp(configParamPath + "warp/enableAngleWarping", false)
	, m_warpParamString(configParamPath + "warp/paramString", "")
	, m_warpUseDegrees(configParamPath + "warp/useDegrees", true)
	, m_warp(CyclicWarp::RADIANS)
	, m_value(Eigen::Vector3d::Zero())
	, m_valueRaw(Eigen::Vector3d::Zero())
	, m_valueRawAuto(Eigen::Vector3d::Zero())
	, m_valuePreWarp(Eigen::Vector3d::Zero())
	, m_quat(Eigen::Quaterniond::Identity())
	, m_haveQuat(false)
	, m_haveValueRaw(false)
	, m_haveValueRawAuto(false)
	, m_calibrating(false)
	, m_showCalib(configParamPath + "showCalib", false)
	, m_showAutoCalib(configParamPath + "showAutoCalib", false)
	, m_clearCalibMarkers(configParamPath + "clearCalibMarkers", false)
	, MM("~mag_calib_data", 2, true, "/ego_rot", configParamPath)
	, m_plotData(configParamPath + "plotData", false)
	, PM(PM_COUNT, resourcePath)
{
	// Set the cycle time
	setCycleTime(cycleTime);

	// Retrieve a ROS node handle
	ros::NodeHandle nhs;

	// Configure the plot manager
	configurePlotManager();

	// Clear the current assumed quaternion orientation
	clearOrientation();

	// Advertise the provided ROS services
	m_srv_startCalibration        = nhs.advertiseService(resourcePath + "startCalibration" , &MagFilter::startCalibration , this);
	m_srv_stopCalibration2D       = nhs.advertiseService(resourcePath + "stopCalibration2D", &MagFilter::stopCalibration2D, this);
	m_srv_stopCalibration3D       = nhs.advertiseService(resourcePath + "stopCalibration3D", &MagFilter::stopCalibration3D, this);
	m_srv_abortCalibration        = nhs.advertiseService(resourcePath + "abortCalibration", &MagFilter::abortCalibration, this);
	m_srv_showCalibration         = nhs.advertiseService(resourcePath + "showCalibration", &MagFilter::showCalibration, this);
	m_srv_clearCalibrationMarkers = nhs.advertiseService(resourcePath + "clearCalibrationMarkers", &MagFilter::clearCalibrationMarkers, this);
	m_srv_warpClearPoints         = nhs.advertiseService(resourcePath + "warpClearPoints", &MagFilter::warpClearPoints, this);
	m_srv_warpAddPoint            = nhs.advertiseService(resourcePath + "warpAddPoint", &MagFilter::warpAddPoint, this);
	m_srv_testAutoCalib           = nhs.advertiseService(resourcePath + "testAutoCalib", &MagFilter::handleTestAutoCalib, this);

	// Set up auto-calibration parameter callbacks (should be before the hard iron parameter callbacks are configured)
	m_autoCalibEnable.setCallback(boost::bind(&MagFilter::updateAutoCalibEnable, this), true);
	m_autoCalibClearData.setCallback(boost::bind(&MagFilter::updateAutoCalibClearData, this), true);
	m_autoCalibShowAllData.setCallback(boost::bind(&MagFilter::autoCalibClearBufData, this), true);
	m_autoCalibBufSize.setCallback(boost::bind(&MagFilter::updateAutoCalibBufSize, this), true);
	m_autoCalibResetToHard.setCallback(boost::bind(&MagFilter::updateHardOffset, this, false), false);
	boost::function<void()> autoFilterEnableCB = boost::bind(&MagFilter::updateAutoCalibFilterEnable, this);
	m_autoCalibEnableXY.setCallback(boost::bind(autoFilterEnableCB));
	m_autoCalibEnableZ.setCallback(boost::bind(autoFilterEnableCB));
	m_autoCalibEnableR.setCallback(boost::bind(autoFilterEnableCB));
	updateAutoCalibFilterEnable();

	// Set up the hard iron calibration parameter callbacks
	m_hardSetFromAutoCalib.set(false);
	m_hardSetFromAutoCalib.setCallback(boost::bind(&MagFilter::setHardIronFromAutoCalib, this), false);
	boost::function<void()> hardOffsetCB = boost::bind(&MagFilter::updateHardOffset, this, true);
	m_hard_x.setCallback(boost::bind(hardOffsetCB));
	m_hard_y.setCallback(boost::bind(hardOffsetCB));
	m_hard_z.setCallback(boost::bind(hardOffsetCB));
	m_hard_B.setCallback(boost::bind(hardOffsetCB));
	m_hard_R.setCallback(boost::bind(hardOffsetCB));
	m_hard_zR.setCallback(boost::bind(hardOffsetCB));
	updateHardOffset(true);

	// Set up the soft iron calibration parameter callbacks
	boost::function<void()> softOffsetCB = boost::bind(&MagFilter::updateSoftOffset, this);
	m_soft_hx.setCallback(boost::bind(softOffsetCB));
	m_soft_hy.setCallback(boost::bind(softOffsetCB));
	m_soft_hz.setCallback(boost::bind(softOffsetCB));
	updateSoftOffset();
	boost::function<void()> softMatrixCB = boost::bind(&MagFilter::updateSoftMatrix, this);
	m_soft_w1.setCallback(boost::bind(softMatrixCB));
	m_soft_w2.setCallback(boost::bind(softMatrixCB));
	m_soft_w3.setCallback(boost::bind(softMatrixCB));
	m_soft_w4.setCallback(boost::bind(softMatrixCB));
	m_soft_w5.setCallback(boost::bind(softMatrixCB));
	m_soft_w6.setCallback(boost::bind(softMatrixCB));
	m_soft_B.setCallback(boost::bind(softMatrixCB));
	updateSoftMatrix();

	// Set up the angle warping parameter callbacks
	boost::function<void()> warpStringCB = boost::bind(&MagFilter::handleWarpParamString, this);
	m_warpParamString.setCallback(boost::bind(warpStringCB));
	m_warpUseDegrees.setCallback(boost::bind(warpStringCB));
	handleWarpParamString();

	// Set up more auto-calibration callbacks (should be after the hard iron parameter callbacks are configured)
	m_autoCalibFilterTsXY.setCallback(boost::bind(&MagFilter::handleAutoCalibFilterXY, this));
	m_autoCalibFilterTsZ.setCallback(boost::bind(&MagFilter::handleAutoCalibFilterZ, this));
	m_autoCalibFilterTsR.setCallback(boost::bind(&MagFilter::handleAutoCalibFilterR, this));
	m_autoCalibFilterDeltaXY.setCallback(boost::bind(&MagFilter::handleAutoCalibFilterXY, this), true);
	m_autoCalibFilterDeltaZ.setCallback(boost::bind(&MagFilter::handleAutoCalibFilterZ, this), true);
	m_autoCalibFilterDeltaR.setCallback(boost::bind(&MagFilter::handleAutoCalibFilterR, this), true);

	// Set up the visualisation marker parameter callbacks
	m_showCalib.setCallback(boost::bind(&MagFilter::handleShowCalib, this), true);
	m_showAutoCalib.setCallback(boost::bind(&MagFilter::handleShowAutoCalib, this), true);
	m_clearCalibMarkers.setCallback(boost::bind(&MagFilter::handleClearCalibMarkers, this), true);

	// Set up plot manager parameter callbacks
	m_plotData.setCallback(boost::bind(&MagFilter::callbackPlotData, this), true);
}

// Set cycle time function
void MagFilter::setCycleTime(double cycleTime)
{
	// Update the cycle time
	if(cycleTime <= 0.0) return;
	m_cycleTime = cycleTime;

	// Update one-time calculated values that depend on the cycle time
	handleAutoCalibFilterXY();
	handleAutoCalibFilterZ();
	handleAutoCalibFilterR();
}

// Update functions
void MagFilter::update(const Eigen::Vector3d& mag)
{
	// Retrieve the current ROS time
	ros::Time now = ros::Time::now();

	// Clear the plot manager
	PM.clear(now);

	// Initialise the magnetometer value
	Eigen::Vector3d magValue = mag;

	// Calculate the required hard iron offset
	Eigen::Vector3d hardOffset = m_hardOffset;
	double hardOffset2DZ = m_hardOffset2DZ;
	double hardRadius = m_hardRadius;
	if(m_autoCalibEnable())
	{
		// Decide on appropriate slope limits in the XY plane to ensure that the direction of fading is towards the XY target
		Eigen::Vector2d targetDirnXY(fabs(m_autoCalibFilterTarget.x() - m_autoCalibFilterX.value()), fabs(m_autoCalibFilterTarget.y() - m_autoCalibFilterY.value()));
		double dxty = m_autoCalibFilterX.maxDelta() * targetDirnXY.y();
		double dytx = m_autoCalibFilterY.maxDelta() * targetDirnXY.x();
		double deltaScaleX = 1.0, deltaScaleY = 1.0;
		if(dxty > dytx && dxty > 0.0)
			deltaScaleX = dytx/dxty;
		if(dytx > dxty && dytx > 0.0)
			deltaScaleY = dxty/dytx;
		double maxDeltaX = m_autoCalibFilterX.maxDelta() * deltaScaleX;
		double maxDeltaY = m_autoCalibFilterY.maxDelta() * deltaScaleY;

		// Update the auto-calibration filters
		m_autoCalibFilterX.put(m_autoCalibFilterTarget.x(), maxDeltaX);
		m_autoCalibFilterY.put(m_autoCalibFilterTarget.y(), maxDeltaY);
		m_autoCalibFilterZ.put(m_autoCalibFilterTarget.z());
		m_autoCalibFilterR.put(m_autoCalibFilterTarget.w());

		// Retrieve the current auto-calibrated hard offset
		hardOffset << m_autoCalibFilterX.value(), m_autoCalibFilterY.value(), m_autoCalibFilterZ.value() + m_hardOffset.z() - m_hardOffset2DZ;
		hardOffset2DZ = m_autoCalibFilterZ.value();
		hardRadius = m_autoCalibFilterR.value();
	}

	// Apply the hard iron calibration if enabled
	if(m_enable_hard())
		magValue -= hardOffset;

	// Apply the soft iron calibration if enabled
	if(m_enable_soft())
		magValue = m_softField * m_softMatrix * (magValue - m_softOffset);

	// Apply the angle warping calibration if enabled
	m_valuePreWarp = magValue;
	if(m_enable_warp())
	{
		double len = sqrt(magValue.x()*magValue.x() + magValue.y()*magValue.y());
		double yaw = atan2(-magValue.y(), magValue.x()); // The negation is intentional => This should be the heading on the field based on the magnetometer measurement assuming that the global magnetic field points towards the positive goal
		double wyaw = m_warp.unwarp(yaw);                // Unwarp the magnetometer value using cyclic warping
		magValue.x() = len*cos(-wyaw);                   // The negation is intentional => See above
		magValue.y() = len*sin(-wyaw);                   // The negation is intentional => See above
	}

	// Store the final corrected magnetometer value
	m_value = magValue;

	// Initialise a flag whether or not we wish to publish markers
	bool publishMarkers = (!m_calibrating && m_showCalib());

	// Clear the auto calib data buffer if necessary
	if(m_autoCalibClearDataBuf)
	{
		m_autoCalibClearDataBuf = false;
		m_autoCalibOldWeight = 0.0;
		m_autoCalibBuf.clear();
		m_autoCalibBufCount = 0;
		m_haveValueRawAuto = false;
		MM.clearAutoDataPoints();
		publishMarkers = true;
	}

	// Debounce the magnetometer data for calibration purposes
	if(!m_haveValueRaw || (mag - m_valueRaw).norm() >= m_calibDataDebounce())
	{
		// Update the last accepted debounced magnetometer value
		m_valueRaw = mag;
		m_haveValueRaw = true;

		// If running a normal 2D/3D calibration then save the debounced magnetometer data point
		if(m_calibrating)
		{
			m_calibrationMeasurements.push_back(mag);
			MM.addRawDataPoint(mag);
			publishMarkers = true;
		}
	}

	// Debounce the magnetometer data for auto-calibration purposes
	double timeSinceLastUpdate = (now - m_autoCalibLastUpdate).toSec();
	if(m_autoCalibLastUpdate.isZero())
		timeSinceLastUpdate = m_autoCalibTimeForUpdate(); // Force the time criterion to be satisfied...
	double weight = 0.0;
	if(m_autoCalibEnable() && m_haveQuat)
	{
		double curTilt = acos(coerceAbs(1.0 - 2.0*(m_quat.x()*m_quat.x() + m_quat.y()*m_quat.y()), 1.0)); // Current tilt angle
		double normedTilt = curTilt / m_autoCalibTiltBound();
		double tiltWeight = coerce(1.0 - normedTilt*normedTilt, 0.0, 1.0);
		double boundZ = m_autoCalibFilterR.value() * sin(m_autoCalibTiltBoundZ());
		double normedZ = (boundZ > 0.0 ? (mag.z() - m_autoCalibFilterZ.value()) / boundZ : 100.0); // If boundZ <= 0.0 then all weights should end up non-positive (all data points are ignored)
		double tiltZWeight = coerce(1.0 - fabs(normedZ*normedZ*normedZ), 0.0, 1.0);
		if(m_autoCalibUseTiltBoundZ())
			weight = coerce(tiltWeight*tiltZWeight, 0.0, 1.0);
		else
			weight = tiltWeight;
	}
	if(!m_haveValueRawAuto || (mag - m_valueRawAuto).norm() >= m_calibDataDebounce() || (weight > 0.0 && m_autoCalibOldWeight <= 0.0) || m_autoCalibTest)
	{
		// Update the last accepted debounced magnetometer value
		m_valueRawAuto = mag;
		m_haveValueRawAuto = true;
		m_autoCalibOldWeight = weight;

		// If auto-calibration is enabled then save the debounced magnetometer data point and recompute the auto-calibration if it is time
		if(m_autoCalibEnable() || m_autoCalibTest)
		{
			if(weight > 0.0 && !m_autoCalibTest)
			{
				m_autoCalibBuf.push_back(Eigen::Vector4d(mag.x(), mag.y(), mag.z(), weight));
				m_autoCalibBufCount++;
				MM.addAutoDataPoint(mag);
				MM.addAutoDataColour(0.6 + 0.4*weight, 0.6*(1.0 - weight), 0.6*(1.0 - weight));
				publishMarkers = true;
			}
			else if(m_autoCalibShowAllData())
			{
				MM.addAutoDataPoint(mag);
				MM.addAutoDataColour(0.6 + 0.4*weight, 0.6*(1.0 - weight), 0.6*(1.0 - weight));
				publishMarkers = true;
			}
			if(((m_autoCalibBufCount >= m_autoCalibCountForUpdate()) || (timeSinceLastUpdate >= m_autoCalibTimeForUpdate())) && !m_autoCalibShowAllData())
			{
				size_t size = m_autoCalibBuf.size();
				MM.AutoData.marker.points.resize(size);
				MM.AutoData.marker.colors.resize(size);
				for(size_t i = 0; i < size; i++)
				{
					const Eigen::Vector4d& point = m_autoCalibBuf[i];
					double weight = point.w();
					MM.setDataPoint(&MM.AutoData, i, point.head<3>());
					MM.setDataColour(&MM.AutoData, i, 0.6 + 0.4*weight, 0.6*(1.0 - weight), 0.6*(1.0 - weight));
				}
				computeAutoCalib();
				publishMarkers = true;
			}
		}
	}

	// Reset the auto-calib test variable
	m_autoCalibTest = false;

	// Plot variables
	if(PM.getEnabled())
	{
		PM.plotScalar(mag.x(), PM_MAG_INPUT_X);
		PM.plotScalar(mag.y(), PM_MAG_INPUT_Y);
		PM.plotScalar(mag.z(), PM_MAG_INPUT_Z);
		PM.plotScalar(m_value.x(), PM_MAG_OUTPUT_X);
		PM.plotScalar(m_value.y(), PM_MAG_OUTPUT_Y);
		PM.plotScalar(m_value.z(), PM_MAG_OUTPUT_Z);
		PM.plotScalar(m_valueRaw.x(), PM_MAG_DEB_X);
		PM.plotScalar(m_valueRaw.y(), PM_MAG_DEB_Y);
		PM.plotScalar(m_valueRaw.z(), PM_MAG_DEB_Z);
		PM.plotScalar(weight, PM_MAG_DEB_W);
		PM.plotScalar(((double) m_autoCalibBufCount) / m_autoCalibCountForUpdate(), PM_AUTO_COUNT);
		PM.plotScalar(timeSinceLastUpdate / m_autoCalibTimeForUpdate(), PM_AUTO_TIME);
		PM.plotScalar(hardOffset.x(), PM_HARD_X);
		PM.plotScalar(hardOffset.y(), PM_HARD_Y);
		PM.plotScalar(hardOffset.z(), PM_HARD_Z);
		PM.plotScalar(hardRadius, PM_HARD_R);
		PM.plotScalar(hardOffset2DZ, PM_HARD_ZR);
		PM.plotScalar(m_autoCalibFilterTarget.x(), PM_HARDTGT_X);
		PM.plotScalar(m_autoCalibFilterTarget.y(), PM_HARDTGT_Y);
		PM.plotScalar(m_autoCalibFilterTarget.z() + m_hardOffset.z() - m_hardOffset2DZ, PM_HARDTGT_Z);
		PM.plotScalar(m_autoCalibFilterTarget.w(), PM_HARDTGT_R);
		PM.plotScalar(m_autoCalibFilterTarget.z(), PM_HARDTGT_ZR);
		PM.publish();
	}

	// Publish markers if required
	if(publishMarkers)
	{
		if(MM.willPublish())
		{
			bool show = m_showAutoCalib();
			Eigen::Vector3d curAutoCalib(m_autoCalibFilterX.value(), m_autoCalibFilterY.value(), m_autoCalibFilterZ.value());
			MM.setCirclePoints(&MM.AutoCurCircle, curAutoCalib, m_autoCalibFilterR.value());
			MM.updatePosition(&MM.AutoCurCircleC, curAutoCalib);
			MM.updatePosition(&MM.AutoCurText, curAutoCalib.x(), curAutoCalib.y(), curAutoCalib.z() + MM_TEXT_OFFSET_ZP);
			MM.AutoCurCircle.setVisible(show);
			MM.AutoCurCircleC.setVisible(show);
			MM.AutoCurText.setVisible(show);
		}
		MM.publishMarkers();
	}
}

// Start calibration service handler
bool MagFilter::startCalibration(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	// To use this service to calibrate the magnetometer, have this magnetometer filter running and constantly receiving new
	// data points from the sensor. After calling this service (startCalibration), all data points are stored in a buffer until
	// a stop calibration service is called (either stopCalibration2D or stopCalibration3D). In the meantime, try to rotate the
	// robot in all possible directions (upside down, face down/up/left/right, etc...) for a 3D calibration, or just in all yaw
	// directions while being perfectly upright for a 2D calibration. Once the calibration has stopped you can read the results
	// from the return data of the service calls, or from the config server values that were changed. If the calibration is good
	// then make sure you save the config variables before the config server is stopped! You can also manually transcribe the
	// values if necessary to the required config_*.yaml file. In any case, if you don't save the variables then they are lost.
	// You can visualise the calibration process by enabling the plotCalibData config parameter. For this you will need to set
	// up a visualisation marker array display in rviz that listens to the configured topic.

	// Inform the user that the magnetometer calibration has started
	ROS_INFO("Starting magnetometer calibration...");
	ROS_INFO("2D calib: Keep the robot perfectly upright and rotate it in pure global yaw");
	ROS_INFO("3D calib: Rotate the robot in all possible orientations including upside-down");

	// Set the internal calibrating flag
	m_calibrating = true;

	// Clear the vector of calibration measurements
	m_calibrationMeasurements.clear();

	// Update the visualisation markers
	MM.reset();
	MM.visible = true;
	MM.forcePublish();
	MM.publishMarkers();

	// Return that the service call was successfully handled
	return true;
}

// Stop calibration 2D service handler
bool MagFilter::stopCalibration2D(robotcontrol::MagCalib2DRequest& req, robotcontrol::MagCalib2DResponse& resp)
{
	// This calibration method assumes that the robot was rotated purely about its z-axis while upright.

	// Don't do anything if a calibration isn't active
	if(!m_calibrating)
	{
		ROS_INFO("StopCalibration2D: Service call received even though no calibration is active");
		if(m_showCalib())
		{
			ROS_INFO("StopCalibration2D: Republishing data points of last calibration");
			MM.forcePublish();
			MM.publishMarkers();
		}
		resp = robotcontrol::MagCalib2DResponse();
		return true;
	}

	// Indicate to the user that this service call has been received
	ROS_INFO("Stopping magnetometer calibration (2D) with %d data points...", (int)m_calibrationMeasurements.size());

	// 2D plane calibration
	Eigen::Vector3d planeNormal, planeMean;
	double planeTiltAxisAngle, planeTiltAngle;
	PlaneFit::fitPlane(m_calibrationMeasurements, planeNormal, planeMean);
	if(planeNormal.z() < 0.0)
		planeNormal = -planeNormal;
	eigenNormalize(planeNormal); // Just to be extra safe...
	rot_conv::TiltFromZVec(planeNormal, planeTiltAxisAngle, planeTiltAngle);

	// Report the results of the 2D plane calibration
	ROS_INFO("2D plane: NormalVec(%.5f, %.5f, %.5f)", planeNormal.x(), planeNormal.y(), planeNormal.z());
	ROS_INFO("          Plane TiltAxisAngle(%.5f), Plane TiltAngle(%.5f)", planeTiltAxisAngle, planeTiltAngle);

	// Hard iron calibration variables
	Eigen::Vector3d hardOffset2D;
	double hardRadius2D, hardRadius3D, hardErr2D, hardErr3D;

	// 2D hard iron calibration: Fit a circle to the data (independent of the z values)
	ConicFit::fitCircle(m_calibrationMeasurements, hardOffset2D, hardRadius2D);
	hardErr2D = ConicFit::fitCircleError(m_calibrationMeasurements, hardOffset2D, hardRadius2D);

	// Update the hard iron calibration parameters on the config server
	m_hard_x.set(hardOffset2D.x());
	m_hard_y.set(hardOffset2D.y());
	m_hard_zR.set(hardOffset2D.z());
	m_hard_R.set(hardRadius2D);
	updateHardOffset(false);

	// 3D hard iron calibration: Fit a sphere to the data using the newly updated hard iron offset (to estimate the field strength, implicitly uses the existing hard offset Z as we don't have data with which we can estimate a new one)
	ConicFit::fitSphereCentred(m_calibrationMeasurements, m_hardOffset, hardRadius3D);
	hardErr3D = ConicFit::fitSphereError(m_calibrationMeasurements, m_hardOffset, hardRadius3D);

	// Update the hard iron calibration parameters on the config server
	m_hard_B.set(hardRadius3D);
	updateHardOffset(true);

	// Report the results of the hard iron calibration
	ROS_INFO("Hard: Offset(%.5lf, %.5lf, %.5lf), Field(%.5lf), Err3D(%.2lg)", m_hardOffset.x(), m_hardOffset.y(), m_hardOffset.z(), m_hardField, hardErr3D);
	ROS_INFO("      Offset2DZ(%.5lf), Field2D(%.5lf), Err2D(%.2lg)", m_hardOffset2DZ, m_hardRadius, hardErr2D);

	// Soft iron calibration variables
	Eigen::Vector3d softEllipseC;
	Eigen::Matrix2d softEllipseA, softEllipseQ, softEllipseWinv;
	Eigen::Vector2d softEllipseR;
	double softEllipseAngle, softEllipseErr;
	bool ellipseSuccess = true;

	// Fit an ellipse to the data (independent of the z values)
	ConicFit::fitEllipse(m_calibrationMeasurements, softEllipseC, softEllipseA);
	ellipseSuccess &= ConicFit::ellipseMatrixToAxes(softEllipseA, softEllipseQ, softEllipseR, softEllipseAngle);
	ellipseSuccess &= ConicFit::ellipseAxesToTransform(softEllipseQ, softEllipseR, softEllipseWinv);
	softEllipseErr = ConicFit::fitEllipseError(m_calibrationMeasurements, softEllipseC, softEllipseWinv);

	// Display a warning if the 2D ellipse fitting failed
	if(ellipseSuccess)
		ROS_INFO("Soft: Fitted an ellipse to the 2D data with error %.2lg (for vis only)", softEllipseErr);
	else
		ROS_WARN("Soft: Fitting of ellipse to 2D data failed!");

	// Unset the internal calibrating flag
	m_calibrating = false;

	// Clear the list of fixed data points
	MM.clearFixedDataPoints();

	// Update and publish the required visualisation markers
	MM.hideAll();
	MM.setCirclePoints(&MM.HardCircle, hardOffset2D, hardRadius2D);
	MM.updatePosition(&MM.HardCircleC, hardOffset2D);
	MM.updateArrow(&MM.HardPlaneNormal, planeMean, 0.4*planeNormal);
	MM.HardCircle.show();
	MM.HardCircleC.show();
	MM.HardPlaneNormal.show();
	updateHardVis3D();
	if(ellipseSuccess)
	{
		MM.setEllipsePoints(&MM.SoftEllipse, softEllipseC, softEllipseR.x(), softEllipseR.y(), softEllipseAngle);
		MM.updatePosition(&MM.SoftEllipseC, softEllipseC);
		MM.SoftEllipse.show();
		MM.SoftEllipseC.show();
	}
	MM.visible = m_showCalib();
	MM.forcePublish();
	MM.publishMarkers();

	// Populate the response packet of the service call
	resp.hardOffsetX = m_hardOffset.x();
	resp.hardOffsetY = m_hardOffset.y();
	resp.hardOffsetZ = m_hardOffset.z();
	resp.hardOffset2DZ = m_hardOffset2DZ;
	resp.hardRadius = m_hardRadius;
	resp.hardField = m_hardField;
	resp.hardErr2D = hardErr2D;
	resp.hardErr3D = hardErr3D;
	resp.planeNormalX = planeNormal.x();
	resp.planeNormalY = planeNormal.y();
	resp.planeNormalZ = planeNormal.z();
	resp.planeTiltAxisAngle = planeTiltAxisAngle;
	resp.planeTiltAngle = planeTiltAngle;

	// Return if the service call was successfully handled
	return ellipseSuccess;
}

// Stop calibration 3D service handler
bool MagFilter::stopCalibration3D(robotcontrol::MagCalib3DRequest& req, robotcontrol::MagCalib3DResponse& resp)
{
	// This calibration method assumes that the robot was rotated in all possible orientations as best possible.

	// Don't do anything if a calibration isn't active
	if(!m_calibrating)
	{
		ROS_INFO("StopCalibration3D: Service call received even though no calibration is active");
		if(m_showCalib())
		{
			ROS_INFO("StopCalibration3D: Republishing data points of last calibration");
			MM.forcePublish();
			MM.publishMarkers();
		}
		resp = robotcontrol::MagCalib3DResponse();
		return true;
	}

	// Indicate to the user that this service call has been received
	ROS_INFO("Stopping magnetometer calibration (3D) with %d data points...", (int)m_calibrationMeasurements.size());

	// Hard iron calibration variables
	Eigen::Vector3d hardOffset3D;
	double hardRadius3D, hardErr3D;

	// 3D hard iron calibration: Fit a sphere to the data
	ConicFit::fitSphere(m_calibrationMeasurements, hardOffset3D, hardRadius3D);
	hardErr3D = ConicFit::fitSphereError(m_calibrationMeasurements, hardOffset3D, hardRadius3D);

	// Update the hard iron calibration parameters on the config server (we have no values to update the 2D circle radius and Z offset, so we just don't touch them)
	m_hard_x.set(hardOffset3D.x());
	m_hard_y.set(hardOffset3D.y());
	m_hard_z.set(hardOffset3D.z());
	m_hard_B.set(hardRadius3D);
	updateHardOffset(true);

	// Report the results of the hard iron calibration
	ROS_INFO("Hard: Offset(%.5lf, %.5lf, %.5lf), Field(%.5lf), Err3D(%.2lg)", m_hardOffset.x(), m_hardOffset.y(), m_hardOffset.z(), m_hardField, hardErr3D);

	// Soft iron calibration variables
	Eigen::Matrix3d softEllipsoidA, softEllipsoidQ, softEllipsoidWinv;
	Eigen::Vector3d softEllipsoidC, softEllipsoidR;
	double fixedSphereR, softEllipsoidErr;
	bool ellipsoidSuccess = true;

	// 3D soft iron calibration: Fit an ellipsoid to the data
	ConicFit::fitEllipsoid(m_calibrationMeasurements, softEllipsoidC, softEllipsoidA);
	ellipsoidSuccess &= ConicFit::ellipsoidMatrixToAxes(softEllipsoidA, softEllipsoidQ, softEllipsoidR);
	fixedSphereR = pow(softEllipsoidR.x()*softEllipsoidR.y()*softEllipsoidR.z(), 1.0/3.0);
	Eigen::Quaterniond softEllipsoidQuat(softEllipsoidQ);
	ellipsoidSuccess &= ConicFit::ellipsoidAxesToTransform(softEllipsoidQ, softEllipsoidR, softEllipsoidWinv);
	softEllipsoidErr = ConicFit::fitEllipsoidError(m_calibrationMeasurements, softEllipsoidC, softEllipsoidWinv);

	// Display a warning if the 3D soft iron calibration failed
	if(!ellipsoidSuccess)
		ROS_WARN("3D soft iron calibration failed!");

	// Update the soft iron calibration parameters on the config server
	if(ellipsoidSuccess)
	{
		m_soft_hx.set(softEllipsoidC.x());
		m_soft_hy.set(softEllipsoidC.y());
		m_soft_hz.set(softEllipsoidC.z());
		updateSoftOffset();
		m_soft_w1.set(softEllipsoidWinv(0, 0));
		m_soft_w2.set(softEllipsoidWinv(0, 1));
		m_soft_w3.set(softEllipsoidWinv(0, 2));
		m_soft_w4.set(softEllipsoidWinv(1, 1));
		m_soft_w5.set(softEllipsoidWinv(1, 2));
		m_soft_w6.set(softEllipsoidWinv(2, 2));
		m_soft_B.set(fixedSphereR);
		updateSoftMatrix();
	}

	// Report the results of the soft iron calibration
	ROS_INFO("Soft: Offset(%.5lf, %.5lf, %.5lf), Field(%.5lf), Err3D(%.2lg)", m_softOffset.x(), m_softOffset.y(), m_softOffset.z(), m_softField, softEllipsoidErr);
	ROS_INFO("               | %8.5lf  %8.5lf  %8.5lf |", m_softMatrix(0,0), m_softMatrix(0,1), m_softMatrix(0,2));
	ROS_INFO("      Matrix = | %8.5lf  %8.5lf  %8.5lf |", m_softMatrix(1,0), m_softMatrix(1,1), m_softMatrix(1,2));
	ROS_INFO("               | %8.5lf  %8.5lf  %8.5lf |", m_softMatrix(2,0), m_softMatrix(2,1), m_softMatrix(2,2));

	// Unset the internal calibrating flag
	m_calibrating = false;

	// Update the list of fixed data points
	updateFixedPoints();

	// Update and publish the required visualisation markers
	MM.hideAll();
	updateHardVis3D();
	if(ellipsoidSuccess)
		updateSoftVis3D();
	MM.visible = m_showCalib();
	MM.forcePublish();
	MM.publishMarkers();

	// Populate the response packet of the service call
	resp.hardOffsetX = m_hardOffset.x();
	resp.hardOffsetY = m_hardOffset.y();
	resp.hardOffsetZ = m_hardOffset.z();
	resp.hardField = m_hardField;
	resp.hardError = hardErr3D;
	resp.softOffsetX = m_softOffset.x();
	resp.softOffsetY = m_softOffset.y();
	resp.softOffsetZ = m_softOffset.z();
	resp.softMatrixW1 = m_softMatrix(0, 0);
	resp.softMatrixW2 = m_softMatrix(0, 1);
	resp.softMatrixW3 = m_softMatrix(0, 2);
	resp.softMatrixW4 = m_softMatrix(1, 1);
	resp.softMatrixW5 = m_softMatrix(1, 2);
	resp.softMatrixW6 = m_softMatrix(2, 2);
	resp.softField = m_softField;
	resp.softError = softEllipsoidErr;

	// Return if the service call was successfully handled
	return ellipsoidSuccess;
}

// Abort the current calibration procedure
bool MagFilter::abortCalibration(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	// Indicate to the user that this service call has been received
	if(m_calibrating)
		ROS_INFO("Aborting magnetometer calibration (no calibration parameters have been touched)...");
	else
		ROS_INFO("Aborting magnetometer calibration (no calibration was actually running)...");
	ROS_INFO("Had %d data points.", (int)m_calibrationMeasurements.size());

	// Unset the internal calibrating flag
	m_calibrating = false;

	// Update the list of fixed data points
	updateFixedPoints();

	// Update and publish the required visualisation markers
	MM.hideAll();
	updateHardVis3D();
	updateSoftVis3D();
	MM.visible = m_showCalib();
	MM.forcePublish();
	MM.publishMarkers();

	// Return that the service call was handled
	return true;
}

// Show the current calibration
bool MagFilter::showCalibration(robotcontrol::MagCalibShowRequest& req, robotcontrol::MagCalibShowResponse& resp)
{
	// Indicate to the user that this service call has been received
	ROS_INFO("Showing the current magnetometer calibration...");
	
	// Don't do anything if a calibration is currently in progress
	if(m_calibrating)
	{
		ROS_WARN("Ignored call to show the current magnetometer calibration because a calibration is currently in progress!");
		return false;
	}
	
	// Ensure we have the latest calibration parameters
	updateHardOffset(false);
	updateSoftOffset();
	updateSoftMatrix();
	
	// Display the current magnetometer calibration
	ROS_INFO("Hard: Offset(%.5lf, %.5lf, %.5lf), Field3D(%.5lf)", m_hardOffset.x(), m_hardOffset.y(), m_hardOffset.z(), m_hardField);
	ROS_INFO("      Offset2DZ(%.5lf), Field2D(%.5lf)", m_hardOffset2DZ, m_hardRadius);
	ROS_INFO("Soft: Offset(%.5lf, %.5lf, %.5lf), Field(%.5lf)", m_softOffset.x(), m_softOffset.y(), m_softOffset.z(), m_softField);
	ROS_INFO("               | %8.5lf  %8.5lf  %8.5lf |", m_softMatrix(0,0), m_softMatrix(0,1), m_softMatrix(0,2));
	ROS_INFO("      Matrix = | %8.5lf  %8.5lf  %8.5lf |", m_softMatrix(1,0), m_softMatrix(1,1), m_softMatrix(1,2));
	ROS_INFO("               | %8.5lf  %8.5lf  %8.5lf |", m_softMatrix(2,0), m_softMatrix(2,1), m_softMatrix(2,2));
	
	// Display what data points on the unit sphere map to by the inverse of the soft iron normalisation transform
	ConicFit::Points3D unitSphere;
	ConicFit::genSphereData(unitSphere, 24, Eigen::Vector3d::Zero(), 1.0);
	MM.clearRawDataPoints();
	MM.clearFixedDataPoints();
	Eigen::Matrix3d W = m_softMatrix.inverse();
	for(size_t i = 0; i < unitSphere.size(); i++)
	{
		MM.addFixedDataPoint(m_softField * unitSphere[i]);
		MM.addRawDataPoint(W*unitSphere[i] + m_softOffset);
	}
	
	// Update and publish the required visualisation markers
	MM.hideAll();
	updateHardVis3D();
	updateSoftVis3D();
	MM.forcePublish();
	MM.publishMarkers();

	// Populate the response packet of the service call
	resp.hardOffsetX = m_hardOffset.x();
	resp.hardOffsetY = m_hardOffset.y();
	resp.hardOffsetZ = m_hardOffset.z();
	resp.hardOffset2DZ = m_hardOffset2DZ;
	resp.hardRadius = m_hardRadius;
	resp.hardField = m_hardField;
	resp.softOffsetX = m_softOffset.x();
	resp.softOffsetY = m_softOffset.y();
	resp.softOffsetZ = m_softOffset.z();
	resp.softMatrixW1 = m_softMatrix(0, 0);
	resp.softMatrixW2 = m_softMatrix(0, 1);
	resp.softMatrixW3 = m_softMatrix(0, 2);
	resp.softMatrixW4 = m_softMatrix(1, 1);
	resp.softMatrixW5 = m_softMatrix(1, 2);
	resp.softMatrixW6 = m_softMatrix(2, 2);
	resp.softField = m_softField;
	
	// Return that the service call was handled
	return true;
}

// Clear the current calibration markers
bool MagFilter::clearCalibrationMarkers(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	// Clear the calibration markers
	clearCalibMarkers();

	// Return that the service call was successful
	return true;
}

// Clear warp points service handler
bool MagFilter::warpClearPoints(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	// Clear the parameter string
	m_warpParamString.set("");
	
	// Return that the service was successfully handled
	return true;
}

// Add warp point service handler (supplied true heading must be in degrees!)
bool MagFilter::warpAddPoint(robotcontrol::WarpAddPointRequest& req, robotcontrol::WarpAddPointResponse& resp)
{
	// Retrieve/calculate the reference headings
	double magHeading = atan2(-m_valuePreWarp.y(), m_valuePreWarp.x()); // In radians!
	double trueHeading = req.trueHeading - 360.0*floor(req.trueHeading / 360.0); // In degrees!
	
	// Convert to the required units
	if(m_warpUseDegrees())
		magHeading *= 180.0 / M_PI;
	else
		trueHeading *= M_PI / 180.0;
	
	// Update the parameter string
	std::string existing = m_warpParamString();
	std::ostringstream ss;
	if(!existing.empty())
		ss << existing << "|";
	ss << trueHeading << " " << magHeading;
	m_warpParamString.set(ss.str());
	
	// Populate the service response struct
	resp.trueHeading = trueHeading;
	resp.magHeading = magHeading;
	resp.newParamString = ss.str();
	
	// Return that the service was successfully handled
	return true;
}

// Handle data from the warp parameter string
void MagFilter::handleWarpParamString()
{
	// The warp parameter string should be a list of pairs "A0 M0|A1 M1|...|An Mn".
	// The units of the parameters can be either radians or degrees depending on the config param m_warpUseDegrees.
	// Ai => Should be the actual true heading on the field (CCW yaw from the +ve goal) when the i-th measurement was taken.
	// Mi => Should be the heading on the field (CCW yaw from the +ve goal) calculated purely from the magnetometer value
	//       of the i-th measurement, using the assumption that the global magnetic field points towards the positive goal.
	// For example, if the magnetometer measurements are perfect and the magnetic field in truth points along the positive
	// y-axis of the field, a parameter string might look like: "0 270|90 0|180 90|270 180"
	
	// Declare variables
	std::vector<double> rawVec, warpedVec;
	double raw, warped;
	char pipe = '|';
	
	// Set radians
	m_warp.setModulus(CyclicWarp::RADIANS);
	
	// Clear the warp parameters if the parameter string is empty
	if(m_warpParamString().empty())
	{
		m_warp.clear();
		return;
	}
	
	// Initialise a string stream with the parameter string
	std::istringstream ss(m_warpParamString());
	
	// Keep reading in data pairs until we run out
	while(!ss.fail() && pipe == '|')
	{
		ss >> raw >> warped;
		if(ss.fail())
		{
			ROS_WARN("Failed to parse the warp parameter string!");
			m_warp.clear();
			return;
		}
		if(m_warpUseDegrees())
		{
			rawVec.push_back(m_warp.wrap(raw * M_PI/180.0));
			warpedVec.push_back(m_warp.wrap(warped * M_PI/180.0));
		}
		else
		{
			rawVec.push_back(m_warp.wrap(raw));
			warpedVec.push_back(m_warp.wrap(warped));
		}
		pipe = '\0';
		ss >> pipe;
	}
	
	// Catch case where we didn't process everything in the string
	if(!ss.fail())
	{
		ROS_WARN("Failed to parse the complete warp parameter string!");
		m_warp.clear();
		return;
	}
	
	// Failure if the two vectors are not of the same size
	size_t N = rawVec.size();
	if(warpedVec.size() != N)
	{
		ROS_WARN("Unequal number of raw and warped parameters in the warp parameter string!");
		m_warp.clear();
		return;
	}
	
	// Sort the reference pairs by their raw values
	bool done = false;
	size_t j = 0;
	while(!done)
	{
		j++;
		done = true;
		for(size_t i = 0; i < N - j; i++)
		{
			if(rawVec[i] > rawVec[i+1])
			{
				std::swap(rawVec[i], rawVec[i+1]);
				std::swap(warpedVec[i], warpedVec[i+1]);
				done = false;
			}
		}
	}
	
	// Try to set the reference values
	if(!m_warp.setRefValues(rawVec, warpedVec))
	{
		ROS_WARN("Warp parameter string was parsed, but invalid parameters resulted!");
		m_warp.clear();
	}
}

// Helper function to convert a double to a string with a given precision
std::string dblToStringPercent(double value, int precision)
{
	// Use a stringstream
	std::ostringstream ss;
	ss << std::fixed << std::setprecision(precision) << 100.0*value << "%" << std::endl;
	return ss.str();
}

// Update whether auto-calibration is enabled (callback)
void MagFilter::updateAutoCalibEnable()
{
	// Initialise/reset variables
	m_autoCalibClearDataBuf = true;
	m_autoCalibLastUpdate.fromNSec(0);
	m_autoCalibFilterTarget.setZero();
	m_autoCalibFilterTargetConf = 0.0;

	// Update the filter parameters (settling time and max delta)
	handleAutoCalibFilterXY();
	handleAutoCalibFilterZ();
	handleAutoCalibFilterR();
}

// Compute the current automatic magnetometer calibration
bool MagFilter::computeAutoCalib()
{
	// Zero the auto-calibration trigger counter and remember the current time
	if(m_autoCalibBufCount >= m_autoCalibCountForUpdate())
		m_autoCalibBufCount = 0;
	m_autoCalibLastUpdate = ros::Time::now();

	// Don't do anything if we don't have enough data points yet to fill the circular buffer
	if(!m_autoCalibBuf.full())
		return false;

	// Reset the auto-calibration trigger counter
	m_autoCalibBufCount = 0;

	// Plot an event that we are attempting to perform an auto-calibration update
	PM.plotEvent("Mag Auto Calib");

	// Manage buffer sizes
	size_t N = m_autoCalibBuf.size();
	m_autoCalibXYW.resize(N);

	// Populate the buffer required for circle fitting and calculate the weighted means and weight sums
	double meanX = 0.0, meanY = 0.0, meanZ = 0.0, sumW = 0.0, sumWW = 0.0;
	for(size_t i = 0; i < N; i++)
	{
		const Eigen::Vector4d& xyzw = m_autoCalibBuf[i];
		double w = fabs(xyzw.w());
		m_autoCalibXYW[i] << xyzw.x(), xyzw.y(), w;
		meanX += w*xyzw.x();
		meanY += w*xyzw.y();
		meanZ += w*xyzw.z();
		sumW += w;
		sumWW += w*w;
	}
	if(sumW <= 0.0)
	{
		ROS_WARN("Magnetometer auto-calibration failed as all weights are zero!");
		return false;
	}
	meanX /= sumW;
	meanY /= sumW;
	meanZ /= sumW;
	Eigen::Vector2d meanXY(meanX, meanY);

	// Calculate the weighted covariance matrix terms of the 2D XY data
	double sumXX = 0.0, sumXY = 0.0, sumYY = 0.0;
	double covDenom = sumW*sumW - sumWW;
	if(covDenom <= 0.0)
	{
		ROS_WARN("Magnetometer auto-calibration failed as the covariance denominator is non-positive!");
		return false;
	}
	for(size_t i = 0; i < N; i++)
	{
		const Eigen::Vector3d& xyw = m_autoCalibXYW[i];
		double Xm = xyw.x() - meanX;
		double Ym = xyw.y() - meanY;
		sumXX += xyw.z()*Xm*Xm;
		sumXY += xyw.z()*Xm*Ym;
		sumYY += xyw.z()*Ym*Ym;
	}
	double covScale = sumW / covDenom;
	sumXX *= covScale;
	sumXY *= covScale;
	sumYY *= covScale;

	// Calculate the planar spread of the data (proportional to the equivalent radius of the covariance ellipse)
	double covDet = coerceMin(sumXX*sumYY - sumXY*sumXY, 0.0); // Mathematically this must be non-negative (assuming non-negative weights), so we should only be trimming off at worst a few eps...
	double planarSpread = std::pow(covDet, 0.25);

	// Fit a circle in a weighted manner to the data points
	double fittedRadius = 0.0;
	Eigen::Vector2d fittedCentre(0.0, 0.0);
	ConicFit::fitCircleWeighted(m_autoCalibXYW, fittedCentre, fittedRadius); // Never fails to produce some kind of a fit...
	double fittingError = ConicFit::fitCircleErrorWeighted(m_autoCalibXYW, fittedCentre, fittedRadius);

	// Calculate the angular spread of the data around the fitted centre
	double angleMeanX = 0.0, angleMeanY = 0.0;
	for(size_t i = 0; i < N; i++)
	{
		const Eigen::Vector3d& xyw = m_autoCalibXYW[i];
		double Xc = xyw.x() - fittedCentre.x();
		double Yc = xyw.y() - fittedCentre.y();
		double norm = sqrt(Xc*Xc + Yc*Yc);
		if(norm <= 0.0)
			norm = 1.0;
		angleMeanX += xyw.z()*Xc/norm;
		angleMeanY += xyw.z()*Yc/norm;
	}
	angleMeanX /= sumW;
	angleMeanY /= sumW;
	double angleSpread = coerce(1.0 - sqrt(angleMeanX*angleMeanX + angleMeanY*angleMeanY), 0.0, 1.0); // 0.0 => Completely biased in one direction, 1.0 => Completely even angular spread about the fitted centre
	static const double c = 2.348923291965; // This is = 2*pi - sqrt(4*pi^2 - 24)
	double spreadAngle = (angleSpread <= c*c/24.0 ? sqrt(24.0*angleSpread) : 0.5*c + 12.0*angleSpread/c); // Take a circular arc that subtends an angle of theta at the circle centre, and compute using integration the angle spread as a function of theta... This expression is an approximate inverse to this function.

	// Retrieve the current auto-calibrated parameters
	Eigen::Vector2d autoXY(m_autoCalibFilterX.value(), m_autoCalibFilterY.value());
	double autoR = m_autoCalibFilterR.value();

	// Sanity check the current auto-calibrated radius
	if(autoR <= 0.0)
	{
		double newR = m_hard_R();
		if(newR <= 0.0)
		{
			newR = m_hard_B();
			if(newR <= 0.0)
				newR = 1.0;
		}
		m_autoCalibFilterR.setValue(newR);
		autoR = m_autoCalibFilterR.value();
		ROS_WARN("The auto-calibrated magnetometer radius was non-positive => Resetting to %.3f!", autoR);
	}

	// Calculate the fade target based on shifting towards the weighted mean
	Eigen::Vector2d meanToAuto = autoXY - meanXY;
	double meanToAutoDist = meanToAuto.norm();
	Eigen::Vector2d meanTarget = (meanToAutoDist <= 0.0 ? meanXY : meanXY + meanToAuto*(autoR/meanToAutoDist));

	// Calculate the confidence we have in the mean target
	double meanToAutoDistFactor = interpolateCoerced<double>(m_autoCalibMeanDistSmall(), m_autoCalibMeanDistLarge(), 0.0, 1.0, meanToAutoDist/autoR);
	double planarSpreadFactor = interpolateCoerced<double>(m_autoCalibPlanarSpreadSmall(), m_autoCalibPlanarSpreadLarge(), 1.0, 0.0, planarSpread/autoR);
	double meanTargetConf = coerce(meanToAutoDistFactor * planarSpreadFactor, 0.0, 1.0);

	// Calculate the confidence we have in the fitted target
	double normedRErr = (fittedRadius - autoR) / (m_autoCalibRadiusErrLimit() * autoR);
	double normedRFactor = 1.0/(1.0 + 9.0*normedRErr*normedRErr); // This bell-shaped factor is 1 at a fitted radius of autoR, and 0.1 at a fitted radius of autoR +- ErrLimit*autoR
	if(normedRFactor < 0.1) normedRFactor = 0.0;
	double fittedRadiusFactor = coerce(1.0 - normedRErr*normedRErr, 0.0, 1.0);
	double spreadAngleReq = coerceMin<double>(interpolate<double>(m_autoCalibSpreadErrorMin(), m_autoCalibSpreadErrorMax(), m_autoCalibSpreadAngleMin(), M_2PI, fittingError), m_autoCalibSpreadAngleMin()); // Spread angle for confidence factor 0.5
	double spreadAngleLow = spreadAngleReq*(1.0 - m_autoCalibSpreadAngleTol());  // Spread angle for confidence factor 0.0
	double spreadAngleHigh = spreadAngleReq*(1.0 + m_autoCalibSpreadAngleTol()); // Spread angle for confidence factor 1.0
	double spreadAngleFactor = interpolateCoerced(spreadAngleLow, spreadAngleHigh, 0.0, 1.0, spreadAngle);
	double fittedTargetConf = coerce(fittedRadiusFactor * spreadAngleFactor * (1.0 + m_autoCalibPreferRadius()*(fittedRadiusFactor - 0.5)) / m_autoCalibReqFittedConf(), 0.0, 1.0);

	// Interpolate between the two targets based on the individual target confidences
	double confSum = fittedTargetConf + meanTargetConf;
	double targetU = (confSum > 0.0 ? fittedTargetConf*(1.0 + meanTargetConf)/confSum : 0.0);
	double targetV = 1.0 - targetU;
	Eigen::Vector2d target = targetU*fittedCentre + targetV*meanTarget;
	double targetConf = coerce(targetU*fittedTargetConf + targetV*meanTargetConf, 0.0, 1.0);
	double targetRadius = coerceMin(targetU*fittedRadius + targetV*autoR, 0.0);

	// Show debug messages if desired
	if(m_autoCalibShowDebug() || m_autoCalibTest)
	{
		ROS_INFO("Current auto-calibration: autoXY(%.3f, %.3f), autoR(%.3f)", autoXY.x(), autoXY.y(), autoR);
		ROS_INFO("Weighted mean of data: (%.3f, %.3f, %.3f)", meanX, meanY, meanZ);
		ROS_INFO("Planar spread of data (standard deviation): %.3f", planarSpread);
		ROS_WARN("Fitted circle: Centre(%.3f, %.3f), Radius(%.3f vs %.3f), Error(%.3f)", fittedCentre.x(), fittedCentre.y(), fittedRadius, autoR, fittingError);
		ROS_INFO("Degrees: Spread angle(%.1f), Spread angle range(%.1f->%.1f) => Factor(%.3f)", spreadAngle*180.0/M_PI, spreadAngleLow*180.0/M_PI, spreadAngleHigh*180.0/M_PI, spreadAngleFactor);
		ROS_INFO("Normalised current calib to weighted mean distance: %.3f", meanToAutoDist/autoR);
		ROS_WARN("meanToAutoDistFactor(%.3f) + planarSpreadFactor(%.3f) => Mean conf(%.1f%%)", meanToAutoDistFactor, planarSpreadFactor, 100.0*meanTargetConf);
		ROS_WARN("fittedRadiusFactor(%.3f) + spreadAngleFactor(%.3f) => Fitted conf(%.1f%%)", fittedRadiusFactor, spreadAngleFactor, 100.0*fittedTargetConf);
		ROS_INFO("Using interpolation: Mean target(%.0f%%) + Fitted target(%.0f%%)", 100.0*targetV, 100.0*targetU);
		ROS_WARN("Final target: Centre(%.3f, %.3f, %.3f), Radius(%.3f), Conf(%.1f%%)", target.x(), target.y(), meanZ, targetRadius, 100.0*targetConf);
		ROS_INFO("----------------------------------------");
	}

	// Sanity check that nothing has exploded in our face
	if(!std::isfinite(target.x()) || !std::isfinite(target.y()) || !std::isfinite(targetConf))
	{
		ROS_WARN("A non-finite magnetometer auto-calibration target and/or confidence was calculated => Not updating filters...");
		return false;
	}

	// Update the auto-calib filter targets
	m_autoCalibFilterTarget << target.x(), target.y(), meanZ, targetRadius;
	m_autoCalibFilterTargetConf = targetConf;

	// Update the filter coefficients (uses m_autoCalibFilterTargetConf)
	handleAutoCalibFilterXY();
	handleAutoCalibFilterZ();
	handleAutoCalibFilterR();

	// Update visualisation markers (calling publishMarkers() whenever appropriate is the duty of the caller of this function)
	Eigen::Vector3d mean3D(meanX, meanY, meanZ);
	Eigen::Vector3d meanTarget3D(meanTarget.x(), meanTarget.y(), meanZ);
	Eigen::Vector3d fittedTarget3D(fittedCentre.x(), fittedCentre.y(), meanZ);
	Eigen::Vector3d target3D = m_autoCalibFilterTarget.head<3>();
	bool show = m_showAutoCalib();
	MM.setCirclePoints(&MM.AutoDataVar, mean3D, planarSpread);
	MM.updatePosition(&MM.AutoDataMean, mean3D);
	MM.updatePosition(&MM.AutoDataText, mean3D.x(), mean3D.y(), mean3D.z() + MM_TEXT_OFFSET_ZN);
	MM.setCirclePoints(&MM.AutoMeanCircle, meanTarget3D, autoR);
	MM.updatePosition(&MM.AutoMeanCircleC, meanTarget3D);
	MM.updatePosition(&MM.AutoMeanText, meanTarget3D.x(), meanTarget3D.y(), meanTarget3D.z() + MM_TEXT_OFFSET_ZP);
	MM.AutoMeanText.setText("Mean Target " + dblToStringPercent(meanTargetConf, 1));
	MM.setCirclePoints(&MM.AutoFittedCircle, fittedTarget3D, fittedRadius);
	MM.updatePosition(&MM.AutoFittedCircleC, fittedTarget3D);
	MM.updatePosition(&MM.AutoFittedText, fittedTarget3D.x(), fittedTarget3D.y(), fittedTarget3D.z() + MM_TEXT_OFFSET_ZP);
	MM.AutoFittedText.setText("Fitted Target " + dblToStringPercent(fittedTargetConf, 1));
	MM.setCirclePoints(&MM.AutoTargetCircle, target3D, targetRadius);
	MM.updatePosition(&MM.AutoTargetCircleC, target3D);
	MM.updatePosition(&MM.AutoTargetText, target3D.x(), target3D.y(), target3D.z() + MM_TEXT_OFFSET_ZN);
	MM.AutoTargetText.setText("Final Target " + dblToStringPercent(targetConf, 1));
	MM.AutoDataVar.setVisible(show);
	MM.AutoDataMean.setVisible(show);
	MM.AutoDataText.setVisible(show);
	MM.AutoMeanCircle.setVisible(show);
	MM.AutoMeanCircleC.setVisible(show);
	MM.AutoMeanText.setVisible(show);
	MM.AutoFittedCircle.setVisible(show);
	MM.AutoFittedCircleC.setVisible(show);
	MM.AutoFittedText.setVisible(show);
	MM.AutoTargetCircle.setVisible(show);
	MM.AutoTargetCircleC.setVisible(show);
	MM.AutoTargetText.setVisible(show);

	// Return that a calibration was performed
	return true;
}

// Callback to update the enabled state of the individual filters
void MagFilter::updateAutoCalibFilterEnable()
{
	// Update the enabled state of the individual auto-calibration filters
	m_autoCalibFilterX.setFrozen(!m_autoCalibEnableXY());
	m_autoCalibFilterY.setFrozen(!m_autoCalibEnableXY());
	m_autoCalibFilterZ.setFrozen(!m_autoCalibEnableZ());
	m_autoCalibFilterR.setFrozen(!m_autoCalibEnableR());
}

// Callback to update the X and Y auto-calib filters
void MagFilter::handleAutoCalibFilterXY()
{
	// Update the X and Y auto-calib filter parameters
	double scaler = getAutoCalibFilterScaler();
	m_autoCalibFilterX.setParams(m_autoCalibFilterTsXY() / scaler, m_autoCalibFilterDeltaXY() * scaler); // Division by zero is ok for Ts!
	m_autoCalibFilterY.setParams(m_autoCalibFilterTsXY() / scaler, m_autoCalibFilterDeltaXY() * scaler); // Division by zero is ok for Ts!
}

// Callback to update the Z auto-calib filter
void MagFilter::handleAutoCalibFilterZ()
{
	// Update the Z auto-calib filter parameters
	double scaler = getAutoCalibFilterScaler();
	m_autoCalibFilterZ.setParams(m_autoCalibFilterTsZ() / scaler, m_autoCalibFilterDeltaZ() * scaler); // Division by zero is ok for Ts!
}

// Callback to update the R auto-calib filter
void MagFilter::handleAutoCalibFilterR()
{
	// Update the R auto-calib filter parameters
	double scaler = getAutoCalibFilterScaler();
	m_autoCalibFilterR.setParams(m_autoCalibFilterTsR() / scaler, m_autoCalibFilterDeltaR() * scaler); // Division by zero is ok for Ts!
}

// Helper function for random data generation
double plusMinus(double value)
{
	// Return a random number in the range [-value,value)
	return value*(2.0*drand48() - 1.0);
}

// Service for testing/tuning of the magnetometer auto-calibration
bool MagFilter::handleTestAutoCalib(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	// Generate random parameters for the data generation
	Eigen::Vector4d centre(m_hardOffset.x() + plusMinus(1.0), m_hardOffset.y() + plusMinus(1.0), m_hardOffset2DZ + plusMinus(0.2), 0.0);
	double spread = plusMinus(1.3);
	double radius = m_hardRadius*(1.0 + plusMinus(0.5));
	double noiseXY = plusMinus(0.3)*spread;
	double noiseZ = plusMinus(0.2)*spread;
	double angleA = plusMinus(M_PI);
	double angleB = angleA + plusMinus(M_2PI)*spread;

	// Indicate what is being done
	ROS_INFO("Generated fake auto calibration data:");
	ROS_INFO("Centre(%.3f, %.3f, %.3f), Radius(%.3f), Angles(%.3f, %.3f)", centre.x(), centre.y(), centre.z(), radius, angleA, angleB);
	ROS_INFO("Spread(%.3f), noiseXY(%.3f), noiseZ(%.3f)", spread, noiseXY, noiseZ);
	ROS_INFO("----------------------------------------");

	// Populate the buffer with the required random data
	size_t N = m_autoCalibBuf.capacity();
	m_autoCalibBuf.resize(N);
	for(size_t i = 0; i < N; i++)
	{
		Eigen::Vector4d point = centre;
		double angle = interpolate(0.0, 1.0, angleA, angleB, drand48());
		point.x() += radius*cos(angle) + plusMinus(noiseXY);
		point.y() += radius*sin(angle) + plusMinus(noiseXY);
		point.z() += plusMinus(noiseZ);
		double normedZ = (point.z() - m_hardOffset2DZ) / 0.4;
		point.w() = 1.0 - normedZ*normedZ;
		m_autoCalibBuf[i] = point;
	}

	// Ensure an auto-calibration update will be performed
	m_autoCalibBufCount = m_autoCalibCountForUpdate();
	m_autoCalibTest = true;

	// Return that the service was successfully handled
	return true;
}

// Callback to update the hard iron offset value
void MagFilter::updateHardOffset(bool resetAutoCalib)
{
	// Update the hard offset variables from the config server parameters
	m_hardOffset << m_hard_x(), m_hard_y(), m_hard_z();
	m_hardOffset2DZ = m_hard_zR();
	m_hardRadius = m_hard_R();
	m_hardField = m_hard_B();

	// Handle requests to reset the filter values
	if(m_autoCalibResetToHard())
	{
		m_autoCalibResetToHard.set(false);
		resetAutoCalib = true;
	}

	// Update the auto-calibration values
	if(resetAutoCalib)
	{
		m_autoCalibFilterX.setValue(m_hardOffset.x());
		m_autoCalibFilterY.setValue(m_hardOffset.y());
		m_autoCalibFilterZ.setValue(m_hardOffset2DZ);
		m_autoCalibFilterR.setValue(m_hardRadius);
	}
}

// Callback to set the hard iron offset from the current auto-calibrated value
void MagFilter::setHardIronFromAutoCalib()
{
	// Reset the set from auto calib config variable if it is true
	if(m_hardSetFromAutoCalib())
		m_hardSetFromAutoCalib.set(false);

	// Don't do anything if auto-calibration isn't currently enabled
	if(!m_autoCalibEnable()) return;

	// Set the hard iron config server parameters based on the currently auto-calibrated values (we have no information to be able to update m_hard_B)
	m_hard_x.set(m_autoCalibFilterX.value()); 
	m_hard_y.set(m_autoCalibFilterY.value());
	m_hard_z.set(m_autoCalibFilterZ.value() + m_hardOffset.z() - m_hardOffset2DZ);
	m_hard_R.set(m_autoCalibFilterR.value());
	m_hard_zR.set(m_autoCalibFilterZ.value());
	updateHardOffset(false);
}

// Callback for the show calib config parameter
void MagFilter::handleShowCalib()
{
	// If there is nothing to display then generate data for the current calibration
	if(m_showCalib() && MM.RawData.marker.points.empty() && (MM.AutoData.marker.points.empty() || !m_showAutoCalib()))
	{
		robotcontrol::MagCalibShow srv;
		showCalibration(srv.request, srv.response);
	}

	// Update the markers
	MM.visible = m_showCalib();
	MM.forcePublish();
	MM.publishMarkers();
}

// Callback for the show auto calibration parameter
void MagFilter::handleShowAutoCalib()
{
	// Show/hide stuff as appropriate
	if(m_showAutoCalib())
	{
		MM.hideAuto = false;
		MM.AutoDataVar.show();
		MM.AutoDataMean.show();
		MM.AutoDataText.show();
		MM.AutoMeanCircle.show();
		MM.AutoMeanCircleC.show();
		MM.AutoMeanText.show();
		MM.AutoFittedCircle.show();
		MM.AutoFittedCircleC.show();
		MM.AutoFittedText.show();
		MM.AutoTargetCircle.show();
		MM.AutoTargetCircleC.show();
		MM.AutoTargetText.show();
		MM.AutoCurCircle.show();
		MM.AutoCurCircleC.show();
		MM.AutoCurText.show();
	}
	else
	{
		MM.hideAuto = true;
		MM.AutoDataVar.hide();
		MM.AutoDataMean.hide();
		MM.AutoDataText.hide();
		MM.AutoMeanCircle.hide();
		MM.AutoMeanCircleC.hide();
		MM.AutoMeanText.hide();
		MM.AutoFittedCircle.hide();
		MM.AutoFittedCircleC.hide();
		MM.AutoFittedText.hide();
		MM.AutoTargetCircle.hide();
		MM.AutoTargetCircleC.hide();
		MM.AutoTargetText.hide();
		MM.AutoCurCircle.hide();
		MM.AutoCurCircleC.hide();
		MM.AutoCurText.hide();
	}
}

// Clear the calib markers
void MagFilter::clearCalibMarkers()
{
	// Clear the config parameter
	m_clearCalibMarkers.set(false);

	// Clear the visualisation markers
	MM.reset();
	MM.forcePublish();
	MM.publishMarkers();
}

// Update the list of fixed points in the marker manager
void MagFilter::updateFixedPoints()
{
	// Clear the current fixed points
	MM.clearFixedDataPoints();
	
	// Correct the calibration measurements and add them to the fixed point list
	for(size_t i = 0; i < m_calibrationMeasurements.size(); i++)
		MM.addFixedDataPoint(m_softField * m_softMatrix * (m_calibrationMeasurements[i] - m_softOffset));
}

// Update the visualisation of the current hard iron calibration
void MagFilter::updateHardVis3D()
{
	// Update the required markers
	MM.updatePosition(&MM.HardSphere, m_hardOffset);
	MM.updateScale(&MM.HardSphere, 2.0*m_hardField);
	MM.updatePosition(&MM.HardSphereC, m_hardOffset);
	
	// Show the required markers
	MM.HardSphere.show();
	MM.HardSphereC.show();
}

// Update the visualisation of the current soft iron calibration
void MagFilter::updateSoftVis3D()
{
	// Declare variables
	Eigen::Matrix3d softEllipsoidA, softEllipsoidQ;
	Eigen::Vector3d softEllipsoidR;
	Eigen::Quaterniond softEllipsoidQuat;
	
	// Calculate the orientation and radii of the ellipse given by the calibration
	softEllipsoidA = m_softMatrix * m_softMatrix;
	if(!ConicFit::ellipsoidMatrixToAxes(softEllipsoidA, softEllipsoidQ, softEllipsoidR))
	{
		ROS_WARN("Tried to update visualisation of soft-iron calibration, but the soft matrix is invalid!");
		return;
	}
	softEllipsoidQuat = softEllipsoidQ;
	
	// Update the required markers
	MM.updatePosition(&MM.SoftEllipsoid, m_softOffset);
	MM.updateOrientation(&MM.SoftEllipsoid, softEllipsoidQuat);
	MM.updateScale(&MM.SoftEllipsoid, 2.0*softEllipsoidR.x(), 2.0*softEllipsoidR.y(), 2.0*softEllipsoidR.z());
	MM.updatePosition(&MM.SoftEllipsoidC, m_softOffset);
	MM.updatePosition(&MM.SoftFixedSphere, 0.0, 0.0, 0.0);
	MM.updateScale(&MM.SoftFixedSphere, 2.0*m_softField);
	MM.updatePosition(&MM.SoftFixedSphereC, 0.0, 0.0, 0.0);
	
	// Show the required markers
	MM.SoftEllipsoid.show();
	MM.SoftEllipsoidC.show();
	MM.SoftFixedSphere.show();
	MM.SoftFixedSphereC.show();
}

// Configure the plot manager
void MagFilter::configurePlotManager()
{
	// Configure the plotted variables
	PM.setName(PM_MAG_INPUT_X, "magInput/x");
	PM.setName(PM_MAG_INPUT_Y, "magInput/y");
	PM.setName(PM_MAG_INPUT_Z, "magInput/z");
	PM.setName(PM_MAG_OUTPUT_X, "magOutput/x");
	PM.setName(PM_MAG_OUTPUT_Y, "magOutput/y");
	PM.setName(PM_MAG_OUTPUT_Z, "magOutput/z");
	PM.setName(PM_MAG_DEB_X, "magDebounced/x");
	PM.setName(PM_MAG_DEB_Y, "magDebounced/y");
	PM.setName(PM_MAG_DEB_Z, "magDebounced/z");
	PM.setName(PM_MAG_DEB_W, "magDebounced/weight");
	PM.setName(PM_AUTO_COUNT, "autoCalib/updateCount");
	PM.setName(PM_AUTO_TIME, "autoCalib/updateTime");
	PM.setName(PM_HARD_X, "hard/x");
	PM.setName(PM_HARD_Y, "hard/y");
	PM.setName(PM_HARD_Z, "hard/z");
	PM.setName(PM_HARD_R, "hard/R");
	PM.setName(PM_HARD_ZR, "hard/zR");
	PM.setName(PM_HARDTGT_X, "hardAutoTarget/x");
	PM.setName(PM_HARDTGT_Y, "hardAutoTarget/y");
	PM.setName(PM_HARDTGT_Z, "hardAutoTarget/z");
	PM.setName(PM_HARDTGT_R, "hardAutoTarget/R");
	PM.setName(PM_HARDTGT_ZR, "hardAutoTarget/zR");

	// Check that we have been thorough
	if(!PM.checkNames())
		ROS_ERROR("Please review any warnings above that are related to the naming of plotter variables!");
}

//
// MagFilter::MarkerMan class
//

// Constructor
MagFilter::MarkerMan::MarkerMan(const std::string& topicName, int publishInterval, bool enabled, const std::string& refFrame, const std::string& markerNs)
 : MarkerManager(topicName, publishInterval, enabled)
 , visible(false)
 , hideAuto(true)
 , RawData(this, refFrame, markerNs + "raw")
 , AutoData(this, refFrame, markerNs + "auto")
 , FixedData(this, refFrame, markerNs + "fixed")
 , HardCircle(this, refFrame, markerNs + "hard")
 , HardCircleC(this, refFrame, MM_CENTRE_SIZE, markerNs + "hard")
 , HardSphere(this, refFrame, 1.0, markerNs + "hard")
 , HardSphereC(this, refFrame, MM_CENTRE_SIZE, markerNs + "hard")
 , HardPlaneNormal(this, refFrame, MM_ARROW_SHAFT_DIAM, MM_ARROW_HEAD_DIAM, MM_ARROW_HEAD_LEN, markerNs + "hard")
 , SoftEllipse(this, refFrame, markerNs + "soft")
 , SoftEllipseC(this, refFrame, MM_CENTRE_SIZE, markerNs + "soft")
 , SoftEllipsoid(this, refFrame, 1.0, markerNs + "soft")
 , SoftEllipsoidC(this, refFrame, MM_CENTRE_SIZE, markerNs + "soft")
 , SoftFixedSphere(this, refFrame, 1.0, markerNs + "softFixed")
 , SoftFixedSphereC(this, refFrame, MM_CENTRE_SIZE, markerNs + "softFixed")
 , AutoDataVar(this, refFrame, markerNs + "autoDebug")
 , AutoDataMean(this, refFrame, MM_CENTRE_SIZE, markerNs + "autoDebug")
 , AutoDataText(this, refFrame, MM_TEXT_SIZE, markerNs + "autoDebug")
 , AutoMeanCircle(this, refFrame, markerNs + "autoDebug")
 , AutoMeanCircleC(this, refFrame, MM_CENTRE_SIZE, markerNs + "autoDebug")
 , AutoMeanText(this, refFrame, MM_TEXT_SIZE, markerNs + "autoDebug")
 , AutoFittedCircle(this, refFrame, markerNs + "autoDebug")
 , AutoFittedCircleC(this, refFrame, MM_CENTRE_SIZE, markerNs + "autoDebug")
 , AutoFittedText(this, refFrame, MM_TEXT_SIZE, markerNs + "autoDebug")
 , AutoTargetCircle(this, refFrame, markerNs + "auto")
 , AutoTargetCircleC(this, refFrame, MM_CENTRE_SIZE, markerNs + "auto")
 , AutoTargetText(this, refFrame, MM_TEXT_SIZE, markerNs + "auto")
 , AutoCurCircle(this, refFrame, markerNs + "auto")
 , AutoCurCircleC(this, refFrame, MM_CENTRE_SIZE, markerNs + "auto")
 , AutoCurText(this, refFrame, MM_TEXT_SIZE, markerNs + "auto")
{
	// Configure the markers as required
	RawData.setType(visualization_msgs::Marker::POINTS);
	RawData.setScale(MM_POINT_SIZE);
	RawData.setColor(0.5, 0.0, 1.0);
	AutoData.setType(visualization_msgs::Marker::POINTS);
	AutoData.setScale(MM_POINT_SIZE);
	AutoData.setColor(1.0, 0.0, 0.0);
	FixedData.setType(visualization_msgs::Marker::POINTS);
	FixedData.setScale(MM_POINT_SIZE);
	FixedData.setColor(0.0, 0.8, 0.8);
	HardCircle.setType(visualization_msgs::Marker::LINE_STRIP);
	HardCircle.setScale(MM_LINE_SIZE);
	HardCircle.setColor(0.8, 0.3, 0.5);
	HardCircleC.setScale(MM_CENTRE_SIZE);
	HardCircleC.setColor(0.8, 0.0, 0.0);
	HardSphere.setColor(0.8, 0.0, 0.0, 0.3);
	HardSphereC.setScale(MM_CENTRE_SIZE);
	HardSphereC.setColor(0.8, 0.5, 0.0);
	HardPlaneNormal.setColor(1.0, 0.0, 0.0);
	SoftEllipse.setType(visualization_msgs::Marker::LINE_STRIP);
	SoftEllipse.setScale(MM_LINE_SIZE);
	SoftEllipse.setColor(0.0, 0.8, 0.7);
	SoftEllipseC.setScale(MM_CENTRE_SIZE);
	SoftEllipseC.setColor(0.0, 0.0, 0.8);
	SoftEllipsoid.setColor(1.0, 0.4, 1.0, 0.3);
	SoftEllipsoidC.setScale(MM_CENTRE_SIZE);
	SoftEllipsoidC.setColor(0.3, 0.3, 0.7);
	SoftFixedSphere.setColor(0.0, 0.0, 0.8, 0.3);
	SoftFixedSphereC.setScale(MM_CENTRE_SIZE);
	SoftFixedSphereC.setColor(0.3, 0.7, 0.3);
	AutoDataVar.setType(visualization_msgs::Marker::LINE_STRIP);
	AutoDataVar.setScale(MM_LINE_SIZE);
	AutoDataVar.setColor(0.60, 0.24, 0.20);                            // Brown => Data mean and standard deviation circle
	AutoDataMean.setScale(MM_CENTRE_SIZE);
	AutoDataMean.setColor(0.60, 0.24, 0.20);
	AutoDataText.setColor(0.60, 0.24, 0.20);
	AutoDataText.setText("Data Spread");
	AutoMeanCircle.setType(visualization_msgs::Marker::LINE_STRIP);
	AutoMeanCircle.setScale(MM_LINE_SIZE);
	AutoMeanCircle.setColor(0.50, 0.50, 0.00);                         // Khaki => Mean target circle
	AutoMeanCircleC.setScale(MM_CENTRE_SIZE);
	AutoMeanCircleC.setColor(0.51, 0.50, 0.07);
	AutoMeanText.setColor(0.51, 0.50, 0.07);
	AutoMeanText.setText("Mean Target");
	AutoFittedCircle.setType(visualization_msgs::Marker::LINE_STRIP);
	AutoFittedCircle.setScale(MM_LINE_SIZE);
	AutoFittedCircle.setColor(0.0, 0.2, 0.8);                          // Blue => Fitted target circle
	AutoFittedCircleC.setScale(MM_CENTRE_SIZE);
	AutoFittedCircleC.setColor(0.0, 0.2, 0.8);
	AutoFittedText.setColor(0.0, 0.2, 0.8);
	AutoFittedText.setText("Fitted Target");
	AutoTargetCircle.setType(visualization_msgs::Marker::LINE_STRIP);
	AutoTargetCircle.setScale(MM_LINE_SIZE);
	AutoTargetCircle.setColor(0.53, 0.31, 0.46);                       // Violet => Total target circle
	AutoTargetCircleC.setScale(MM_CENTRE_SIZE);
	AutoTargetCircleC.setColor(0.53, 0.31, 0.46);
	AutoTargetText.setColor(0.53, 0.31, 0.46);
	AutoTargetText.setText("Final Target");
	AutoCurCircle.setType(visualization_msgs::Marker::LINE_STRIP);
	AutoCurCircle.setScale(MM_LINE_SIZE);
	AutoCurCircle.setColor(0.75, 0.63, 0.00);                          // Gold => Current 2D calibration
	AutoCurCircleC.setScale(MM_CENTRE_SIZE);
	AutoCurCircleC.setColor(0.75, 0.63, 0.00);
	AutoCurText.setColor(0.75, 0.63, 0.00);
	AutoCurText.setText("Current");

	// Reset the state of the markers
	reset();
}

// Add a point to a marker
void MagFilter::MarkerMan::addDataPoint(vis_utils::GenMarker* marker, double x, double y, double z)
{
	// Add the required point to the required marker
	geometry_msgs::Point p;
	p.x = MM_SCALE * x;
	p.y = MM_SCALE * y;
	p.z = MM_SCALE * z + MM_OFFSET_Z;
	marker->marker.points.push_back(p);
}

// Set a point in a marker (the index is assumed to be in range)
void MagFilter::MarkerMan::setDataPoint(vis_utils::GenMarker* marker, size_t index, double x, double y, double z)
{
	// Add the required point to the required marker
	geometry_msgs::Point& p = marker->marker.points[index];
	p.x = MM_SCALE * x;
	p.y = MM_SCALE * y;
	p.z = MM_SCALE * z + MM_OFFSET_Z;
}

// Update an arrow marker
void MagFilter::MarkerMan::updateArrow(vis_utils::ArrowMarker* marker, const Eigen::Vector3d& from, const Eigen::Vector3d& vec)
{
	// Update the required arrow marker
	Eigen::Vector3d fromPt(MM_SCALE*from.x(), MM_SCALE*from.y(), MM_SCALE*from.z() + MM_OFFSET_Z);
	Eigen::Vector3d toPt = fromPt + MM_SCALE*vec;
	marker->setPoint(0, fromPt.x(), fromPt.y(), fromPt.z());
	marker->setPoint(1, toPt.x(), toPt.y(), toPt.z());
}

// Update the position of a marker
void MagFilter::MarkerMan::updatePosition(vis_utils::GenMarker* marker, double x, double y, double z)
{
	// Update the position of the marker
	marker->marker.pose.position.x = MM_SCALE * x;
	marker->marker.pose.position.y = MM_SCALE * y;
	marker->marker.pose.position.z = MM_SCALE * z + MM_OFFSET_Z;
}

// Update the orientation of the marker
void MagFilter::MarkerMan::updateOrientation(vis_utils::GenMarker* marker, double w, double x, double y, double z)
{
	// Update the orientation of the marker
	marker->marker.pose.orientation.w = w;
	marker->marker.pose.orientation.x = x;
	marker->marker.pose.orientation.y = y;
	marker->marker.pose.orientation.z = z;
}

// Update the scale of a marker
void MagFilter::MarkerMan::updateScale(vis_utils::GenMarker* marker, double scaleX, double scaleY, double scaleZ)
{
	// Update the scale of the marker
	marker->marker.scale.x = MM_SCALE * scaleX;
	marker->marker.scale.y = MM_SCALE * scaleY;
	marker->marker.scale.z = MM_SCALE * scaleZ;
}

// Set a line strip marker to show a particular ellipse in 3D (constrained to be parallel to the xy plane, angle is the CCW rotation of the ellipse)
void MagFilter::MarkerMan::setEllipsePoints(vis_utils::GenMarker* marker, const Eigen::Vector3d& centre, double radiusX, double radiusY, double angle)
{
	// Alias of the marker points vector
	std::vector<geometry_msgs::Point>& P = marker->marker.points;
	
	// Clear the points list
	P.clear();
	
	// Trigonometric precomputation
	double ca = cos(angle);
	double sa = sin(angle);
	
	// Calculate points that define the ellipse
	geometry_msgs::Point pt;
	for(size_t i = 0; i <= MM_ELLIPSE_POINTS; i++)
	{
		double t = i * (2.0*M_PI / MM_ELLIPSE_POINTS);
		double rawX = radiusX*cos(t);
		double rawY = radiusY*sin(t);
		pt.x = MM_SCALE*(centre.x() + ca*rawX - sa*rawY);
		pt.y = MM_SCALE*(centre.y() + sa*rawX + ca*rawY);
		pt.z = MM_SCALE*(centre.z()) + MM_OFFSET_Z;
		P.push_back(pt);
	}
}

// Set the lifetime of all the markers
void MagFilter::MarkerMan::setAllLifetimes(double lifetime)
{
	// Hide all the markers
	RawData.setLifetime(lifetime);
	AutoData.setLifetime(lifetime);
	FixedData.setLifetime(lifetime);
	HardCircle.setLifetime(lifetime);
	HardCircleC.setLifetime(lifetime);
	HardSphere.setLifetime(lifetime);
	HardSphereC.setLifetime(lifetime);
	HardPlaneNormal.setLifetime(lifetime);
	SoftEllipse.setLifetime(lifetime);
	SoftEllipseC.setLifetime(lifetime);
	SoftEllipsoid.setLifetime(lifetime);
	SoftEllipsoidC.setLifetime(lifetime);
	SoftFixedSphere.setLifetime(lifetime);
	SoftFixedSphereC.setLifetime(lifetime);
	AutoDataVar.setLifetime(lifetime);
	AutoDataMean.setLifetime(lifetime);
	AutoDataText.setLifetime(lifetime);
	AutoMeanCircle.setLifetime(lifetime);
	AutoMeanCircleC.setLifetime(lifetime);
	AutoMeanText.setLifetime(lifetime);
	AutoFittedCircle.setLifetime(lifetime);
	AutoFittedCircleC.setLifetime(lifetime);
	AutoFittedText.setLifetime(lifetime);
	AutoTargetCircle.setLifetime(lifetime);
	AutoTargetCircleC.setLifetime(lifetime);
	AutoTargetText.setLifetime(lifetime);
	AutoCurCircle.setLifetime(lifetime);
	AutoCurCircleC.setLifetime(lifetime);
	AutoCurText.setLifetime(lifetime);
}

// Publish the latest markers
void MagFilter::MarkerMan::publishMarkers()
{
	// Clear the current markers
	clear();
	
	// Update the lifetimes as required
	if(visible)
		setAllLifetimes(MM_LIFE_ETERNITY);
	else
		setAllLifetimes(MM_LIFE_SHORT);
	
	
	// Do certain preparatory work only if we're actually going to publish
	if(willPublish())
	{
		// Hide or show the point data
		if(RawData.marker.points.empty()) RawData.hide();
		else RawData.show();
		if(AutoData.marker.points.empty() || hideAuto) AutoData.hide();
		else AutoData.show();
		if(FixedData.marker.points.empty()) FixedData.hide();
		else FixedData.show();
		
		// Add all the markers to the publish array
		add(&RawData);
		add(&AutoData);
		add(&FixedData);
		add(&HardCircle);
		add(&HardCircleC);
		add(&HardSphere);
		add(&HardSphereC);
		add(&HardPlaneNormal);
		add(&SoftEllipse);
		add(&SoftEllipseC);
		add(&SoftEllipsoid);
		add(&SoftEllipsoidC);
		add(&SoftFixedSphere);
		add(&SoftFixedSphereC);
		add(&AutoDataVar);
		add(&AutoDataMean);
		add(&AutoDataText);
		add(&AutoMeanCircle);
		add(&AutoMeanCircleC);
		add(&AutoMeanText);
		add(&AutoFittedCircle);
		add(&AutoFittedCircleC);
		add(&AutoFittedText);
		add(&AutoTargetCircle);
		add(&AutoTargetCircleC);
		add(&AutoTargetText);
		add(&AutoCurCircle);
		add(&AutoCurCircleC);
		add(&AutoCurText);
	}
	
	// Publish the markers
	publish();
}

// Hide all the markers other than the data points
void MagFilter::MarkerMan::hideAll()
{
	// Hide all the markers
	HardCircle.hide();
	HardCircleC.hide();
	HardSphere.hide();
	HardSphereC.hide();
	HardPlaneNormal.hide();
	SoftEllipse.hide();
	SoftEllipseC.hide();
	SoftEllipsoid.hide();
	SoftEllipsoidC.hide();
	SoftFixedSphere.hide();
	SoftFixedSphereC.hide();
	AutoDataVar.hide();
	AutoDataMean.hide();
	AutoDataText.hide();
	AutoMeanCircle.hide();
	AutoMeanCircleC.hide();
	AutoMeanText.hide();
	AutoFittedCircle.hide();
	AutoFittedCircleC.hide();
	AutoFittedText.hide();
	AutoTargetCircle.hide();
	AutoTargetCircleC.hide();
	AutoTargetText.hide();
	AutoCurCircle.hide();
	AutoCurCircleC.hide();
	AutoCurText.hide();
}
// EOF
