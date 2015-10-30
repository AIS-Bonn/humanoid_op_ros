// Magnetometer filter that performs a hard-iron correction
// File: magfilter.cpp
// Authors: Max Schwarz <max.schwarz@uni-bonn.de>
//          Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <robotcontrol/hw/magfilter.h>
#include <Eigen/SVD>
#include <numeric>
#include <sstream>

// Defines
#define MAG_PLOT_SCALE  2.0

// Namespaces
using namespace stateestimation;

//
// MagFilter class
//

// Constructor/destructor
MagFilter::MagFilter(const std::string& paramPath)
	: m_plot_calib_data(paramPath + "/plotCalibData", false)
	, m_enable_hard(paramPath + "/hard/enableHardIron", true)
	, m_hard_x(paramPath + "/hard/x", -1.0, 0.001, 1.0, 0.0)
	, m_hard_y(paramPath + "/hard/y", -1.0, 0.001, 1.0, 0.0)
	, m_hard_z(paramPath + "/hard/z", -1.0, 0.001, 1.0, 0.0)
	, m_enable_warp(paramPath + "/warp/enableAngleWarping", false)
	, m_warpParamString(paramPath + "/warp/paramString", "")
	, m_warpUseDegrees(paramPath + "/warp/useDegrees", true)
	, m_warp(CyclicWarp::RADIANS)
	, m_value(Eigen::Vector3d::Zero())
	, m_valuePreWarp(Eigen::Vector3d::Zero())
	, m_calibrating(false)
{
	// Retrieve a ROS node handle
	ros::NodeHandle nh("~");

	// Advertise the provided ROS services
	m_srv_startCalibration  = nh.advertiseService(paramPath + "/startCalibration" , &MagFilter::startCalibration , this);
	m_srv_stopCalibration2D = nh.advertiseService(paramPath + "/stopCalibration2D", &MagFilter::stopCalibration2D, this);
	m_srv_stopCalibration3D = nh.advertiseService(paramPath + "/stopCalibration3D", &MagFilter::stopCalibration3D, this);
	m_srv_warpClearPoints   = nh.advertiseService(paramPath + "/warpClearPoints",   &MagFilter::warpClearPoints, this);
	m_srv_warpAddPoint      = nh.advertiseService(paramPath + "/warpAddPoint",      &MagFilter::warpAddPoint, this);

	// Set up the plotting of the calibration data points
	m_pub_marker = nh.advertise<visualization_msgs::Marker>("mag_calib_marker", 1);
	m_marker.header.frame_id = "/ego_rot";
	m_marker.header.stamp = ros::Time::now();
	m_marker.ns = paramPath;
	m_marker.id = 0;
	m_marker.type = visualization_msgs::Marker::POINTS;
	m_marker.action = visualization_msgs::Marker::ADD;
	m_marker.pose.position.x = 0.0;
	m_marker.pose.position.y = 0.0;
	m_marker.pose.position.z = 2.0;
	m_marker.scale.x = 0.02;
	m_marker.scale.y = 0.02;
	m_marker.color.a = 1.0;
	m_marker.color.r = 0.5;
	m_marker.color.g = 0.0;
	m_marker.color.b = 1.0;
	m_marker.lifetime = ros::Duration(0, 0);

	// Set up the calibration data plotting config server parameter callback
	m_plot_calib_data.setCallback(boost::bind(&MagFilter::updatePlotCalibData, this));

	// Set up the warp parameter string callback
	m_warpParamString.setCallback(boost::bind(&MagFilter::handleWarpParamString, this));
	m_warpUseDegrees.setCallback(boost::bind(&MagFilter::handleWarpParamString, this));
	handleWarpParamString();
}
MagFilter::~MagFilter()
{
}

// Update functions
void MagFilter::update(const Eigen::Vector3d& mag)
{
	// Retrieve the current calibrated hard iron offset
	Eigen::Vector3d hard_iron(m_hard_x(), m_hard_y(), m_hard_z());

	// Offset the measured mag by the hard iron calibration offset
	Eigen::Vector3d mvalue;
	if(m_enable_hard())
		mvalue = mag - hard_iron;
	else
		mvalue = mag;

	// Apply angle warping
	m_valuePreWarp = mvalue;
	if(m_enable_warp())
	{
		double len = sqrt(mvalue.x()*mvalue.x() + mvalue.y()*mvalue.y());
		double yaw = atan2(-mvalue.y(), mvalue.x()); // The negation is intentional => This should be the heading on the field based on the magnetometer measurement assuming that the global magnetic field points towards the positive goal
		double wyaw = m_warp.unwarp(yaw);
		mvalue.x() = len*cos(-wyaw);                 // The negation is intentional => See above
		mvalue.y() = len*sin(-wyaw);                 // The negation is intentional => See above
	}

	// Store the new magnetometer value
	m_value = mvalue;

	// Store the current mag measurement if we are currently running the calibration
	if(m_calibrating)
	{
		m_calibrationMeasurements.push_back(mag);
		geometry_msgs::Point p;
		p.x = MAG_PLOT_SCALE * mag.x();
		p.y = MAG_PLOT_SCALE * mag.y();
		p.z = MAG_PLOT_SCALE * mag.z();
		m_marker.points.push_back(p);
		updatePlotCalibData();
	}
}
void MagFilter::update(double magX, double magY, double magZ)
{
	// Piggyback on the other overload of the update() function...
	Eigen::Vector3d mag(magX, magY, magZ);
	update(mag);
}

// Start calibration service handler
bool MagFilter::startCalibration(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	// To use this service to calibrate the magnetometer, have this magnetometer filter running and constantly receiving new
	// data points from the sensor (e.g. have robotcontrol running with the NimbRo-OP robotinterface using robot_calib.launch).
	// Then, manually trigger this service using:
	//    rosservice call /robotcontrol/magnetometer/startCalibration
	// From that point onwards all data points are stored in a buffer until a stop calibration service call is made:
	//    rosservice call /robotcontrol/magnetometer/stopCalibration3D
	// In the meantime, try to rotate the robot in all possible directions (upside down, face down/up/left/right, etc...).
	// Once the calibration has been stopped, you can read the results of the calibration from the console in which
	// robotcontrol was started, or from the config server. You can use the Parameter Tuner for that, or:
	//    rosservice call /config_server/get_parameter 'name: "/robotcontrol/magnetometer/hard/x"'
	//    rosservice call /config_server/get_parameter 'name: "/robotcontrol/magnetometer/hard/y"'
	//    rosservice call /config_server/get_parameter 'name: "/robotcontrol/magnetometer/hard/z"'
	// You should write down the calibration results (3 floats), and if the calibration is good, then transcribe the values
	// manually into the correct config_*.yaml file. If you do not then the calibration is lost the next time you start up
	// the config server/robotcontrol!
	// By setting the /robotcontrol/magnetometer/plotCalibData boolean config parameter and having a Marker display for the
	// ROS topic /robotcontrol/mag_calib_marker in Rviz, you can check the quality of the magnetometer calibration. Ideally
	// if you rotated the robot enough you should see a 'sphere' of data points with one single point in the middle that
	// corresponds to the calculated hard iron offset.

	// Inform the user that the magnetometer calibration has started
	ROS_INFO("Starting magnetometer calibration...");
	ROS_INFO("If using stopCalibration2D: Keep the robot perfectly upright and rotate it in pure global yaw");
	ROS_INFO("If using stopCalibration3D: Rotate the robot in all possible orientations including upside-down");

	// Set the internal calibrating flag
	m_calibrating = true;

	// Clear the data point lists
	m_calibrationMeasurements.clear();
	m_marker.points.clear();

	// Publish the cleared marker
	updatePlotCalibData();

	// Return that the service call was successfully handled
	return true;
}

// Stop calibration 2D service handler
bool MagFilter::stopCalibration2D(robotcontrol::MagCalibRequest& req, robotcontrol::MagCalibResponse& resp)
{
	// This calibration method assumes that the robot was rotated purely about its z-axis while upright.
	// Suppose the i-th measurement is (mxi,myi,mzi). We discard the z component and try to fit a circle
	// of radius R and centre (cx,cy) to the (mxi,myi) data. Then:
	// A = N rows of [mxi myi 1]              {Nx3 matrix}
	// x = [2*cx; 2*cy; R^2-cx^2-cy^2]        {3x1 column vector}
	// b = N rows of [mxi^2 + myi^2]          {Nx1 column vector}
	// Ax = b is equivalent to the condition that all measured data points lie on a circle of radius R,
	// centered at (cx,cy). Hence by solving Ax = b for x in a least squares sense, we are implicitly
	// solving for the values of cx, cy, R for which the corresponding circle most closely fits the data.

	// Don't do anything if a calibration isn't active
	if(!m_calibrating)
	{
		ROS_INFO("StopCalibration2D: Service call received even though no calibration is active");
		if(m_plot_calib_data() && !m_marker.points.empty())
		{
			ROS_INFO("StopCalibration2D: Republishing data points of last calibration");
			updatePlotCalibData();
		}
		resp.hardIronX = resp.hardIronY = resp.hardIronZ = 0.0;
		return true;
	}

	// Indicate to the user that this service call has been received
	ROS_INFO("Stopping magnetometer calibration (2D)...");

	// Declare variables
	size_t N = m_calibrationMeasurements.size();
	Eigen::MatrixXd A(N, 3);
	Eigen::VectorXd b(N);

	// Construct the linear equation to solve (Ax = b)
	for(size_t i = 0; i < N; i++)
	{
		const Eigen::Vector3d m = m_calibrationMeasurements[i];
		A(i, 0) = m.x();
		A(i, 1) = m.y();
		A(i, 2) = 1;
		b(i) = m.x()*m.x() + m.y()*m.y();
	}

	// Find the least squares solution to Ax = b using Singular Value Decomposition (linear least squares fitting)
	Eigen::VectorXd coefficients = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	// Extract the hard iron calibration coefficients and the magnetic field strength
	double offsetX = coefficients(0) / 2.0;
	double offsetY = coefficients(1) / 2.0;
	double radius  = sqrt(coefficients(2) + offsetX*offsetX + offsetY*offsetY);

	// Inform the user of the results of the magnetometer calibration
	ROS_INFO("Calibration (2D) ended with hard iron offset (%10.7lf, %10.7lf, #####) and radius %10.7lf gauss", offsetX, offsetY, radius);

	// Unset the internal calibrating flag
	m_calibrating = false;

	// Update the x/y hard iron calibration coefficients on the config server (from which they are retrieved in every call to update())
	m_hard_x.set((float) offsetX);
	m_hard_y.set((float) offsetY);

	// Calculate an average z value
	double offsetZ = m_hard_z();
	if(N >= 1)
		offsetZ = std::accumulate(m_calibrationMeasurements.begin(), m_calibrationMeasurements.end(), Eigen::Vector3d::Zero().eval()).z() / m_calibrationMeasurements.size();

	// Add the calculated hard iron offset point to the list
	geometry_msgs::Point p;
	p.x = MAG_PLOT_SCALE * offsetX;
	p.y = MAG_PLOT_SCALE * offsetY;
	p.z = MAG_PLOT_SCALE * offsetZ;
	m_marker.points.push_back(p);
	updatePlotCalibData();

	// Populate the response packet of the service call
	resp.hardIronX = offsetX;
	resp.hardIronY = offsetY;
	resp.hardIronZ = offsetZ;

	// Return that the service call was successfully handled
	return true;
}

// Stop calibration 3D service handler
bool MagFilter::stopCalibration3D(robotcontrol::MagCalibRequest& req, robotcontrol::MagCalibResponse& resp)
{
	// The hard iron calibration method used here is described in:
	//    Freescale Semiconductor AN4246: Calibrating an eCompass in the Presence of Hard and Soft-Iron Interference
	//    Freescale Semiconductor AN4399: High Precision Calibration of a Three-Axis Accelerometer
	// Suppose the i-th measurement is (mxi,myi,mzi), and we are trying to find the hard iron calibration
	// offset (cx,cy,cz) (centre of the sphere that the measurements should be on), and the magnetic field
	// strength B (radius of the sphere on which the measurements should be on). Then:
	// A = N rows of [mxi myi mzi 1]                  {Nx4 matrix}
	// x = [2*cx; 2*cy; 2*cz; B^2-cx^2-cy^2-cz^2]     {4x1 column vector}
	// b = N rows of [mxi^2 + myi^2 + mzi^2]          {Nx1 column vector}
	// Ax = b is equivalent to the condition that all measured data points lie on a sphere of radius B,
	// centered at (cx,cy,cz). Hence by solving Ax = b for x in a least squares sense, we are implicitly
	// solving for the values of cx, cy, cz, B for which the corresponding sphere most closely fits the data.

	// Don't do anything if a calibration isn't active
	if(!m_calibrating)
	{
		ROS_INFO("StopCalibration3D: Service call received even though no calibration is active");
		if(m_plot_calib_data() && !m_marker.points.empty())
		{
			ROS_INFO("StopCalibration3D: Republishing data points of last calibration");
			updatePlotCalibData();
		}
		resp.hardIronX = resp.hardIronY = resp.hardIronZ = 0.0;
		return true;
	}

	// Indicate to the user that this service call has been received
	ROS_INFO("Stopping magnetometer calibration (3D)...");

	// Declare variables
	size_t N = m_calibrationMeasurements.size();
	Eigen::MatrixXd A(N, 4);
	Eigen::VectorXd b(N);

	// Construct the linear equation to solve for the hard iron calibration coefficients (Ax = b)
	for(size_t i = 0; i < N; i++)
	{
		const Eigen::Vector3d m = m_calibrationMeasurements[i];
		A.row(i).head<3>() = m;
		A(i, 3) = 1;
		b(i) = m.squaredNorm();
	}

	// Find the least squares solution to Ax = b using Singular Value Decomposition (linear least squares fitting)
	Eigen::VectorXd coefficients = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	// Extract the hard iron calibration coefficients and the magnetic field strength
	Eigen::Vector3d hard_iron = coefficients.head<3>() / 2.0;
	double B = sqrt(coefficients(3) + hard_iron.squaredNorm());

	// Inform the user of the results of the magnetometer calibration
	ROS_INFO("Calibration (3D) ended with hard iron offset (%10.7lf, %10.7lf, %10.7lf) and field strength %10.7lf gauss", hard_iron.x(), hard_iron.y(), hard_iron.z(), B);

	// Unset the internal calibrating flag
	m_calibrating = false;

	// Update the hard iron calibration coefficients on the config server (from which they are retrieved in every call to update())
	m_hard_x.set((float) hard_iron.x());
	m_hard_y.set((float) hard_iron.y());
	m_hard_z.set((float) hard_iron.z());

	// Add the calculated hard iron offset point to the list
	geometry_msgs::Point p;
	p.x = MAG_PLOT_SCALE * hard_iron.x();
	p.y = MAG_PLOT_SCALE * hard_iron.y();
	p.z = MAG_PLOT_SCALE * hard_iron.z();
	m_marker.points.push_back(p);
	updatePlotCalibData();

	// Populate the response packet of the service call
	resp.hardIronX = hard_iron.x();
	resp.hardIronY = hard_iron.y();
	resp.hardIronZ = hard_iron.z();

	// Return that the service call was successfully handled
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

// Update functions for config server parameters
void MagFilter::updatePlotCalibData()
{
	// If the config parameter has changed whether we should be plotting the calib data or not, then republish the marker as appropriate
	if(m_plot_calib_data() && !m_marker.points.empty())
		m_marker.lifetime = ros::Duration(0, 0); // Set the marker to never expire
	else
		m_marker.lifetime = ros::Duration(0.05); // Set the marker to expire after a short amount of time
	m_marker.header.stamp = ros::Time::now();
	m_pub_marker.publish(m_marker);
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

//
// CyclicWarp class
//

// Constants
const double CyclicWarp::UNIT = 1.0;
const double CyclicWarp::RADIANS = 2.0*M_PI;
const double CyclicWarp::DEGREES = 360.0;

// Reset functions
void CyclicWarp::reset(double modulus)
{
	// Reset variables
	m_M = modulus;
	clear();
}
void CyclicWarp::clear()
{
	// Set warping to identity transformation
	m_N = 1;
	m_raw.assign(1, 0.0);
	m_warped.assign(1, 0.0);
}

// Get function for the value vectors
void CyclicWarp::getRefValues(std::vector<double>& raw, std::vector<double>& warped)
{
	// Copy out the reference values
	raw = m_raw;
	warped = m_warped;
}

// Set function for the value vectors
bool CyclicWarp::setRefValues(const std::vector<double>& raw, const std::vector<double>& warped)
{
	// Get the number of reference values
	size_t N = raw.size();
	if(N < 1 || N != warped.size())
		return false;
	
	// Error checking for jumps over M
	size_t rstart = N, wstart = N;
	for(size_t i = 0; i < N; i++)
	{
		if(raw[i] < 0.0 || raw[i] >= m_M) return false; // Value out of range...
		if(warped[i] < 0.0 || warped[i] >= m_M) return false; // Value out of range...
		size_t j = (i + 1) % N;
		if(raw[i] >= raw[j])
		{
			if(rstart != N) return false; // More than one jump over M...
			rstart = j;
		}
		if(warped[i] >= warped[j])
		{
			if(wstart != N) return false; // More than one jump over M...
			wstart = j;
		}
	}
	
	// Set the parameters
	m_N = N;
	m_raw = raw;
	m_warped = warped;
	
	// Return success
	return true;
}

// Tranformation worker function
double CyclicWarp::transform(const std::vector<double>& in, const std::vector<double>& out, double inValue) const
{
	// Ensure the input value is in range
	inValue = wrap(inValue);
	
	// Initialise the output value
	double outValue = inValue;
	
	// Perform the required warp lookup
	for(size_t i = 0; i < m_N; i++)
	{
		// Get the current and next reference values
		size_t j = (i + 1) % m_N;
		double idi = in[i];
		double idj = in[j];
		double odi = out[i];
		double odj = out[j];
		
		// If the output reference values jump over M then increase the second output value by the modulus for correct output space interpolation
		if(odi >= odj)
			odj += m_M;
		
		// If the input reference values do not jump over M and contain the input value then interpolate
		if(idi <= inValue && inValue <= idj && idi != idj)
		{
			outValue = odi + ((inValue - idi) / (idj - idi)) * (odj - odi);
			break;
		}
		
		// If the input reference values jump over M then adjust the input reference and interpolate
		if(idi >= idj)
		{
			if(inValue >= idi)
			{
				idj += m_M;
				outValue = odi + ((inValue - idi) / (idj - idi)) * (odj - odi);
				break;
			}
			if(inValue <= idj)
			{
				idi -= m_M;
				outValue = odi + ((inValue - idi) / (idj - idi)) * (odj - odi);
				break;
			}
		}
	}
	
	// Ensure the output is in range
	outValue = wrap(outValue);
	
	// Return the output value
	return outValue;
}
// EOF