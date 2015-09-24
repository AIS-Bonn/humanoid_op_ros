// Magnetometer filter that performs a hard-iron correction
// File: magfilter.cpp
// Authors: Max Schwarz <max.schwarz@uni-bonn.de>
//          Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <robotcontrol/hw/magfilter.h>
#include <Eigen/SVD>

// Defines
#define MAG_PLOT_SCALE  2.0

// Namespaces
using namespace stateestimation;

// Constructor/destructor
MagFilter::MagFilter(const std::string& paramPath)
	: m_plot_calib_data(paramPath + "/plotCalibData", false)
	, m_hard_x(paramPath + "/hard/x", -1.0, 0.001, 1.0, 0.0)
	, m_hard_y(paramPath + "/hard/y", -1.0, 0.001, 1.0, 0.0)
	, m_hard_z(paramPath + "/hard/z", -1.0, 0.001, 1.0, 0.0)
	, m_value(Eigen::Vector3d::Zero())
	, m_calibrating(false)
{
	// Retrieve a ROS node handle
	ros::NodeHandle nh("~");

	// Advertise the provided ROS services
	m_srv_startCalibration  = nh.advertiseService(paramPath + "/startCalibration" , &MagFilter::startCalibration , this);
	m_srv_stopCalibration   = nh.advertiseService(paramPath + "/stopCalibration"  , &MagFilter::stopCalibration  , this);
	m_srv_stopCalibration2D = nh.advertiseService(paramPath + "/stopCalibration2D", &MagFilter::stopCalibration2D, this);

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
	m_value = mag - hard_iron;

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

// ROS service handlers
bool MagFilter::startCalibration(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&)
{
	// To use this service to calibrate the magnetometer, have this magnetometer filter running and constantly receiving new
	// data points from the sensor (e.g. have robotcontrol running with the NimbRo-OP robotinterface using robot_calib.launch).
	// Then, manually trigger this service using:
	//    rosservice call /robotcontrol/magnetometer/startCalibration
	// From that point onwards all data points are stored in a buffer until a stop calibration service call is made:
	//    rosservice call /robotcontrol/magnetometer/stopCalibration
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
	ROS_INFO("If using stopCalibration:   Rotate the robot in all possible orientations including upside-down");
	ROS_INFO("If using stopCalibration2D: Keep the robot perfectly upright and rotate it in pure global yaw");

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
bool MagFilter::stopCalibration(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&)
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
		ROS_INFO("StopCalibration: Service call received even though no calibration is active");
		if(m_plot_calib_data() && !m_marker.points.empty())
		{
			ROS_INFO("StopCalibration: Republishing data points of last calibration");
			updatePlotCalibData();
		}
		return true;
	}

	// Indicate to the user that this service call has been received
	ROS_INFO("Stopping magnetometer calibration...");

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
	ROS_INFO("Calibration ended with hard iron offset (%10.7lf, %10.7lf, %10.7lf) and field strength %10.7lf gauss", hard_iron.x(), hard_iron.y(), hard_iron.z(), B);

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

	// Return that the service call was successfully handled
	return true;
}
bool MagFilter::stopCalibration2D(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&)
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
	ROS_INFO("Calibration (2D) ended with recommended hard iron offset (%10.7lf, %10.7lf, #####) and radius %10.7lf gauss. No config server parameters have been automatically updated.", offsetX, offsetY, radius);

	// Unset the internal calibrating flag
	m_calibrating = false;

	// Add the calculated hard iron offset point to the list
	geometry_msgs::Point p;
	p.x = MAG_PLOT_SCALE * offsetX;
	p.y = MAG_PLOT_SCALE * offsetY;
	if(N >= 1) p.z = MAG_PLOT_SCALE * m_calibrationMeasurements[0].z();
	else p.z = MAG_PLOT_SCALE * m_hard_z();
	m_marker.points.push_back(p);
	updatePlotCalibData();

	// Return that the service call was successfully handled
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
// EOF