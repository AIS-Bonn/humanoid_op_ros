// Motion control for the robot head
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef HEADCONTROL_H
#define HEADCONTROL_H

// Includes
#include <robotcontrol/model/robotmodel.h>
#include <robotcontrol/motionmodule.h>
#include <head_control/LookAtTarget.h>
#include <head_control/HeadControlStatus.h>
#include <config_server/parameter.h>
#include <nimbro_utils/math_spline.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>

// Head control namespace
namespace headcontrol
{
	//
	// BoundarySegment struct
	//
	struct BoundarySegment
	{
		// Constructor
		BoundarySegment() { a = b = d = 0.0; c = 1.0; tStart = 0.0; tEnd = 1.0; useX = true; }
		BoundarySegment(double a, double b, double c, double d = 0.0, double tStart = 0.0, double tEnd = 1.0, bool useX = true) : a(a), b(b), c(c), d(d), tStart(tStart), tEnd(tEnd), useX(useX) {}
		
		// Validity functions
		bool invalid() const { return (a == 0.0 && b == 0.0 && d == 0.0); }
		static bool isInvalid(const BoundarySegment& B) { return B.invalid(); }
		
		// Shift function
		void shiftBy(double margin);
		
		// Serialisation
		static std::string serialiseArray(const std::vector<BoundarySegment>& array);
		static bool deserialiseArray(const std::string& str, std::vector<BoundarySegment>& array);
		
		// Data members (function is a*x + b*y + c + d*x^2 >= 0)
		double a; // Coeff of x
		double b; // Coeff of y
		double c; // Coeff of 1
		double d; // Coeff of x^2
		double tStart; // Start of parameterised range
		double tEnd; // End of parameterised range
		bool useX; // Flag whether the parameter is x or y (d is ignored if useX is false)
		
		// Constants
		static const double minC = 0.01;
	};
	
	//
	// HeadLimiter class
	//
	class HeadLimiter
	{
	public:
		// Constructor
		HeadLimiter() {}
		
		// Virtual functions
		virtual bool applyLimit(double& yaw, double& pitch) const { return false; } // This function should update yaw/pitch if necessary to be the closest possible that is in the allowable position space
	};
	
	//
	// SimpleLimiter class
	//
	class SimpleLimiter : public HeadLimiter
	{
	public:
		// Constants
		static const double minYawCmd = -1.57;
		static const double maxYawCmd = 1.57;
		static const double minPitchCmd = -0.5;
		static const double maxPitchCmd = 0.5;
		
		// Constructor
		SimpleLimiter() {}
		
		// Overrides
		virtual bool applyLimit(double& yaw, double& pitch) const;
	};
	
	//
	// BoundaryLimiter class
	//
	class BoundaryLimiter : public HeadLimiter
	{
	public:
		// Constructor
		BoundaryLimiter() : m_params(NULL) {}
		BoundaryLimiter(const std::vector<BoundarySegment>* params) : m_params(params) {}
		
		// Reference to boundary parameters
		void setParams(const std::vector<BoundarySegment>* params) { m_params = params; }
		const std::vector<BoundarySegment>* getParams() const { return m_params; }
		
		// Overrides
		virtual bool applyLimit(double& yaw, double& pitch) const;
		
	private:
		// Internal variables
		const std::vector<BoundarySegment>* m_params;
	};
	
	//
	// HeadControl class
	//
	class HeadControl : public robotcontrol::MotionModule
	{
	public:
		// Constructor
		HeadControl();
		virtual ~HeadControl();

		// Motion module overrides
		virtual bool init(robotcontrol::RobotModel* model);
		virtual bool isTriggered();
		virtual void step();

	private:
		// Return whether head control should be enabled
		bool active() const { return m_headControlEnabled() && m_haveTarget && (m_model->state() != m_state_kicking) && (m_model->state() != m_state_standingUp); }
		
		// Reset target function
		void resetTarget();
		
		// ROS topic handlers
		void handleLookAtTarget(const head_control::LookAtTargetConstPtr& msg);
		
		// Node handle
		ros::NodeHandle m_nh;

		// Publishers and subscribers
		ros::Subscriber m_sub_lookAtTarget;
		ros::Publisher m_pub_headControlStatus;
		
		// Head control status updates
		void updateHeadControlStatus();
		head_control::HeadControlStatus m_msgStatus;
		bool m_lastActive;

		// Pointer to robot model
		robotcontrol::RobotModel* m_model;

		// Robot joints
		robotcontrol::Joint::Ptr m_headYawJoint;
		robotcontrol::Joint::Ptr m_headPitchJoint;

		// Robotcontrol states
		robotcontrol::RobotModel::State m_state_kicking;
		robotcontrol::RobotModel::State m_state_standingUp;

		// Config server parameters
		config_server::Parameter<bool>  m_headControlEnabled;
		config_server::Parameter<bool>  m_yawRelax;
		config_server::Parameter<float> m_yawMaxVel;
		config_server::Parameter<float> m_yawMaxAcc;
		config_server::Parameter<float> m_yawDefaultEffort;
		config_server::Parameter<bool>  m_pitchRelax;
		config_server::Parameter<float> m_pitchMaxVel;
		config_server::Parameter<float> m_pitchMaxAcc;
		config_server::Parameter<float> m_pitchDefaultEffort;
		
		// Yaw tracking state
		double m_yawTarget;
		double m_yawCurX;
		double m_yawCurV;
		double m_yawCurEffort;
		double m_yawEffortTarget;
		nimbro_utils::TrapVelSpline m_yawSpline;
		nimbro_utils::LinearSpline m_yawEffortSpline;
		
		// Pitch tracking state
		double m_pitchTarget;
		double m_pitchCurX;
		double m_pitchCurV;
		double m_pitchCurEffort;
		double m_pitchEffortTarget;
		nimbro_utils::TrapVelSpline m_pitchSpline;
		nimbro_utils::LinearSpline m_pitchEffortSpline;
		
		// Joystick
		config_server::Parameter<bool> m_enableJoystick;
		ros::Subscriber m_sub_joystickData;
		void handleJoystickData(const sensor_msgs::JoyConstPtr& msg);
		bool m_joystickButton9Pressed;
		bool m_joystickSaysEnabled;
		
		// Miscellaneous
		bool m_haveTarget;
		BoundaryLimiter m_posLimiter;
		
		// Typedefs
		typedef std::pair<double, double> Point;
		
		// Calibration initialisation
		void initCalibration();
		
		// Data recording
		bool m_recording;
		void recordPoint();
		std::vector<Point> m_recordedData;
		
		// Calibration process
		bool m_calibrating;
		void subscribeClickedPoint() { m_sub_clickedPoint = m_nh.subscribe("/clicked_point", 1, &HeadControl::handleClickedPoint, this); }
		void unsubscribeClickedPoint() { m_sub_clickedPoint.shutdown(); }
		void handleClickedPoint(const geometry_msgs::PointStampedConstPtr& msg);
		void handleCalibParamString();
		BoundarySegment fitLine(const Point& p1, const Point& p2);
		BoundarySegment fitParabola(const Point& p1, const Point& p2, const Point& p3);
		ros::Subscriber m_sub_clickedPoint;
		std::vector<Point> m_calibData;
		std::vector<BoundarySegment> m_calibParams;
		std::vector<BoundarySegment> m_curCalibParams;
		config_server::Parameter<float> m_calibSafetyMargin;
		config_server::Parameter<std::string> m_calibParamString;
		
		// Calibration services
		ros::ServiceServer m_srv_startRecording;
		ros::ServiceServer m_srv_stopRecording;
		ros::ServiceServer m_srv_startCalibration;
		ros::ServiceServer m_srv_stopCalibration;
		ros::ServiceServer m_srv_showHeadLimits;
		bool startRecording(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&) { return startRecording(); }
		bool stopRecording(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&) { return stopRecording(); }
		bool startCalibration(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&) { return startCalibration(); }
		bool stopCalibration(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&) { return stopCalibration(); }
		bool showHeadLimits(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&) { return showHeadLimits(); }
		bool startRecording();
		bool stopRecording();
		bool startCalibration();
		bool stopCalibration();
		bool showHeadLimits();
		void calcCalibParams(bool shrink);
		void plotCalibParams();
		
		// Calibration visualisation
		void updatePlotCalibData();
		config_server::Parameter<bool> m_plotCalibData;
		visualization_msgs::Marker m_dataMarker;
		visualization_msgs::Marker m_calibMarker;
		ros::Publisher m_pub_dataMarker;
		ros::Publisher m_pub_calibMarker;
		
		// Test functions
		void testLimiter();
	};
}

#endif
// EOF