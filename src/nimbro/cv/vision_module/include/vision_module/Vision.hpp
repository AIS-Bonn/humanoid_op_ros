//Vision.hpp
// Created on: Apr 20, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
/** @addtogroup VisionModule */
/*@{*/

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <boost/timer/timer.hpp>
#include <inttypes.h>
#include <time.h>
#include <std_msgs/Int64.h>
#include <string.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <config_server/parameter.h>
#include <std_srvs/Empty.h>
#include <boost/format.hpp>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <math.h>
#include <algorithm>    // std::sort
#include <vis_utils/marker_manager.h>
#include <nav_msgs/OccupancyGrid.h>
#include <head_control/LookAtTarget.h>

#include <vision_module/Inputs/CameraDummy.hpp>
#include <vision_module/Tools/General.hpp>
#include <vision_module/Tools/Parameters.hpp>
#include <boost/foreach.hpp>
#include <vision_module/Inputs/Camera.hpp>
#include <vision_module/Projections/CameraProjections.hpp>
#include <vision_module/Tools/MatPublisher.hpp>
#include <vision_module/Projections/IPM.hpp>
#include <vision_module/Tools/LineSegment.hpp>
#include <vision_module/soccer_objects/FieldDetector.hpp>
#include <vision_module/soccer_objects/BallDetector.hpp>
#include <vision_module/soccer_objects/ObstacleDetector.hpp>
#include <vision_module/soccer_objects/LineDetector.hpp>
#include <vision_module/soccer_objects/GoalDetector.hpp>

#include <vision_module/Tools/HSVPresenter.hpp>
#include <vision_module/Inputs/ICamera.hpp>
#include <vector>
#include <robotcontrol/RobotHeading.h>

#define TEENSIZE_FIELD 0
#define BONN_FIELD 1
#if TEENSIZE_FIELD
const double A = 9;
const double B = 6;
const double E = 1;
const double F = 5;
const double G = 2.1;
const double H = 1.50;
const double D = 2.6;
#elif BONN_FIELD
const double A = 5.45;
const double B = 4.10;
const double E = 0.60;
const double F = 3.40;
const double G = 1.30;
const double H = 1.20;
const double D = 2.6;
#endif
/**
* @class Vision
* @brief This Class is for managing all the required actions to find interesting objects in soccer field
**/
class Vision
{
public:
	CameraProjections _cameraProjections;
private:

	const static double LOWPASSNORMAL = 0.1;
	const static double LOWPASSSTRONG = 0.2;
	const static double LOWPASSWEAK = 0.05;

	double getUpdateCoef(double coef, cv::Point2f point)
	{
		double distance = GetDistance(point);

		if (distance > 8)
			return 0;
		if (distance < 3)
			return coef;

		return coef * (1 - (distance / 8));
	}

	double getUpdateCoef(double coef, LineSegment line)
	{
		return getUpdateCoef(coef,
				line.GetClosestPointOnLineSegment(cv::Point2d(0, 0)));
	}

	Point3d Localization;
	double confidence;
	long int visionCounter;
	ICamera *cam;
	Mat rawHSV;
	Mat guiTopViwRotate, guiRawImg, guiRawUndistorted;
	Mat ballBinary, fieldBinary, fieldConvectHull, goalBinary, lineBinary,
			obstacleBinary;
	ros::NodeHandle nodeHandle;
	vis_utils::MarkerManager topViewMarker;
	vis_utils::MarkerManager topViewMarkerLR;
	vis_utils::MarkerManager localizationMarker;
	vis_utils::GenMarker topViewLines;
	vis_utils::GenMarker topViewGoalPostLeft;
	vis_utils::GenMarker topViewGoalPostRight;
	vis_utils::GenMarker topViewCircle;
	vis_utils::GenMarker topViewLinesLR;
	vis_utils::GenMarker topViewGoalPostLeftLR;
	vis_utils::GenMarker topViewGoalPostRightLR;
	vis_utils::GenMarker topViewCircleLR;
	vis_utils::GenMarker LocPhiMarker;
	vis_utils::SphereMarker ballMarker;
	vis_utils::SphereMarker LocMarker;
	MatPublisher fieldImg_pub;
	MatPublisher filedConvecxImg_pub;
	MatPublisher lineImg_pub;
	MatPublisher goalImg_pub;
	MatPublisher ballImg_pub;
	MatPublisher ballImg_debug_pub;
	MatPublisher guiRawImg_pub;
	MatPublisher ballMaskDebug_pub;
	ros::Publisher head_pub;
	Point3d Head_Control_Pos;

	FieldDetector _fieldDetector;
	BallDetector _ballDetector;
	LineDetector _lineDetector;
	GoalDetector _goalDetector;
	ObstacleDetector _obstacleDetector;
	HSVPresenter _hsvPresenter;
	ros::Subscriber mose_left_click_sub;
	ros::Subscriber mose_right_click_sub;
	ros::Subscriber mose_middle_click_sub;
	ros::Subscriber rviz_click_sub;

	vector<cv::Point2f> FieldHullReal;
	cv::Point2f FieldHullRealCenter;
	geometry_msgs::PointStamped ballTarget;

	ros::Publisher robotPos_pub;
	ros::Publisher ballTarget_pub;
	bool ballTargetUpdated;
	tf::TransformListener *m_tf;
	tf::StampedTransform tfOdom;
	float lastOdomX, lastOdomY;

public:
	Vision(bool dummy) :
		 _cameraProjections(),
			confidence(-1), visionCounter(0), topViewMarker(
					"/vision/ballPoint"), topViewMarkerLR(
					"/vision/topViewMarkerLR"), localizationMarker(
					"/vision/localization"), topViewLines(&topViewMarker,
					"/ego_floor"), topViewGoalPostLeft(&topViewMarker,
					"/ego_floor"), topViewGoalPostRight(&topViewMarker,
					"/ego_floor"), topViewCircle(&topViewMarker, "/ego_floor"), topViewLinesLR(
					&topViewMarkerLR, "/ego_floor"), topViewGoalPostLeftLR(
					&topViewMarkerLR, "/ego_floor"), topViewGoalPostRightLR(
					&topViewMarkerLR, "/ego_floor"), topViewCircleLR(
					&topViewMarkerLR, "/ego_floor"), LocPhiMarker(
					&localizationMarker, "/ego_floor"), ballMarker(
					&topViewMarker, "/ego_floor"), LocMarker(
					&localizationMarker, "/ego_floor"), fieldImg_pub(
					"/vision/fieldImg"), filedConvecxImg_pub(
					"/vision/fieldConvexImg"), lineImg_pub("/vision/lineImg"), goalImg_pub(
					"/vision/goalImg"), ballImg_pub("/vision/ballImg"), ballImg_debug_pub(
					"vision/ball_debugImg"), guiRawImg_pub("/vision/guiRawImg"), ballMaskDebug_pub(
					"/vision/ballMask"), _fieldDetector(), _ballDetector(), _lineDetector(), _goalDetector(), _obstacleDetector(), _hsvPresenter(), ballTarget(), ballTargetUpdated(
					false), lastOdomX(0), lastOdomY(0)
	{
		if (dummy == true)
		{
			cam = new CameraDummy();
		}
		else
		{
			cam = new Camera();
		}
		head_pub = nodeHandle.advertise<head_control::LookAtTarget>(
				"/robotcontrol/headcontrol/target", 10);
		ballTarget_pub = nodeHandle.advertise<geometry_msgs::PointStamped>(
				"/vision/ballTarget", 10);
		robotPos_pub = nodeHandle.advertise<geometry_msgs::PointStamped>(
						"/vision/robotPose", 10);
		mose_right_click_sub = nodeHandle.subscribe<geometry_msgs::Point>(
				"/event_image_view/mouse_right_click", 1,
				&Vision::mouse_right_click_callback, this);
		mose_left_click_sub = nodeHandle.subscribe<geometry_msgs::Point>(
				"/event_image_view/mouse_left_click", 1,
				&Vision::mouse_left_click_callback, this);
		mose_middle_click_sub = nodeHandle.subscribe<geometry_msgs::Point>(
				"/event_image_view/mouse_middle_click", 1,
				&Vision::mouse_middle_click_callback, this);
		rviz_click_sub = nodeHandle.subscribe<geometry_msgs::PointStamped>(
				"/clicked_point", 1, &Vision::rviz_click_callback, this);
		m_tf = new tf::TransformListener(ros::Duration(10));

//		m_tf->waitForTransform("/ego_rot", "/camera_optical", ros::Time(0),
//				ros::Duration(2));

	}
	bool GenerateTopViewImg(ros::Time now);
	void UpdateRangeImage(ros::Time now);
	void Process(ros::Time now);
	virtual ~Vision()
	{
		delete cam;
		delete m_tf;
	}
	bool update();
	bool Init();

	void rviz_click_callback(const geometry_msgs::PointStampedConstPtr& msg)
	{
		if (msg->point.z > 0.1 || msg->point.z < -0.1)
		{
			ROS_INFO("I heard: [%f , %f , %f]", msg->point.x, msg->point.y,
					msg->point.z);
			ROS_WARN("The clicked point in the rviz is not on the zero plane!");
			return;
		}
		ROS_INFO("I heard: [%f , %f]", msg->point.x, msg->point.y);
		if (params.camCalibrator.clicked.size()
				> params.camCalibrator.rvizClicked.size())
		{
			params.camCalibrator.rvizClicked.push_back(
					Point2d(msg->point.x, msg->point.y));
			params.camCalibrator.cameraLocation.push_back(
					_cameraProjections.cameraLocation);
			params.camCalibrator.opticalAngle.push_back(
					_cameraProjections.OpticalAngle);

			cout << "-->  cameraLocation = "
					<< _cameraProjections.cameraLocation;
			cout << "  opticalAngle = " << _cameraProjections.OpticalAngle
					<< endl;
		}
		else
		{
			params.camCalibrator.rvizClicked.pop_back();
			params.camCalibrator.cameraLocation.pop_back();
			params.camCalibrator.opticalAngle.pop_back();

			params.camCalibrator.rvizClicked.push_back(
					Point2d(msg->point.x, msg->point.y));
			params.camCalibrator.cameraLocation.push_back(
					_cameraProjections.cameraLocation);
			params.camCalibrator.opticalAngle.push_back(
					_cameraProjections.OpticalAngle);
		}

	}

	void mouse_left_click_callback(const geometry_msgs::Point::ConstPtr& msg)
	{
		ROS_INFO("I heard: [%f , %f]", msg->x, msg->y);
		if (params.camCalibrator.clicked.size()
				<= params.camCalibrator.rvizClicked.size())
		{
			params.camCalibrator.clicked.push_back(Point(msg->x, msg->y));
		}
		else
		{
			params.camCalibrator.clicked.pop_back();
			params.camCalibrator.clicked.push_back(Point(msg->x, msg->y));
		}
	}

	void mouse_right_click_callback(const geometry_msgs::Point::ConstPtr& msg)
	{
		ROS_INFO("I heard: [%f , %f]", msg->x, msg->y);

		if (params.camCalibrator.clicked.size() > 0)
		{
			params.camCalibrator.clicked.pop_back();
		}
		if (params.camCalibrator.clicked.size() > 0
				&& params.camCalibrator.clicked.size()
						< params.camCalibrator.rvizClicked.size())
		{
			params.camCalibrator.rvizClicked.pop_back();
			params.camCalibrator.cameraLocation.pop_back();
			params.camCalibrator.opticalAngle.pop_back();

		}

	}
	void mouse_middle_click_callback(const geometry_msgs::Point::ConstPtr& msg)
	{
		ROS_INFO("Clear All Points");
		params.camCalibrator.rvizClicked.clear();
		params.camCalibrator.cameraLocation.clear();
		params.camCalibrator.opticalAngle.clear();
		params.camCalibrator.clicked.clear();
	}
};
/** @}*/
