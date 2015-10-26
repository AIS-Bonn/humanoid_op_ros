//CameraRealConvert.hpp
// Created on: Apr 22, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
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
#include <sensor_msgs/image_encodings.h>
#include <config_server/parameter.h>
#include <std_srvs/Empty.h>
#include <boost/format.hpp>
#include <visualization_msgs/Marker.h>
#include <vis_utils/marker_manager.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vision_module/Tools/General.hpp>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/MatPublisher.hpp>
#include <vision_module/Projections/IPM.hpp>
#include <vision_module/Projections/DistortionModel.hpp>
#include <vision_module/Tools/LineSegment.hpp>
#include <vision_module/Tools/HillOptimizer.hpp>
#include <robotcontrol/RobotHeading.h>

using namespace cv;

class CameraProjections
{
public:
	double headingOffset;//in radian
private:
	robotcontrol::RobotHeading headingData;
	ros::Subscriber heading_sub_robotstate;
	void handleHeadingData( const robotcontrol::RobotHeadingConstPtr& msg) //in radian
	{
		headingData=*msg;
	}

	ros::NodeHandle nodeHandle;
	tf::TransformListener *m_tf;
	vis_utils::MarkerManager topViewMarker;
	Mat realHomoFor, realHomoBack;

	Point2d _upRightPoint;
	Point2d _upRightPointt;
	Point2d _downRightPoint;
	Point2d _downLeftPoint;


	nav_msgs::OccupancyGrid mmap;
	ros::Publisher mmap_pub;
	ros::Publisher reset_time_pub;
	ros::Publisher reset_clock_pub;

	vis_utils::GenMarker topViewCPoints;
	vis_utils::GenMarker topViewGPoints;
	MatPublisher topImg_pub;
	MatPublisher unDistortedImg_pub;

	tf::StampedTransform tfOptical;
	sensor_msgs::CameraInfo ci;
	MatPublisher calibImg_pub;
	image_geometry::PinholeCameraModel cam_model;
	Mat calibViewBGR, topViewBGR;
	Point2d  gPointRes[4], physicalCorners[4];
	Point3d transformedPointsRes[4];

	bool lastProjectionIsValid;
	ros::Time lastTime;
	bool dummy;
	float ComputeError(vector<OptimizorParam> &d);
	float ComputeAvgError(	vector<Point2f> realClicked,vector<Point2f> rvizClicked);
	bool calibrating;
	vector<OptimizorParam> initialParameters;
	HillOptimizer<CameraProjections> optimizer;
	float diagnalAngleView;
	IPM ipm;
public:
	Point3d cameraLocation, cameraOrintation;
	Point3d OpticalAngle;
	Point2d outerCornetrsUndistortedImg[4];
	Point outerCornetrsRawImg[4];
	DistortionModel _distorionModel;

	double getHeading()
	{
		return CorrectAngleRadian360(headingData.heading+headingOffset);
	}
	void Calibrate();
	bool GetOnImageCordinate(const vector<Point2f> contour,
			vector<Point> &resCountour);

	bool GetOnRealCordinate(const vector<Point> contour,
			vector<Point2f> &resCountour);
	bool GetOnImageCordinate(const vector<LineSegment> inLine,
			vector<LineSegment> &resLines);
	bool GetOnRealCordinate(const vector<LineSegment> inLine,
			vector<LineSegment> &resLines);
	bool GetOnRealCordinate(const Point pointIn, Point2f &resPoint);
	bool GetOnImageCordinate(Point2f pointIn, Point &resPoint);
	bool Init(bool _dummy);
	inline CameraProjections() : headingOffset(0),topViewMarker("/vision/testPoints"), topViewCPoints(
					&topViewMarker, "/ego_floor"), topViewGPoints(
					&topViewMarker, "/ego_floor"), topImg_pub("/vision/topImg") ,unDistortedImg_pub("/vision/unDistortedImg"), calibImg_pub(
					"/vision/calibImg"), lastProjectionIsValid(false), lastTime(
					ros::Time::now()),dummy(false),calibrating(false),optimizer(*this,&CameraProjections::ComputeError),
					diagnalAngleView(params.camera.diagonalAngleView->get()),ipm(diagnalAngleView)
	{
		mmap_pub = nodeHandle.advertise<nav_msgs::OccupancyGrid>("/vision/mmap",
				10);
		reset_time_pub = nodeHandle.advertise<std_msgs::Empty>("/reset_time",
				10);
		reset_clock_pub = nodeHandle.advertise<std_msgs::Empty>("/reset_clock",
				10);
		m_tf = new tf::TransformListener(ros::Duration(10)); /*to cache data for 10 second before*/
		heading_sub_robotstate = nodeHandle.subscribe("/robotmodel/robot_heading", 1, &CameraProjections::handleHeadingData, this);
	}


	virtual inline ~CameraProjections()
	{
		delete m_tf;
	}
	vector<Point2f> RotateTowardHeading(vector<Point2f> in);
	Point2d RotateTowardHeading(Point2d in);
	Point2f RotateTowardHeading(Point2f in);
	vector<LineSegment> RotateTowardHeading(vector<LineSegment> in);
	bool CalculateProjection();
	bool Update();
	void Publish(Mat &guiRawImg, bool showHorizonBox);
};
