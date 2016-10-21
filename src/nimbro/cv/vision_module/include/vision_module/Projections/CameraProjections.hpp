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
#include <std_msgs/Int8.h>
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
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_module/Tools/HillOptimizer.hpp>
#include <vision_module/Tools/SimplexOptimizer.hpp>
#include <robotcontrol/RobotHeading.h>
#include <tf_tools/tf_filtered_listener.h>

using namespace cv;
/**
* @ingroup VisionModule
*
* @brief For pixel projections
**/
class CameraProjections
{
private:
	vector<Point2f> realCoordinateVector;
	robotcontrol::RobotHeading headingData;
	ros::Subscriber heading_sub_robotstate;
	ros::NodeHandle nodeHandle;
	tf_tools::FilteredListener m_tf;
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
	vis_utils::GenMarker egoCPoints;
	vis_utils::GenMarker egoGPoints;
	MatPublisher topImg_pub;
	MatPublisher unDistortedImg_pub;
	sensor_msgs::CameraInfo ci;
//	MatPublisher calibImg_pub;
	image_geometry::PinholeCameraModel cam_model;
	Mat calibViewBGR, topViewBGR;
	Point2d gPointRes[4], physicalCorners[4];
	Point3d transformedPointsRes[4];
	bool lastProjectionIsValid;
	ros::Time lastTime;
	bool dummy;
	float ComputeError(const vector<OptimizorParam> &d);
	float ComputeError(const vector<float> &d);
	float ComputeAvgError(vector<Point2f> realClicked,
			vector<Point2f> rvizClicked);
	bool calibrating;
	vector<OptimizorParam> initialParameters;
	SimplexOptimizer<CameraProjections> simplexoptimizer;
	HillOptimizer<CameraProjections> hilloptimizer;
	float diagnalAngleView;
	float ballExtraDistance;
	IPM ipm;

	inline void handleHeadingData(const robotcontrol::RobotHeadingConstPtr& msg) //in radian
	{
		headingData = *msg;
	}

public:
	double headingOffset; //in radian
	double lastAvalibleTF; //in radian
	Point3d cameraLocation, cameraOrintation;
	Point3d opticalLocation, opticalAngle;
	Point2d outerCornetrsUndistortedImg[4];
	Point outerCornetrsRawImg[4];
	DistortionModel distorionModel;
	ros::Subscriber checkboxSub;
	vis_utils::MarkerManager kinCalibMarker;
	vis_utils::GenMarker kinCalibRvizPoints;
	vis_utils::GenMarker kinCalibResPoints;
	vis_utils::GenMarker kinCalibRobotLoc;
	ros::Publisher pcl_pub;
	sensor_msgs::PointCloud2 output;

	inline CameraProjections() :
			m_tf(ros::Duration(3)) /*to cache data for 3 second before*/, topViewMarker(
					"/vision/testPoints"), egoCPoints(&topViewMarker,
					"/ego_floor"), egoGPoints(&topViewMarker, "/ego_floor"), topImg_pub(
					"/vision/topImg"), unDistortedImg_pub(
					"/vision/unDistortedImg"), /*calibImg_pub("/vision/calibImg"),*/lastProjectionIsValid(
					false), lastTime(ros::Time::now()), dummy(false), calibrating(
					false), simplexoptimizer(*this,
					&CameraProjections::ComputeError), hilloptimizer(*this,
					&CameraProjections::ComputeError), diagnalAngleView(
					params.camera->diagonalAngleView()), ballExtraDistance(0), ipm(
					diagnalAngleView), headingOffset(0), lastAvalibleTF(0), kinCalibMarker(
					"/vision/kinCalib"), kinCalibRvizPoints(&kinCalibMarker,
					"/loc_field"), kinCalibResPoints(&kinCalibMarker,
					"/loc_field"), kinCalibRobotLoc(&kinCalibMarker,
					"/loc_field")
	{

		mmap_pub = nodeHandle.advertise<nav_msgs::OccupancyGrid>("/vision/mmap",
				10);

		reset_time_pub = nodeHandle.advertise<std_msgs::Empty>("/reset_time",
				10);
		reset_clock_pub = nodeHandle.advertise<std_msgs::Empty>("/reset_clock",
				10);
		pcl_pub = nodeHandle.advertise<sensor_msgs::PointCloud2>("/vision/pcl",
				1);

		m_tf.setFutureIgnoreRatio(0.5);
		std::vector<std::string> intrestingTFFrames;
		intrestingTFFrames.push_back("ego_floor");
		intrestingTFFrames.push_back("ego_rot");
		intrestingTFFrames.push_back("trunk_link");
		intrestingTFFrames.push_back("neck_link");
		intrestingTFFrames.push_back("head_link");
		intrestingTFFrames.push_back("camera_optical");
		m_tf.start(intrestingTFFrames);

		heading_sub_robotstate = nodeHandle.subscribe(
				"/robotmodel/robot_heading", 1,
				&CameraProjections::handleHeadingData, this);
	}

	//To find out the point in real coordinate with respect to camera
	inline Point2d unroateCameraYaw(Point2d _realPoint)
	{
		Point2d res;
		RotateAroundPoint(_realPoint, -Radian2Degree(cameraOrintation.z), res);
		return res;
	}

	inline double getHeading() //In Radian
	{
		if (abs(Radian2Degree(headingOffset)) > 50)
		{
			ROS_WARN("Flip prevented!");
			headingOffset = 0;
		}
		return CorrectAngleRadian360(headingData.heading + headingOffset);
	}
	virtual inline ~CameraProjections()
	{

	}
	vector<Point2f> RotateTowardHeading(const vector<Point2f> &in);
	Point2d RotateTowardHeading(const Point2d &in);
	Point2f RotateTowardHeading(const Point2f &in);
	vector<LineSegment> RotateTowardHeading(const vector<LineSegment> &in);
	void CalcFullRealCordinate();
	bool CalculateProjection();
	bool Update(ros::Time capTime);
	void Publish(Mat &guiRawImg, bool shouldPublish);
	void Calibrate();

	bool GetOnImageCordinate(const vector<Point2f> &contour,
			vector<Point> &resCountour);

	bool GetOnRealCordinate(const vector<Point> &contour,
			vector<Point2f> &resCountour);
	bool GetOnImageCordinate(const vector<LineSegment> &inLine,
			vector<LineSegment> &resLines);
	bool GetOnRealCordinate(const vector<LineSegment> &inLine,
			vector<LineSegment> &resLines);
	bool GetOnRealCordinate_FromUndistorted(const vector<Point2f> &contour,
			vector<Point2f> &resCountour);
	bool GetOnRealCordinate_FromUndistorted_Single(
			Point2f &pointIn, Point2f &resPoint);
	bool GetOnRealCordinate_single(const Point &pointIn, Point2f &resPoint);
	bool GetOnImageCordinate_slow(const Point2f &pointIn, Point &resPoint);
	Point2f convertToBallProjection(Point2f _in);

	bool Init(bool _dummy);

};
