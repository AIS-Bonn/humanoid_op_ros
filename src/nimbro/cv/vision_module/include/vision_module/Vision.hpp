//Vision.hpp
// Created on: Apr 20, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
/**
* @defgroup VisionModule Vision Module Library
*
* @author Hafez Farazi (<farazi@ais.uni-bonn.de>)
* @date Apr 20, 2015
* @version 1.1
*
* @section sclsec1 Overview
* The %Vision Module Library is for detecting soccer objects in the filed.
* @section sclsec1a Academic Sources
* The %Vision Module Library is detailed in the following paper.
*
* > H.Farazi , P. Allgeuer and S. Behnke, "A Monocular Vision System for Playing Soccer in Low Color Information Environments
* > ," in _Proceedings of the 10th Workshop on Humanoid Soccer Robots,
* > IEEE-RAS Int. Conference on Humanoid Robots_, Seoul, Korea, 2015.
*
* You are kindly asked to cite this paper if you use this framework for academic work.
@verbatim
@InProceedings{Farazi2015,
  Title                    = {A Monocular Vision System for Playing Soccer in Low Color Information Environments},
  Author                   = {Hafez Farazi and Philipp Allgeuer and Sven Behnke},
  Booktitle                = {Proceedings of 10th Workshop on Humanoid Soccer Robots, IEEE-RAS Int. Conference on Humanoid Robots},
  Year                     = {2015},
  Address                  = {Seoul, Korea}
}
@endverbatim
*
**/
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
#include <vision_module/Inputs/CameraDummy.hpp>
#include <vision_module/Tools/General.hpp>
#include <vision_module/Tools/Parameters.hpp>
#include <boost/foreach.hpp>
#include <vision_module/Inputs/Camera.hpp>
#include <vision_module/Projections/CameraProjections.hpp>
#include <vision_module/Tools/MatPublisher.hpp>
#include <vision_module/Projections/IPM.hpp>
#include <vision_module/Tools/LineSegment.hpp>
#include <vision_module/vision_outputs.h>
#include <vision_module/Tools/HSVPresenter.hpp>
#include <vision_module/Inputs/ICamera.hpp>
#include <vector>
#include <robotcontrol/RobotHeading.h>
#include <plot_msgs/plot_manager.h>
#include <vision_module/SoccerObjects/BallDetector.hpp>
#include <vision_module/SoccerObjects/CircleDetector.hpp>
#include <vision_module/SoccerObjects/FieldDetector.hpp>
#include <vision_module/SoccerObjects/GoalDetector.hpp>
#include <vision_module/SoccerObjects/LineDetector.hpp>
#include <vision_module/SoccerObjects/ObstacleDetector.hpp>
#include <vision_module/Localization/Localization.hpp>
#include <vision_module/vision_outputs.h>
#include <vision_module/localization_output.h>
#include <numeric>
#include <vision_module/Tools/GuiManager.hpp>

/**
* @ingroup VisionModule
*
* @brief This class is for managing all the required actions to find interesting objects in soccer field
**/
class Vision
{
public:
	CameraProjections ProjectionObj;
	Mat RawHSVImg;
	Mat GrayImg;
private:
	bool destroyed;
	long int visionCounter;
	bool updateGuiImg;
	ICamera *cam;
	Mat guiTopViwRotate, guiImg, guiUndistorted;
	Mat ballBinary, fieldBinary, fieldConvectHull, goalBinary, cannyImgInField,
			obstacleBinary;
	ros::NodeHandle nodeHandle;
	vis_utils::MarkerManager egoDetectionMarker;
	vis_utils::MarkerManager localizationMarker;
	vis_utils::GenMarker egoLinesM;
	vis_utils::GenMarker egoFieldM;
	vis_utils::GenMarker egoGoalPostLM;
	vis_utils::GenMarker egoGoalPostRM;
	vis_utils::GenMarker egoCircleM;
	vis_utils::SphereMarker egoBallM;
	vis_utils::GenMarker locPhiM;
	vis_utils::SphereMarker locM;
	MatPublisher edgeImg_pub;
	MatPublisher guiImg_pub;
	MatPublisher webImg_pub;
	Point3d head_Control_Pos;
	FieldDetector fieldDetector;
	BallDetector ballDetector;
	LineDetector lineDetector;
	CircleDetector circleDetector;
	GoalDetector goalDetector;
	ObstacleDetector obstacleDetector;
	vector<ObstacleC> obstacles;
	Localization loc;
	HSVPresenter hsvPresenter;
	GuiManager guiManager;
	vector<cv::Point2f> fieldHullReal;
	vector<cv::Point2f> fieldHullRealRotated;
	cv::Point2f fieldHullRealCenter;
	ros::Publisher visionOutputs_pub;

	enum PMIds
	{
		PM_FRAME_RATE = 0,
		PM_HEADING_OFFSET,
		PM_LAST_AVALIBLE_TF,
		PM_OBSTACLE_TIME,
		PM_LINE_TIME,
		PM_BALL_TIME,
		PM_LOCALIZATION_TIME,
		PM_GOAL_TIME,
		PM_CIRCLE_TIME,
		PM_COUNT
	};
	plot_msgs::PlotManagerFS plotM;
	string rName;
	vision_module::vision_outputs outputs;
public:
	Vision(bool dummy) :
			ProjectionObj(), destroyed(false), visionCounter(0), updateGuiImg(
					false), egoDetectionMarker("/vision/egoDetectionMarker"), localizationMarker(
					"/vision/localization"), egoLinesM(&egoDetectionMarker,
					"/ego_floor", "lines"), egoFieldM(&egoDetectionMarker,
					"/ego_floor", "field"), egoGoalPostLM(&egoDetectionMarker,
					"/ego_floor", "goals"), egoGoalPostRM(&egoDetectionMarker,
					"/ego_floor", "goals"), egoCircleM(&egoDetectionMarker,
					"/ego_floor", "circle"), egoBallM(&egoDetectionMarker,
					"/ego_floor", "ball"), locPhiM(&localizationMarker,
					"/ego_floor"), locM(&localizationMarker, "/ego_floor"), edgeImg_pub(
					"/vision/edgeImg"), guiImg_pub("/vision/guiImg"), webImg_pub(
					"/vision/webImg"), fieldDetector(), ballDetector(), lineDetector(), circleDetector(), goalDetector(), obstacleDetector(), loc(
					&obstacles), hsvPresenter(), guiManager(&ProjectionObj), plotM(
					PM_COUNT, "/VisionModule")
	{

		char const* robotName;
		robotName = getenv("VIS_HOSTNAME");
		rName = "";
		if (robotName != NULL)
		{
			std::size_t found = string(robotName).find("xs");
			if (found != std::string::npos)
			{
				rName = robotName;
			}
			else
			{
				rName = "unknown";
			}
			ROS_INFO("Robot Name = %s", rName.c_str());
		}

		if (dummy == true)
		{
			cam = new CameraDummy();
		}
		else
		{
			cam = new Camera();
		}

		visionOutputs_pub = nodeHandle.advertise<vision_module::vision_outputs>(
				"/vision/outputs", 10);


		plotM.setName(PM_FRAME_RATE, "FPS");
		plotM.setName(PM_HEADING_OFFSET, "headingOffset");
		plotM.setName(PM_OBSTACLE_TIME, "obst_time");
		plotM.setName(PM_LINE_TIME, "line_time");
		plotM.setName(PM_BALL_TIME, "ball_time");
		plotM.setName(PM_GOAL_TIME, "goal_time");
		plotM.setName(PM_CIRCLE_TIME, "circle_time");
		plotM.setName(PM_LOCALIZATION_TIME, "loc_time");
		plotM.setName(PM_LAST_AVALIBLE_TF, "lastAvalibleTF");
		if (!plotM.checkNames())
		{
			ROS_WARN("Check checkNames function for plotM!");
		}
	}

	void plot_fps(double _in)
	{
		plotM.plotScalar(_in, PM_FRAME_RATE);
	}

	bool GenerateTopViewImg(ros::Time now);
	void UpdateRangeImage(ros::Time now);
	void Process(ros::Time capTime);
	virtual ~Vision()
	{
		DeInit();
	}
	bool update();
	bool Init();
	void DeInit()
	{
		if (!destroyed)
		{
			cam->DeInitCameraDevice();
			delete cam;
			destroyed = true;
		}
	}

};

