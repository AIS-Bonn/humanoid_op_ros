//Parameters.hpp
// Created on: Apr 20, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once

#include <config_server/parameter.h>
#include <string.h>
#include <opencv2/opencv.hpp>

#define COLORED_OBJECT_COUNT 4
#define IMGWIDTH 640
#define IMGHEIGHT 480
using namespace std;
using namespace cv;

template<class T>
class constParameter
{
private:

public:
	T data;
	constParameter(T in)
	{
		set(in);
	}
	;
	T get()
	{
		return data;
	}
	void set(T in)
	{
		data = in;
	}
};

class hsvRangeC
{
public:
	config_server::Parameter<int> *h0;
	config_server::Parameter<int> *h1;
	config_server::Parameter<int> *s0;
	config_server::Parameter<int> *s1;
	config_server::Parameter<int> *v0;
	config_server::Parameter<int> *v1;
	config_server::Parameter<int> *erode;
	config_server::Parameter<int> *dilate;
	config_server::Parameter<int> *erode2;
	config_server::Parameter<int> *dilate2;
	config_server::Parameter<int> *min_minLen;
	config_server::Parameter<int> *max_minLen;
	config_server::Parameter<int> *min_maxLen;
	config_server::Parameter<int> *max_maxLen;
	config_server::Parameter<int> *maxDistanceToOthers;
	config_server::Parameter<float> *lowPassGain;
	config_server::Parameter<bool> *active;
	config_server::Parameter<bool> *track;
	config_server::Parameter<bool> *calibHistogram;
	config_server::Parameter<float> *minHistogramDiff;
	config_server::Parameter<float> *maxAngle;
	config_server::Parameter<float> *minAngle;
	config_server::Parameter<float> *maxRatio;
	config_server::Parameter<float> *minRatio;
	config_server::Parameter<int> *MinLineLength;
	config_server::Parameter<int> *MaxOutField;
	config_server::Parameter<int> *MinOutHorPoly;
	config_server::Parameter<int> *MaxDistanceToHorLine;
	config_server::Parameter<int> *MaxDistanceToHorLine2;
	config_server::Parameter<int> *CountZeroPercent;
	config_server::Parameter<int> *AngleToMerge;
	config_server::Parameter<float> *DistanceToMerge;
	config_server::Parameter<float> *DistanceToMerge2;

	config_server::Parameter<float> *NearestDistance;
	config_server::Parameter<float> *FarestDistance;
	config_server::Parameter<int> *NearMinLen;
	config_server::Parameter<int> *NearMaxLen;
	config_server::Parameter<int> *FarMinLen;
	config_server::Parameter<int> *FarMaxLen;
	config_server::Parameter<int> *maxContourCount;
	config_server::Parameter<int> *minArea;
	config_server::Parameter<float> *BiggerRectHT;
	config_server::Parameter<float> *BiggerRectHB;
	config_server::Parameter<float> *BiggerRectW;

	config_server::Parameter<float> *histogramH;
	config_server::Parameter<float> *histogramS;
	config_server::Parameter<float> *histogramV;
	config_server::Parameter<float> *histogramH_C;
	config_server::Parameter<float> *histogramS_C;
	config_server::Parameter<float> *histogramV_C;

	config_server::Parameter<float> *approxPoly;
	config_server::Parameter<int> *dist2cascade;
	config_server::Parameter<int> *lineVoteDouble;
	config_server::Parameter<int> *lineVote;
	config_server::Parameter<int> *lineVoteColor;

	config_server::Parameter<int> *jumpMin;
	config_server::Parameter<int> *jumpMax;

};

class cameraC
{
public:
	config_server::Parameter<float> *aspW;
	constParameter<float> *aspH;
	config_server::Parameter<bool> *flipVer;
	config_server::Parameter<bool> *flipHor;
	config_server::Parameter<int> *devNumber;
	constParameter<int> *width;
	constParameter<int> *height;
	constParameter<int> *widthUnDistortion;
	constParameter<int> *heightUnDistortion;
	config_server::Parameter<float> *diagonalAngleView;
};

class topViewC
{
public:
	config_server::Parameter<float> *scale;
	config_server::Parameter<int> *width;
	config_server::Parameter<bool> *calibrateKinematic;
};

class vectorC
{
public:
	config_server::Parameter<float> *x;
	config_server::Parameter<float> *y;
	config_server::Parameter<float> *z;
};

class vectorCC
{
public:
	constParameter<float> *x;
	constParameter<float> *y;
	constParameter<float> *z;
};

class calibC
{
public:
	config_server::Parameter<string> *filePath;
};

class debugC
{
public:
	config_server::Parameter<bool> *showHorizonBox;
	config_server::Parameter<bool> *showCameraPoints;
	config_server::Parameter<bool> *showTopView;
	config_server::Parameter<bool> *showGoalVer;
	config_server::Parameter<bool> *showGoalHor;
	config_server::Parameter<bool> *showAllGoal;
	config_server::Parameter<bool> *showAllLine;
	config_server::Parameter<bool> *showCombinedLine;
	config_server::Parameter<bool> *showBall;
	config_server::Parameter<bool> *showFieldHull;
	config_server::Parameter<bool> *showAllField;
	config_server::Parameter<bool> *maskField;
	config_server::Parameter<bool> *maskBall;
	config_server::Parameter<bool> *maskGoal;
	config_server::Parameter<bool> *maskLine;
	config_server::Parameter<bool> *showBallD;
	config_server::Parameter<bool> *showLineD;
	config_server::Parameter<bool> *showGoalD;
	config_server::Parameter<bool> *showObstacleD;
	config_server::Parameter<int> *publishTime;
	config_server::Parameter<float> *timeToShift;
};

class CameraCalibratorC
{
public:
	vector<Point> clicked;
	vector<Point2f> rvizClicked;
	vector<Point3d> cameraLocation, opticalAngle;
	bool IsReady()
	{
		return (rvizClicked.size() == clicked.size()
				&& clicked.size() == opticalAngle.size()
				&& opticalAngle.size() == cameraLocation.size()
				&& rvizClicked.size() > 0);
	}
};

class ParametersC
{
public:
	CameraCalibratorC camCalibrator;
	hsvRangeC ball, field, line, goal,obstacle;
	cameraC camera;
	topViewC topView;
	vectorC location, orientation;
	config_server::Parameter<int> *cannyThreadshold;
	calibC calib;
	debugC debug;
	void Init()
	{
		ball.h0 = new config_server::Parameter<int>("/vision/ball/h0", 0, 1,
				180, 0);
		ball.h1 = new config_server::Parameter<int>("/vision/ball/h1", 0, 1,
				180, 0);
		ball.s0 = new config_server::Parameter<int>("/vision/ball/s0", 0, 1,
				255, 0);
		ball.s1 = new config_server::Parameter<int>("/vision/ball/s1", 0, 1,
				255, 0);
		ball.v0 = new config_server::Parameter<int>("/vision/ball/v0", 0, 1,
				255, 0);
		ball.v1 = new config_server::Parameter<int>("/vision/ball/v1", 0, 1,
				255, 0);
		ball.erode = new config_server::Parameter<int>("/vision/ball/erode", 0,
				1, 20, 1);
		ball.dilate = new config_server::Parameter<int>("/vision/ball/dilate",
				0, 1, 20, 1);

		ball.maxDistanceToOthers = new config_server::Parameter<int>(
				"/vision/ball/maxDisToOthers", 0, 1, IMGWIDTH, 30);
		ball.lowPassGain = new config_server::Parameter<float>(
				"/vision/ball/lowPassGain", 0, 0.05, 1, 0.1);
		ball.active = new config_server::Parameter<bool>("/vision/ball/active",
				false);
		ball.track = new config_server::Parameter<bool>("/vision/ball/track",
				false);
		ball.calibHistogram = new config_server::Parameter<bool>(
				"/vision/ball/calibHistogram", false);
		ball.minHistogramDiff = new config_server::Parameter<float>(
				"/vision/ball/minHistogramDiff", 0, 0.05, 1, 0.6);

		ball.maxRatio = new config_server::Parameter<float>(
				"/vision/ball/maxRatio", 1, 0.1, 100, 100);
		ball.minRatio = new config_server::Parameter<float>(
				"/vision/ball/minRatio", 1, 0.1, 100, 1);


		ball.NearestDistance = new config_server::Parameter<float>(
				"/vision/ball/NearestDistance",0, 0.1, 10.81, 0);

		ball.FarestDistance = new config_server::Parameter<float>(
				"/vision/ball/FarestDistance", 0, 0.1,10.81 , 10.81);

		ball.NearMinLen = new config_server::Parameter<int>(
				"/vision/ball/NearMinLen", 0, 1,IMGWIDTH , 0);
		ball.NearMaxLen = new config_server::Parameter<int>(
				"/vision/ball/NearMaxLen", 0, 1,IMGWIDTH , IMGWIDTH);
		ball.FarMinLen = new config_server::Parameter<int>(
				"/vision/ball/FarMinLen", 0, 1,IMGWIDTH , 0);
		ball.FarMaxLen = new config_server::Parameter<int>(
				"/vision/ball/FarMaxLen", 0, 1,IMGWIDTH , IMGWIDTH);


		ball.BiggerRectHT = new config_server::Parameter<float>(
				"/vision/ball/BiggerRectHT", 0, 0.1,2, 0.2);
		ball.BiggerRectHB = new config_server::Parameter<float>(
				"/vision/ball/BiggerRectHB", 0, 0.1,2, 0.5);
		ball.BiggerRectW = new config_server::Parameter<float>(
				"/vision/ball/BiggerRectW", 0, 0.1,2, 0.5);

		ball.histogramH = new config_server::Parameter<float>(
				"/vision/ball/histogramH", 0, 0.1,1, 0.5);
		ball.histogramS = new config_server::Parameter<float>(
				"/vision/ball/histogramS", 0, 0.1,1, 0.5);
		ball.histogramV = new config_server::Parameter<float>(
				"/vision/ball/histogramV", 0, 0.1,1, 0.5);

		ball.histogramH_C = new config_server::Parameter<float>(
				"/vision/ball/histogramH_C", 0, 0.1,1, 0.7);
		ball.histogramS_C = new config_server::Parameter<float>(
				"/vision/ball/histogramS_C", 0, 0.1,1, 0.7);
		ball.histogramV_C = new config_server::Parameter<float>(
				"/vision/ball/histogramV_C", 0, 0.1,1, 0.7);



		ball.approxPoly = new config_server::Parameter<float>(
				"/vision/ball/approxPoly", 0, 0.005,1, 0.02);


		ball.dist2cascade = new config_server::Parameter<int>(
				"/vision/ball/dist2cascade", 0, 1,IMGWIDTH , 220);



		goal.h0 = new config_server::Parameter<int>("/vision/goal/h0", 0, 1,
				180, 0);
		goal.h1 = new config_server::Parameter<int>("/vision/goal/h1", 0, 1,
				180, 0);
		goal.s0 = new config_server::Parameter<int>("/vision/goal/s0", 0, 1,
				255, 0);
		goal.s1 = new config_server::Parameter<int>("/vision/goal/s1", 0, 1,
				255, 0);
		goal.v0 = new config_server::Parameter<int>("/vision/goal/v0", 0, 1,
				255, 0);
		goal.v1 = new config_server::Parameter<int>("/vision/goal/v1", 0, 1,
				255, 0);
		goal.erode = new config_server::Parameter<int>("/vision/goal/erode", 0,
				1, 20, 1);
		goal.dilate = new config_server::Parameter<int>("/vision/goal/dilate",
				0, 1, 20, 1);
		goal.active = new config_server::Parameter<bool>("/vision/goal/active",
				true);
		goal.MinLineLength = new config_server::Parameter<int>(
				"/vision/goal/MinLineLength", 40, 1, IMGWIDTH, 50);
		goal.MaxOutField = new config_server::Parameter<int>(
				"/vision/goal/MaxOutField", -40, 1, 40, 0);
		goal.MinOutHorPoly = new config_server::Parameter<int>(
				"/vision/goal/MinOutHorPoly", -IMGWIDTH, 1, 5, -5);
		goal.MaxDistanceToHorLine = new config_server::Parameter<int>(
				"/vision/goal/MaxDistanceToHorLine", 0, 1, IMGHEIGHT, 30);
		goal.MaxDistanceToHorLine2 = new config_server::Parameter<int>(
				"/vision/goal/MaxDistanceToHorLine2", 0, 1, IMGHEIGHT, 4);
		goal.CountZeroPercent = new config_server::Parameter<int>(
				"/vision/goal/CountZeroPercent", 0, 1, 100, 35);
		goal.DistanceToMerge = new config_server::Parameter<float>(
				"/vision/goal/DistanceToMerge", 0, 1, 100, 15);
		goal.DistanceToMerge2 = new config_server::Parameter<float>(
				"/vision/goal/DistanceToMerge2", 0, 1, 100, 15);

		goal.NearestDistance = new config_server::Parameter<float>(
				"/vision/goal/NearestDistance",0, 0.1, 10.81, 0);

		goal.FarestDistance = new config_server::Parameter<float>(
				"/vision/goal/FarestDistance", 0, 0.1,10.81 , 10.81);

		goal.NearMinLen = new config_server::Parameter<int>(
				"/vision/goal/NearMinLen", 0, 1,IMGWIDTH , 0);
		goal.NearMaxLen = new config_server::Parameter<int>(
				"/vision/goal/NearMaxLen", 0, 1,IMGWIDTH , IMGWIDTH);
		goal.FarMinLen = new config_server::Parameter<int>(
				"/vision/goal/FarMinLen", 0, 1,IMGWIDTH , 0);
		goal.FarMaxLen = new config_server::Parameter<int>(
				"/vision/goal/FarMaxLen", 0, 1,IMGWIDTH , IMGWIDTH);

		goal.BiggerRectHT = new config_server::Parameter<float>(
				"/vision/goal/BiggerRectHT", 0, 0.1,2, 0.2);
		goal.BiggerRectHB = new config_server::Parameter<float>(
				"/vision/goal/BiggerRectHB", 0, 0.1,2, 0.5);
		goal.BiggerRectW = new config_server::Parameter<float>(
				"/vision/goal/BiggerRectW", 0, 0.1,2, 0.5);


		obstacle.h0 = new config_server::Parameter<int>("/vision/obstacle/h0", 0, 1,
				180, 0);
		obstacle.h1 = new config_server::Parameter<int>("/vision/obstacle/h1", 0, 1,
				180, 0);
		obstacle.s0 = new config_server::Parameter<int>("/vision/obstacle/s0", 0, 1,
				255, 0);
		obstacle.s1 = new config_server::Parameter<int>("/vision/obstacle/s1", 0, 1,
				255, 0);
		obstacle.v0 = new config_server::Parameter<int>("/vision/obstacle/v0", 0, 1,
				255, 0);
		obstacle.v1 = new config_server::Parameter<int>("/vision/obstacle/v1", 0, 1,
				255, 0);
		obstacle.erode = new config_server::Parameter<int>("/vision/obstacle/erode", 0,
				1, 20, 1);
		obstacle.dilate = new config_server::Parameter<int>("/vision/obstacle/dilate",
				0, 1, 20, 1);


		line.h0 = new config_server::Parameter<int>("/vision/line/h0", 0, 1,
				180, 0);
		line.h1 = new config_server::Parameter<int>("/vision/line/h1", 0, 1,
				180, 0);
		line.s0 = new config_server::Parameter<int>("/vision/line/s0", 0, 1,
				255, 0);
		line.s1 = new config_server::Parameter<int>("/vision/line/s1", 0, 1,
				255, 0);
		line.v0 = new config_server::Parameter<int>("/vision/line/v0", 0, 1,
				255, 0);
		line.v1 = new config_server::Parameter<int>("/vision/line/v1", 0, 1,
				255, 0);
		line.erode = new config_server::Parameter<int>("/vision/line/erode", 0,
				1, 20, 1);
		line.dilate = new config_server::Parameter<int>("/vision/line/dilate",
				0, 1, 20, 1);
		line.active = new config_server::Parameter<bool>("/vision/line/active",
				false);
		line.MinLineLength = new config_server::Parameter<int>(
				"/vision/line/MinLineLength", 10, 1, 1000, 100);

		line.AngleToMerge = new config_server::Parameter<int>(
				"/vision/line/AngleToMerge", 0, 1, 90, 10);
		line.DistanceToMerge = new config_server::Parameter<float>(
				"/vision/line/DistanceToMerge", 0, 0.05, 1, 0.1);

		line.jumpMin = new config_server::Parameter<int>(
				"/vision/line/jumpMin", 0, 1, 64, 2);
		line.jumpMax = new config_server::Parameter<int>(
				"/vision/line/jumpMax", 0, 1, 64, 20);

		line.lineVoteDouble = new config_server::Parameter<int>(
						"/vision/line/lineVoteDouble", 0, 1,64 , 12);

		line.lineVote = new config_server::Parameter<int>(
						"/vision/line/lineVote", 0, 1,64 , 7);

				line.lineVoteColor = new config_server::Parameter<int>(
						"/vision/line/lineVoteColor", 0, 1,64 , 12);

		field.h0 = new config_server::Parameter<int>("/vision/field/h0", 0, 1,
				180, 0);
		field.h1 = new config_server::Parameter<int>("/vision/field/h1", 0, 1,
				180, 0);
		field.s0 = new config_server::Parameter<int>("/vision/field/s0", 0, 1,
				255, 0);
		field.s1 = new config_server::Parameter<int>("/vision/field/s1", 0, 1,
				255, 0);
		field.v0 = new config_server::Parameter<int>("/vision/field/v0", 0, 1,
				255, 0);
		field.v1 = new config_server::Parameter<int>("/vision/field/v1", 0, 1,
				255, 0);
		field.erode = new config_server::Parameter<int>("/vision/field/erode",
				0, 1, 20, 1);
		field.dilate = new config_server::Parameter<int>("/vision/field/dilate",
				0, 1, 20, 1);
		field.erode2 = new config_server::Parameter<int>("/vision/field/erode2",
				0, 1, 100, 15);
		field.dilate2 = new config_server::Parameter<int>("/vision/field/dilate2",
				0, 1, 100, 0);
		field.maxContourCount = new config_server::Parameter<int>("/vision/field/maxContourCount",
				1, 1, 10, 2);
		field.minArea = new config_server::Parameter<int>("/vision/field/minArea",
				0, 1, IMGWIDTH*IMGHEIGHT, 0);
		field.active = new config_server::Parameter<bool>(
				"/vision/field/active", false);

		camera.flipHor = new config_server::Parameter<bool>(
				"/vision/camera/flipHor", false);
		camera.flipVer = new config_server::Parameter<bool>(
				"/vision/camera/flipVer", false);
		camera.devNumber = new config_server::Parameter<int>(
				"/vision/camera/devNumber", 0, 1, 3, 0);

		camera.width = new constParameter<int>(IMGWIDTH);
		camera.height = new constParameter<int>(IMGHEIGHT);

		camera.widthUnDistortion = new constParameter<int>(0);
		camera.heightUnDistortion = new constParameter<int>(0);
		camera.diagonalAngleView = new config_server::Parameter<float>(
				"/vision/camera/diagonalAngleView", 1, 0.05, 180, 75);

		camera.aspW = new config_server::Parameter<float>("/vision/camera/aspW",
				0, 0.1, 40, 4.0);
		camera.aspH = new constParameter<float>(3.0);
		topView.scale = new config_server::Parameter<float>(
				"/vision/topView/scale", 1, 0.5, 10, 3);
		topView.width = new config_server::Parameter<int>(
				"/vision/topView/width", 200, 1, 10000, 900);
		topView.calibrateKinematic = new config_server::Parameter<bool>(
				"/vision/topView/calibrateKinematic", false);

		location.x = new config_server::Parameter<float>(
				"/vision_tmp/location/x", -10, 0.005, 10, 0);
		location.y = new config_server::Parameter<float>(
				"/vision_tmp/location/y", -10, 0.005, 10, 0);
		location.z = new config_server::Parameter<float>(
				"/vision_tmp/location/z", 0, 0.005, 5, 0.85);

		orientation.x = new config_server::Parameter<float>(
				"/vision_tmp/orientation/x", -M_PI, 0.001, M_PI, 0);
		orientation.y = new config_server::Parameter<float>(
				"/vision_tmp/orientation/y", -M_PI, 0.001, M_PI, 0);
		orientation.z = new config_server::Parameter<float>(
				"/vision_tmp/orientation/z", -M_PI, 0.001, M_PI, 0);

		cannyThreadshold = new config_server::Parameter<int>(
				"/vision_tmp/cannyThreadshold", 0, 1, 100, 40);
		calib.filePath = new config_server::Parameter<string>(
				"/vision/calib/filePath",
				"/nimbro/share/launch/config/vision/cCFile.yml");

		debug.publishTime = new config_server::Parameter<int>(
				"/vision/debug/publishTime", 1, 1, 300, 1);
		debug.timeToShift = new config_server::Parameter<float>(
				"/vision/debug/timeToShift", 0, 0.005, 3, 0.20);

		debug.showHorizonBox = new config_server::Parameter<bool>(
				"/vision/debug/showHorizonBox", false);
		debug.showCameraPoints = new config_server::Parameter<bool>(
				"/vision/debug/showCameraPoints", false);
		debug.showTopView = new config_server::Parameter<bool>(
				"/vision/debug/showTopView", false);

		debug.showGoalVer = new config_server::Parameter<bool>(
				"/vision/debug/showGoalVer", false);

		debug.showGoalHor = new config_server::Parameter<bool>(
				"/vision/debug/showGoalHor", false);
		debug.showAllGoal = new config_server::Parameter<bool>(
				"/vision/debug/showAllGoal", false);

		debug.showAllLine = new config_server::Parameter<bool>(
				"/vision/debug/showAllLine", false);

		debug.showBallD = new config_server::Parameter<bool>(
				"/vision/debug/showBallD", false);
		debug.showLineD = new config_server::Parameter<bool>(
				"/vision/debug/showLineD", false);
		debug.showObstacleD = new config_server::Parameter<bool>(
				"/vision/debug/showObstacleD", false);
		debug.showGoalD = new config_server::Parameter<bool>(
				"/vision/debug/showGoalD", false);

		debug.showCombinedLine = new config_server::Parameter<bool>(
				"/vision/debug/showCombinedLine", false);

		debug.showBall = new config_server::Parameter<bool>(
				"/vision/debug/showBall", false);

		debug.showFieldHull = new config_server::Parameter<bool>(
				"/vision/debug/showFieldHull", false);
		debug.showAllField = new config_server::Parameter<bool>(
				"/vision/debug/showAllField", false);
		debug.maskField = new config_server::Parameter<bool>(
				"/vision/debug/maskField", false);
		debug.maskBall = new config_server::Parameter<bool>(
				"/vision/debug/maskBall", false);
		debug.maskGoal = new config_server::Parameter<bool>(
				"/vision/debug/maskGoal", false);
		debug.maskLine = new config_server::Parameter<bool>(
				"/vision/debug/maskLine", false);

	}
};

extern ParametersC params;
