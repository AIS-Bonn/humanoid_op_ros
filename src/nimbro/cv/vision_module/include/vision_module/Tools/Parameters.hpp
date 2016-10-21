//Parameters.hpp
// Created on: Apr 20, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once

#include <config_server/parameter.h>
#include <config_server/ParameterLoadTime.h>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Time.h>
#define IMGWIDTH 640
#define IMGHEIGHT 480
#define MAXIMGDIST 800
#define MAXCONSIDERDISTANCE 10.81
#define IMGBOX cv::Rect(0,0,IMGWIDTH,IMGHEIGHT)
#define IMGPYRDOWNBOX cv::Rect(0,0,320,240)
#define COLORED_OBJECT_COUNT 6

using namespace std;
using namespace cv;

#define GETS(x)  #x

#define CSTR(x) prefix + GETS(x)

#define MP(x,...) x(CSTR(x),__VA_ARGS__)

#define MPT(x,y,...) y(prefix+#x+#y,__VA_ARGS__)

typedef config_server::Parameter<bool> pbool;
typedef config_server::Parameter<int> pint;
typedef config_server::Parameter<float> pfloat;
typedef config_server::Parameter<string> pstring;

class GroupParameter
{
	vector<bool> lastDataState;
	vector<config_server::Parameter<bool>*> data;
public:

	GroupParameter()
	{

	}

	void add(config_server::Parameter<bool>* _in)
	{
		data.push_back(_in);
		lastDataState.resize(data.size());
		for (size_t i = 0; i < data.size(); i++)
		{
			lastDataState[i] = data[i]->get();
		}
	}

	void update()
	{
		int firstChange2True = -1;
		int numberOf2TrueChange = 0;
		for (size_t i = 0; i < data.size(); i++)
		{
			if (data[i]->get() && !lastDataState[i])
			{
				numberOf2TrueChange++;
				if (firstChange2True == -1)
				{
					firstChange2True = i;
				}
			}
		}

		if (numberOf2TrueChange >= 1)
		{
			for (size_t i = 0; i < data.size(); i++)
			{
				if ((int) i == firstChange2True)
				{
					data[i]->set(true);
				}
				else
				{
					data[i]->set(false);
				}
			}
		}

		for (size_t i = 0; i < data.size(); i++)
		{
			lastDataState[i] = data[i]->get();
		}
	}
};

template<class T>
class localParameter
{
private:

public:
	T data;
	localParameter(T in)
	{
		set(in);
	}
	inline T get() const
	{
		return data;
	}
	inline T operator()() const
	{
		return get();
	}

	void set(T in)
	{
		data = in;
	}
};
/**
* @ingroup VisionModule
*
* @brief HSV range class
**/
class hsvRangeC
{
public:
	pbool *active;
	pint*h0;
	pint*h1;
	pint *s0;
	pint *s1;
	pint *v0;
	pint *v1;
};

#define GETHSVSTRUCT()                          \
hsvRangeC GetHSVRange()                         \
{                                               \
    hsvRangeC hsvRes;                           \
    hsvRes.h0=&h0;                              \
    hsvRes.h1=&h1;                              \
    hsvRes.s0=&s0;                              \
    hsvRes.s1=&s1;                              \
    hsvRes.v0=&v0;                              \
    hsvRes.v1=&v1;                              \
    hsvRes.active=&active;                      \
    return hsvRes;                              \
}
/**
* @ingroup VisionModule
*
* @brief Parameters container class for field detection
**/
class fieldParamC
{
public:
	pbool enable;
	pbool showMask;
	pbool showResult;
	pbool showDebug;
	pint h0;
	pint h1;
	pint s0;
	pint s1;
	pint v0;
	pint v1;
	pbool active;
	pint erode;
	pint dilate;
	pint erode2;
	pint dilate2;
	pint maxContourCount;
	pint minArea;
	pint maxDownDiffPixel;
	pfloat approxPoly;
	pfloat maxAcceptDistance;
	pfloat minAcceptX;
	fieldParamC(string prefix) :
			enable(CSTR(enable), true), showMask(CSTR(showMask), false), showResult(
					CSTR(showResult), false), showDebug(CSTR(showDebug), false), h0(
					CSTR(hsv/h0), 0, 1, 180, 0), h1(CSTR(hsv/h1), 0, 1, 180, 0), s0(
					CSTR(hsv/s0), 0, 1, 255, 0), s1(CSTR(hsv/s1), 0, 1, 255, 0), v0(
					CSTR(hsv/v0), 0, 1, 255, 0), v1(CSTR(hsv/v1), 0, 1, 255, 0), active(
					CSTR(hsv/active), false), erode(CSTR(morph/erode), 0, 1, 20,
					1), dilate(CSTR(morph/dilate), 0, 1, 20, 1), erode2(
					CSTR(morph/erode2), 0, 1, 100, 15), dilate2(
					CSTR(morph/dilate2), 0, 1, 100, 0), maxContourCount(
					CSTR(shape/maxContourCount), 1, 1, 10, 2), minArea(
					CSTR(shape/minArea), 0, 1, IMGWIDTH * IMGHEIGHT, 0),MPT(shape/,maxDownDiffPixel,0,1,IMGHEIGHT,200), approxPoly(
					CSTR(real/approxPoly), 0, 0.005, 1, 0.02),MPT(shape/,maxAcceptDistance,0,0.1,45,10.81),MPT(shape/,minAcceptX,-10.81,0.1,0,-1)
	{

	}

	GETHSVSTRUCT()

};
/**
* @ingroup VisionModule
*
* @brief Parameters container class for ball detection
**/
class ballParamC
{
public:
	Point debugThisPos;
	bool debugThis;
	vector<Point3d> distSizeVec;
	string distSizeVecPath;
	vector<Mat> histList[3];
	string histListcPath[3];
	localParameter<float> radius;
	pbool enable;
	pbool enableHog;
	pbool showMask;
	pbool showResult;
	pbool showInput;
	pbool showAllCircle;
	pbool showCandidate;
	pbool showBoxs;
	pint h0;
	pint h1;
	pint s0;
	pint s1;
	pint v0;
	pint v1;
	pbool active;
	pfloat histogramH;
	pfloat histogramS;
	pfloat histogramV;
	pint erode;
	pint dilate;
	pint erode2;
	pint dilate2;
	pfloat BiggerRectHT;
	pfloat BiggerRectHB;
	pfloat BiggerRectW;
	pfloat histogramH_C;
	pfloat histogramS_C;
	pfloat histogramV_C;
	pint dist2cascade;
	pfloat houghDP;
	pint houghMaxDist;
	pint houghCanny;
	pint houghCircle;
	pint houghMinR;
	pint houghMaxR;
	pfloat houghCandWRatio;
	pfloat houghCandHTopRatio;
	pfloat houghCandHDownRatio;
	pbool pyrDown;
	pint whitePercent;
	pint notGreenPercent;
	pint btnGCheckPercent;
	pint kernelBlur;
	pint sigmaBlur;
	pbool subResultFromCanny;
	pfloat btnGCheckBoxtop;
	pfloat btnGCheckBoxdown;
	pfloat btnGCheckBoxW;
	pbool addSizeDistance;
	pbool popSizeDistance;
	pbool clearDistSizeVec;
	pbool addHistogram;
	pbool popHistogram;
	pbool clearHistogram;
	pbool savePositive;
	pbool saveNegative;
	pbool saveNegativeFull;
	pint rectRotationMax;
	pint rectRotationStep;
	pint rectResizeWidth;
	pint rectResizeHeight;
	pfloat cascadeScale;
	pint cascadeNinNeighbors;
	pbool loadCascadeFile;
	pbool printBallDetection;
	pbool mergeResult;
	ballParamC(string prefix) :
			radius(0.095), enable(CSTR(enable), true), enableHog(
					CSTR(cascade/enable), true), showMask(CSTR(showMask),
					false), showResult(CSTR(showResult), false), showInput(
					CSTR(showInput), false), showAllCircle(CSTR(showAllCircle),
					false), showCandidate(CSTR(showCandidate), false), showBoxs(
					CSTR(showBoxs), false), h0(CSTR(hsv/h0), 0, 1, 180, 0), h1(
					CSTR(hsv/h1), 0, 1, 180, 0), s0(CSTR(hsv/s0), 0, 1, 255, 0), s1(
					CSTR(hsv/s1), 0, 1, 255, 0), v0(CSTR(hsv/v0), 0, 1, 255, 0), v1(
					CSTR(hsv/v1), 0, 1, 255, 0), active(CSTR(hsv/active),
					false), histogramH(CSTR(hsv/histogramH), 0, 0.05, 1, 0.5), histogramS(
					CSTR(hsv/histogramS), 0, 0.05, 1, 0.5), histogramV(
					CSTR(hsv/histogramV), 0, 0.05, 1, 0.5), erode(
					CSTR(morph/erode), 0, 1, 20, 1), dilate(CSTR(morph/dilate),
					0, 1, 20, 1), erode2(CSTR(morph/erode2), 0, 1, 20, 0), dilate2(
					CSTR(morph/dilate2), 0, 1, 20, 0), BiggerRectHT(
					CSTR(cascade/BiggerRectHT), 0, 0.1, 5, 1.4), BiggerRectHB(
					CSTR(cascade/BiggerRectHB), 0, 0.1, 5, 2), BiggerRectW(
					CSTR(cascade/BiggerRectW), 0, 0.1, 5, 2), histogramH_C(
					CSTR(cascade/histogramH_C), 0, 0.05, 1, 0.7), histogramS_C(
					CSTR(cascade/histogramS_C), 0, 0.05, 1, 0.7), histogramV_C(
					CSTR(cascade/histogramV_C), 0, 0.05, 1, 0.7), dist2cascade(
					CSTR(cascade/dist2cascade), 0, 1, IMGWIDTH, 220), houghDP(
					CSTR(hough/DP), 0.1, 0.1, 10, 2), houghMaxDist(
					CSTR(hough/maxDist), 2, 1,
					IMGWIDTH, 100), houghCanny(CSTR(hough/canny), 1, 1, 255,
					200), houghCircle(CSTR(hough/circle), 1, 1, 255, 100), houghMinR(
					CSTR(hough/minR), 1, 1, IMGWIDTH, 1), houghMaxR(
					CSTR(hough/maxR), 1, 1, IMGWIDTH, 1), houghCandWRatio(
					CSTR(hough/houghCandWRatio), 1, 0.05, 5, 1), houghCandHTopRatio(
					CSTR(hough/houghCandHTopRatio), 1, 0.05, 2.5, 1), houghCandHDownRatio(
					CSTR(hough/houghCandHDownRatio), 1, 0.05, 2.5, 1), pyrDown(
					CSTR(hough/pyrDown), true), whitePercent(
					CSTR(reject/whitePercent), 0, 1, 100, 10), notGreenPercent(
					CSTR(reject/notGPercent), 0, 1, 100, 10), MPT(reject/,btnGCheckPercent,0,1,100,50), kernelBlur(
					CSTR(hough/kernelBlur), 0, 1, 75, 15), sigmaBlur(
					CSTR(hough/sigmaBlur), 0, 1, 150, 2), subResultFromCanny(
					CSTR(subResultFromCanny), true), MPT(reject/,btnGCheckBoxtop,0,0.1,1.5,0),
			MPT(reject/,btnGCheckBoxdown,0,0.1,1.5,1.3), MPT(reject/,btnGCheckBoxW,0,0.1,1.5,1.3),
			MPT(gui/,addSizeDistance, false), MPT(gui/,popSizeDistance, false), MPT(gui/,clearDistSizeVec, false), MPT(gui/,addHistogram,false),
			MPT(gui/,popHistogram, false), MPT(gui/,clearHistogram, false), MPT(gui/,savePositive, false), MPT(gui/,saveNegative, false), MPT(gui/,saveNegativeFull,false),
			MPT(gui/,rectRotationMax,0,1,45,20), MPT(gui/,rectRotationStep,1,1,20,10), MPT(gui/,rectResizeWidth,0,1,IMGWIDTH,0), MPT(gui/,rectResizeHeight,0,1,IMGHEIGHT,0),
			MPT(cascade/,cascadeScale,1,0.01,2,1.1), MPT(cascade/,cascadeNinNeighbors,0,1,20,10), MPT(cascade/,loadCascadeFile,false),MP(printBallDetection,false),MP(mergeResult,false)
	{
		distSizeVecPath = "ball_distSizeVec.yml";
		histListcPath[0] = "ballColorHist0.yml";
		histListcPath[1] = "ballColorHist1.yml";
		histListcPath[2] = "ballColorHist2.yml";
		debugThisPos.x = debugThisPos.y = -1;
		debugThis = false;
	}

	GETHSVSTRUCT()

};
/**
* @ingroup VisionModule
*
* @brief Parameters container class for line detection
**/
class lineParamC
{
public:
	pbool enable;
	pbool showMask;
	pbool showResult;
	pbool showAllLine;
	pbool showVote;
	pbool showCanny;
	pint h0;
	pint h1;
	pint s0;
	pint s1;
	pint v0;
	pint v1;
	pbool active;
	pint MinLineLength;
	pint AngleToMerge;
	pfloat DistanceToMerge;
	pint maxLineGapHough;
	pfloat rhoHough;
	pint thetaHough;
	pint threasholdHough;
	pint jumpMax;
	pint jumpMin;
	pfloat widthCheck;
	pbool aprxDist;
	pint doubleVote;
	pint greenVote;
	pint colorVote;
	pbool doubleVUse;
	pbool greenVUse;
	pbool colorVUse;
	pfloat doubleVStart;
	pfloat greenVStart;
	pfloat colorVStart;
	pfloat doubleVEnd;
	pfloat greenVEnd;
	pfloat colorVEnd;
	pint cannyThreadshold;
	pint blurSize;
	pint cannyaperture;

	lineParamC(string prefix) :
			enable(CSTR(enable), true), showMask(CSTR(showMask), false), showResult(
					CSTR(showResult), false), showAllLine(CSTR(showAllLine),
					false), showVote(CSTR(showVote), false), showCanny(
					CSTR(showCanny), false), h0(CSTR(hsv/h0), 0, 1, 180, 0), h1(
					CSTR(hsv/h1), 0, 1, 180, 0), s0(CSTR(hsv/s0), 0, 1, 255, 0), s1(
					CSTR(hsv/s1), 0, 1, 255, 0), v0(CSTR(hsv/v0), 0, 1, 255, 0), v1(
					CSTR(hsv/v1), 0, 1, 255, 0), active(CSTR(hsv/active),
					false), MinLineLength(CSTR(hough/MinLineLength), 10, 1,
					1000, 100), AngleToMerge(CSTR(shape/AngleToMerge), 0, 1, 90,
					10), DistanceToMerge(CSTR(shape/DistanceToMerge), 0, 0.05,
					1, 0.1), maxLineGapHough(CSTR(hough/maxLineGapHough), 0, 1,
					150, 20), rhoHough(CSTR(hough/rhoHough), 0.01, 0.01, 2, 1), thetaHough(
					CSTR(hough/thetaHough), 1, 1, 180, 45), threasholdHough(
					CSTR(hough/threasholdHough), 1, 1, 255, 20), jumpMax(
					CSTR(vote/jumpMax), 0, 1, 150, 20), jumpMin(
					CSTR(vote/jumpMin), 1, 1, 20, 3), widthCheck(
					CSTR(vote/widthCheck), 0.01, 0.01, 1, 0.07), aprxDist(
					CSTR(vote/aprxDist), true), doubleVote(
					CSTR(vote/doubleVote), 0, 1, 100, 60), greenVote(
					CSTR(vote/greenVote), 0, 1, 100, 60), colorVote(
					CSTR(vote/colorVote), 0, 1, 100, 60), doubleVUse(
					CSTR(vote/doubleVoteUse), true), greenVUse(
					CSTR(vote/greenVoteUse), true), colorVUse(
					CSTR(vote/colorVoteUse), true), doubleVStart(
					CSTR(vote/doubleVStart), 0, 0.1, 1, 0), greenVStart(
					CSTR(vote/greenVStart), 0, 0.1, 1, 0), colorVStart(
					CSTR(vote/colorVStart), 0, 0.1, 1, 0), doubleVEnd(
					CSTR(vote/doubleVEnd), 0, 0.1, 1, 1), greenVEnd(
					CSTR(vote/greenVEnd), 0, 0.1, 1, 1), colorVEnd(
					CSTR(vote/colorVEnd), 0, 0.1, 1, 1), MPT(canny/,cannyThreadshold, 1, 1, 100, 23), MPT(canny/,blurSize, 1, 1, 20, 3), MPT(canny/,cannyaperture
					, 1, 1, 20, 3)
	{

	}

	GETHSVSTRUCT()

};
/**
* @ingroup VisionModule
*
* @brief Parameters container class for goal detection
**/
class goalParamC
{
public:
	vector<Point3d> distSizeVec;
	string distSizeVecPath;
	pbool enable;
	pbool showMask;
	pbool showResult;
	pbool showAllLines;
	pbool showResLine;
	pbool showVote;
	pint h0;
	pint h1;
	pint s0;
	pint s1;
	pint v0;
	pint v1;
	pbool active;
	pint MinLineLength;
	pint MaxOutField;
	pfloat DistanceToMerge;
	pfloat NearestDistance;
	pfloat FarestDistance;
	pint NearMinLen;
	pint NearMaxLen;
	pint FarMinLen;
	pint FarMaxLen;
	pint jumpMax;
	pint doubleVote;
	pint minDoubleLength;
	pint minContinuesColor;

	pbool addSizeDistance;
	pbool popSizeDistance;
	pbool clearDistSizeVec;
	goalParamC(string prefix) :
			enable(CSTR(enable), true), showMask(CSTR(showMask), false), showResult(
					CSTR(showResult), false), showAllLines(CSTR(showAllLines),
					false), showResLine(CSTR(showResLine), false), showVote(
					CSTR(showVote), false), h0(CSTR(hsv/h0), 0, 1, 180, 0), h1(
					CSTR(hsv/h1), 0, 1, 180, 0), s0(CSTR(hsv/s0), 0, 1, 255, 0), s1(
					CSTR(hsv/s1), 0, 1, 255, 0), v0(CSTR(hsv/v0), 0, 1, 255, 0), v1(
					CSTR(hsv/v1), 0, 1, 255, 0), active(CSTR(hsv/active), true), MinLineLength(
					CSTR(shape/MinLineLength), 40, 1, IMGWIDTH, 50), MaxOutField(
					CSTR(shape/MaxOutField), -40, 1, 40, 0), DistanceToMerge(
					CSTR(shape/DistanceToMerge), 0, 1, 100, 15), NearestDistance(
					CSTR(shape/NearestDistance), 0, 0.1, 10.81, 0), FarestDistance(
					CSTR(shape/FarestDistance), 0, 0.1, 10.81, 10.81), NearMinLen(
					CSTR(shape/NearMinLen), 0, 1, IMGWIDTH, 0), NearMaxLen(
					CSTR(shape/NearMaxLen), 0, 1, IMGWIDTH, IMGWIDTH), FarMinLen(
					CSTR(shape/FarMinLen), 0, 1, IMGWIDTH, 0), FarMaxLen(
					CSTR(shape/FarMaxLen), 0, 1, IMGWIDTH, IMGWIDTH), jumpMax(
					CSTR(vote/JumpDouble), 0, 1, 64, 2), doubleVote(
					CSTR(vote/VoteDouble), 0, 1, 100, 40), minDoubleLength(
					CSTR(vote/minDoubleLength), 0, 1, 480, 3), minContinuesColor(
					CSTR(vote/minContinuesColor), 0, 1, 480, 7), MPT(gui/,addSizeDistance,false), MPT(gui/,popSizeDistance,false), MPT(gui/,clearDistSizeVec,false)
	{
		distSizeVecPath = "goal_distSizeVec.yml";
	}

	GETHSVSTRUCT()

};
/**
* @ingroup VisionModule
*
* @brief Parameters container class for obstacle detection
**/
class obstacleParamC
{
public:
	pbool enable;
	pbool mask;
	pbool showResult;
	pint h0;
	pint h1;
	pint s0;
	pint s1;
	pint v0;
	pint v1;
	pbool active;
	pint erode;
	pint dilate;
	pint minArea;
	pfloat decayConfidence;
	pfloat maxPossibleJump;
	pfloat lowPassCoef;
	pfloat minDistance;
	pfloat maxDistance;

	obstacleParamC(string prefix) :
			enable(CSTR(enable), true), mask(CSTR(showMask), false), showResult(
					CSTR(showResult), false), h0(CSTR(hsv/h0), 0, 1, 180, 0), h1(
					CSTR(hsv/h1), 0, 1, 180, 0), s0(CSTR(hsv/s0), 0, 1, 255, 0), s1(
					CSTR(hsv/s1), 0, 1, 255, 0), v0(CSTR(hsv/v0), 0, 1, 255, 0), v1(
					CSTR(hsv/v1), 0, 1, 255, 0), active(CSTR(hsv/active),
					false), erode(CSTR(morph/erode), 0, 1, 20, 1), dilate(
					CSTR(morph/dilate), 0, 1, 20, 1), minArea(
					CSTR(shape/minArea), 0, 1,
					IMGWIDTH * IMGHEIGHT, 500),MPT(model/,decayConfidence,0,0.001,1,0.99),MPT(model/,maxPossibleJump,0,0.01,10,0.4),
					MPT(model/,lowPassCoef,0,0.01,1,0.9),MPT(shape/,minDistance,0,0.01,10.81,0),MPT(shape/,maxDistance,0,0.01,10.81,10.81)

	{

	}

	GETHSVSTRUCT()

};
/**
* @ingroup VisionModule
*
* @brief Parameters container class for igus color
**/
class igusParamC
{
public:
	pbool active;
	pint h0;
	pint h1;
	pint s0;
	pint s1;
	pint v0;
	pint v1;

	igusParamC(string prefix) :
			active(CSTR(hsv/active), false), h0(CSTR(hsv/h0), 0, 1, 180, 0), h1(
					CSTR(hsv/h1), 0, 1, 180, 0), s0(CSTR(hsv/s0), 0, 1, 255, 0), s1(
					CSTR(hsv/s1), 0, 1, 255, 0), v0(CSTR(hsv/v0), 0, 1, 255, 0), v1(
					CSTR(hsv/v1), 0, 1, 255, 0)

	{

	}

	GETHSVSTRUCT()

};
/**
* @ingroup VisionModule
*
* @brief Parameters container class for camera
**/
class cameraC
{
public:
	pbool flipHor;
	pbool flipVer;
	localParameter<int> width;
	localParameter<int> height;
	localParameter<int> widthUnDistortion;
	localParameter<int> heightUnDistortion;
	pfloat diagonalAngleView;
	pfloat aspW;
	localParameter<float> aspH;
	pbool freezInput;
	cameraC(string prefix) :
			flipHor(CSTR(flipHor), false), flipVer(CSTR(flipVer), false), width(
			IMGWIDTH), height(
			IMGHEIGHT), widthUnDistortion(0), heightUnDistortion(0), diagonalAngleView(
					CSTR(diagonalAngleView), 1, 0.05, 180, 75), aspW(CSTR(aspW),
					0, 0.1, 40, 4.0), aspH(3.0),MP(freezInput,false)
	{
		freezInput.set(false);
	}
};
/**
* @ingroup VisionModule
*
* @brief Parameters container class for top view
**/
class topViewC
{
public:
	pfloat scale;
	pint width;

	topViewC(string prefix) :
			scale(CSTR(scale), 1, 0.5, 10, 3), width(CSTR(width), 200, 1, 10000,
					900)
	{

	}
};
/**
* @ingroup VisionModule
*
* @brief Parameters container class for camera calibration
**/
class calibC
{
public:
	pstring filePath;
	pint boardWidth;
	pint boardHeight;
	pint squareSize;
	pint delay;
	pint frameCount;
	pbool high_dimension;

	calibC(string prefix) :
			filePath(CSTR(filePath), "cCFile.yml"), MP(boardWidth, 1, 1, 15, 7), MP(boardHeight, 1, 1, 15, 10), MP(squareSize
					, 1, 1, 200, 58), MP(delay, 1, 1, 3000,
					200), MP(frameCount, 0, 1, 200, 60), MP(high_dimension,true)

	{

	}
};
/**
* @ingroup VisionModule
*
* @brief Parameters container class for debuging purposes
**/
class debugC
{
public:
	pint publishTime;
	pint webpublishTime;
	pbool showHorizonBox;
	pbool showCameraPoints;
	pbool showTopView;
	pbool showPCL;

	debugC(string prefix) :
			publishTime(CSTR(publishTime), 1, 1, 300, 1), webpublishTime(
					CSTR(webpublishTime), 1, 1, 300, 7), showHorizonBox(
					CSTR(showHorizonBox), false), showCameraPoints(
					CSTR(showCameraPoints), false), showTopView(
					CSTR(showTopView), false),MP(showPCL,false)
	{

	}
};
/**
* @ingroup VisionModule
*
* @brief Parameters container class for circle detection
**/
class circleC
{
public:
	pbool enable;
	pfloat minLineLen;
	pfloat maxLineLen;
	pfloat maxDistBetween2LS;
	pfloat radiusMaxCoef;
	pfloat radiusMinCoef;
	pfloat confiusedDist;
	pint minLineSegmentCount;

	circleC(string prefix) :
			enable(CSTR(enable), true), minLineLen(CSTR(minLineLen), 0, 0.01, 2,
					0.2), maxLineLen(CSTR(maxLineLen), 0, 0.01, 3, 1.5), maxDistBetween2LS(
					CSTR(maxDistBetween2LS), 0, 0.01, 2, 0.3), radiusMaxCoef(
					CSTR(Coef/radiusMaxCoef), 0, 0.01, 3, 1.5), radiusMinCoef(
					CSTR(Coef/radiusMinCoef), 0, 0.01, 2, 0.7), confiusedDist(
					CSTR(confiusedDist), 0, 0.01, 3, 1), minLineSegmentCount(
					CSTR(minLineSegmentCount), 0, 1, 10, 3)
	{

	}
};
/**
* @ingroup VisionModule
*
* @brief Parameters container class for localization
**/
class localizationC
{
public:
	pbool enable;
	pfloat minLineLen;
	pfloat UPDATENORMAL;
	pfloat UPDATESTRONG;
	pfloat UPDATEWEAK;
	pfloat TOTALGAIN;
	pfloat VerLineMinDistanceToUpdate;
	pbool useKalman;
	pbool forwardRobotTrackerXY;
	pbool forwardRobotTrackerZ;

	localizationC(string prefix) :
			enable(CSTR(enable), true), minLineLen(
					CSTR(minLineLen), 0, 0.01, 5, 1.5), UPDATENORMAL(
					CSTR(coef/UPDATENORMAL), 0, 0.01, 1, 0.1), UPDATESTRONG(
					CSTR(coef/UPDATESTRONG), 0, 0.01, 1, 0.2), UPDATEWEAK(
					CSTR(coef/UPDATEWEAK), 0, 0.01, 1, 0.05), TOTALGAIN(
					CSTR(coef/TOTALGAIN), 0, 0.01, 1, 1), VerLineMinDistanceToUpdate(
					CSTR(coefs/VerLineMinDistanceToUpdate), 0, 0.01, 2, 0.7), useKalman(
					CSTR(useKalman), false), forwardRobotTrackerXY(
					CSTR(forwardRobotTrackerXY), false), forwardRobotTrackerZ(
					CSTR(forwardRobotTrackerZ), false)
	{

	}
};
/**
* @ingroup VisionModule
*
* @brief Parameters container class for tf calibration
**/
class tfC
{
public:
	pfloat locationX;
	pfloat locationY;
	pfloat locationZ;
	pfloat robotX;
	pfloat robotY;
	pfloat robotZ;
	pfloat orientationX;
	pfloat orientationY;
	pfloat orientationZ;
	pbool calibrateKinematicHill;
	pbool calibrateKinematicSimplex;
	pbool precalculateProjection;
	pfloat timeToShift;
	pint maxIteration;

	tfC(string prefix) :
			locationX(CSTR(location/x), -2, 0.005, 2, 0), locationY(
					CSTR(location/y), -2, 0.005, 2, 0), locationZ(
					CSTR(location/z), -2, 0.005, 2, 0), robotX(CSTR(robot/x),
					-10, 0.01, 10, 0), robotY(CSTR(robot/y), -10, 0.01, 10, 0), robotZ(
					CSTR(robot/z), -M_PI, 0.001,
					M_PI, 0), orientationX(CSTR(orientation/x), -M_PI, 0.001,
			M_PI, 0), orientationY(CSTR(orientation/y), -M_PI, 0.001,
			M_PI, 0), orientationZ(CSTR(orientation/z), -M_PI, 0.001,
			M_PI, 0), calibrateKinematicHill(CSTR(kinCalibHill), false), calibrateKinematicSimplex(
					CSTR(kinCalibSimplex), false), precalculateProjection(
					CSTR(precalculateProjection), false), timeToShift(
					CSTR(timeToShift), 0, 0.005, 3, 0.20), maxIteration(
					CSTR(maxIteration), 1, 1, 100000, 100)
	{

	}
};
/**
* @ingroup VisionModule
*
* @brief Parameters container class for gui
**/
class guiC
{
public:
	GroupParameter grpRviz;
	GroupParameter grpGui;

	pbool calibDistSize_Goal;
	pbool calibDistSize_Ball;
	pbool showDistSize_Goal;
	pbool showDistSize_Ball;
	pbool saveSample;
	pbool saveHistogram;
	pbool debugCascade;
	pbool debugBallCandidates;
	pbool tfCalib;
	pbool rvizSetLoc;
	pbool rvizSetCalibTF;
	pbool printUnderMouseInfo;
	pbool rvizSetRobotTFOrigin;

	guiC(string prefix) :
			MP(calibDistSize_Goal,false), MP(calibDistSize_Ball,false), MP(showDistSize_Goal,false), MP(showDistSize_Ball,false), MP(saveSample,false),
			MP(saveHistogram,false), MP(debugCascade,false), MP(debugBallCandidates,false), MP(tfCalib,false), rvizSetLoc(
					CSTR(rviz/setLoc), false), rvizSetCalibTF(
					CSTR(rviz/setCalibTF), false), MP(printUnderMouseInfo,false) ,rvizSetRobotTFOrigin(
					CSTR(rviz/setRobotTFOrigin), false)
	{
		grpRviz.add(&rvizSetLoc);
		grpRviz.add(&rvizSetCalibTF);
		grpRviz.add(&rvizSetRobotTFOrigin);


		grpGui.add(&calibDistSize_Goal);
		grpGui.add(&calibDistSize_Ball);
		grpGui.add(&showDistSize_Goal);
		grpGui.add(&showDistSize_Ball);
		grpGui.add(&saveSample);
		grpGui.add(&saveHistogram);
		grpGui.add(&debugCascade);
		grpGui.add(&debugBallCandidates);
		grpGui.add(&tfCalib);
		grpGui.add(&printUnderMouseInfo);

		debugCascade.set(false); //To prevent running with debugCascade=true
	}
	void update()
	{
		grpRviz.update();
		grpGui.update();
	}
};
/**
* @ingroup VisionModule
*
* @brief Parameters container class for camera calibration
**/
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
/**
* @ingroup VisionModule
*
* @brief Parameters container class
**/
class ParametersC
{
public:
	bool inited;
	bool destroyed;
	string configPath;
	localParameter<int> maxrate;
	localParameter<float> fps;
	localParameter<unsigned long> counter;
	Point2f locClicked;
	bool locIsClicked;
	CameraCalibratorC camCalibrator;

	ballParamC *ball;
	fieldParamC *field;
	obstacleParamC *obstacle;
	goalParamC *goal;
	lineParamC *line;
	igusParamC *igus;
	cameraC *camera;
	topViewC *topView;
	calibC *calib;
	debugC *debug;
	circleC *circle;
	localizationC *loc;
	tfC *tfP;
	guiC *gui;
	ros::Time lastLoadTime;
	string lastLoadPath;
	ros::Subscriber loadTime_sub;
	ParametersC() :
			inited(false), destroyed(false), maxrate(30), fps(maxrate.get()), counter(
					0), locClicked(Point2f(0, 0)), locIsClicked(false), camCalibrator(), ball(
			NULL), field(NULL), obstacle(NULL), goal(NULL), line(NULL), igus(
			NULL), camera(NULL), topView(NULL), calib(NULL), debug(
			NULL), circle(NULL), loc(NULL), tfP(NULL), gui(NULL)
	{
		configPath = ros::package::getPath("launch") + "/config/vision/";
		lastLoadTime.sec=lastLoadTime.nsec=0;
		lastLoadPath="";
	}

	void loadTime_callback(const config_server::ParameterLoadTimeConstPtr &msg)
	{
		if(lastLoadTime.sec!=0 && lastLoadTime.nsec!=0 && lastLoadPath!="" && msg->filename!=lastLoadPath && msg->stamp.sec!=lastLoadTime.sec && msg->stamp.nsec!=lastLoadTime.nsec)
		{
			ROS_ERROR("Config server reloaded, so this node will be shutdown intentionally.");
			exit(1);
		}
		else
		{
			lastLoadTime=msg->stamp;
			lastLoadPath=msg->filename;
		}
	}

	void Init(ros::NodeHandle &nh)
	{
		if (inited)
		{
			return;
		}
		loadTime_sub=nh.subscribe<config_server::ParameterLoadTime>("/config_server/load_time",10,&ParametersC::loadTime_callback,this);
		ball = new ballParamC("/vision/ball/");
		field = new fieldParamC("/vision/field/");
		obstacle = new obstacleParamC("/vision/obstacle/");
		goal = new goalParamC("/vision/goal/");
		line = new lineParamC("/vision/line/");
		igus = new igusParamC("/vision/igus/");
		camera = new cameraC("/vision/camera/");
		topView = new topViewC("/vision/topView/");
		calib = new calibC("/vision/calib/");
		debug = new debugC("/vision/debug/");
		circle = new circleC("/vision/circle/");
		loc = new localizationC("/localization/");
		tfP = new tfC("/vision/tf/");
		gui = new guiC("/vision/gui/");
		inited=true;
	}

	void Destroy()
	{
		if (!inited)
		{
			return;
		}
		if (!destroyed)
		{
			delete ball;
			delete field;
			delete obstacle;
			delete goal;
			delete line;
			delete igus;
			delete camera;
			delete topView;
			delete calib;
			delete debug;
			delete circle;
			delete loc;
			delete tfP;
			delete gui;
			destroyed = true;
		}
	}

	void update()
	{
		gui->update();
	}

	virtual ~ParametersC()
	{
		Destroy();
	}
};

extern ParametersC params;
