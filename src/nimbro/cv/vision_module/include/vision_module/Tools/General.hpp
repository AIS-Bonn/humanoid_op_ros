// General Functions for Vision Module
// Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#pragma once
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/LineSegment.hpp>
#include <sstream>
using namespace cv;
using namespace std;

#define min_n(a,b) ((a) < (b) ? (a) : (b))
#define max_n(a,b) ((a) > (b) ? (a) : (b))
#define DB() printf("Line number %d in file %s\n", __LINE__, __FILE__)
struct MyCircle
{
	cv::Point2f Center;
	float radius;
};
cv::Rect MyCircleToRect(MyCircle circ);
MyCircle RectToMyCircle(cv::Rect rec);
void RotateAroundPoint(Point2d pQuery, double alpha, Point2d &res);
void RotateCoordinateAxis(double alpha, Point2d p, Point2d &res);
double Radian2Degree(double r);
double Degree2Radian(double d);
void lowPass(Point3d newrec, Point3d &res, double coef = 0.01);
void lowPass(double newrec, double &res, double coef = 0.01);
Point2d GetCenterMinBounding(vector<Point> con);
Point2d GetCenterBoundingRec(vector<Point> con);
double GetDistance(Point2d p);
double GetDistance(Point2d p, Point2d p2);
int Left(Rect rec);


cv::Point2f RotateCoordinateAxis(double alpha, cv::Point2f p);

cv::Point2f RotateAroundPoint(cv::Point2f pQuery, double alpha, cv::Point2f pCenter);

cv::Point2f RotateAroundPoint(cv::Point2f pQuery, double alpha);
int Top(Rect rec);
int Right(Rect rec);
int Bottom(Rect rec);
Scalar grayWhite();
cv::Scalar pinkColor();
cv::Scalar pinkMeloColor();
Scalar whiteColor();
Scalar redColor();
Scalar greenColor();
Scalar yellowColor();
Scalar blueColor();
Scalar blackColor();

Scalar randColor();
Point2d GetCenter(Rect rec);
bool MinXPoint(vector<Point2f> con, Point2f &res);
bool MinYPoint(vector<Point2f> con, Point2f &res);
bool MaxXPoint(vector<Point2f> con, Point2f &res);
bool MaxYPoint(vector<Point2f> con, Point2f &res);

double AngleBetween0to180(RotatedRect calculatedRect);
vector<Point> convert(const vector<Point2f> &con, vector<Point> res);

vector<Point2f> convert(const vector<Point> &con, vector<Point2f> res);
Point convert(const Point2f &in);
Point2f convert(const Point &in);
float
dist3D_Segment_to_Segment(LineSegment S1, LineSegment S2);
bool checkBox_Size_Scale_Angle(const vector<Point2f> &con, hsvRangeC type);

float DistanceFromLineSegment(LineSegment line, Point2f p);
double Distance2point(Point2f begin, Point2f end);

Point2f GetAverage(Point2f p0, Point2f p1);
Point2f GetWeigthedAverage(Point2f p0, Point2f p1, float w0, float w1);
float GetWeigthedAverage(float p0, float p1, float w0, float w1);
Point2f AddP(Point2f a, float b);
//Normalize to [0,360):
double CorrectAngleDegree360(double x);
//Normalize to [-180,180)
double CorrectAngleDegree180(double x);
//Normalize to [0,360):
double CorrectAngleRadian360(double x);
//Normalize to [-180,180)
double CorrectAngleRadian180(double x);

bool MergeLinesMax(vector<LineSegment> resLinesReal, double maxDegree,
		double maxDistance, vector<LineSegment> &clusteredLines, Rect box,
		bool useBounding = false);

bool MergeLinesOnce(vector<LineSegment> resLinesReal, double maxDegree,
		double maxDistance, vector<LineSegment> &clusteredLines, Rect box,
		bool useBounding = false);
void drawLine(Mat img, Vec4f line, int thickness, Scalar color);
template<class _Tp> void write2File(const string& filename,
		const vector<_Tp> &items)
{
	string root_path = "/nimbro/share/launch/config/vision/";
	FileStorage fs(root_path + filename, FileStorage::WRITE);

	ostringstream oss;

	fs << "Count" << (int) items.size();

	for (size_t i = 0; i < items.size(); ++i)
	{
		oss << "Element" << i;
		fs << oss.str() << items[i];
		// clear, because eof or other bits may be still set.
		oss.clear();
		oss.str("");
	}
	fs.release();
}

template<class _Tp> bool readFromFile(const string& filename,
		vector<_Tp> &myVec)
{
	string root_path = "/nimbro/share/launch/config/vision/";
	FileStorage fs(root_path + filename, FileStorage::READ);
	ostringstream oss;
	int num;
	num = (int) fs["Count"];
	for (size_t i = 0; i < num; ++i)
	{
		_Tp apoint;
		oss << "Element" << i;
		fs[oss.str()] >> apoint;
		myVec.push_back(apoint.clone());
		// clear, because eof or other bits may be still set.
		oss.clear();
		oss.str("");
	}
	fs.release();
	return true;
}

template<class _Tp> void add2File(const string& filename, const _Tp &item)
{
	vector<_Tp> readVec;
	readFromFile(filename, readVec);
	readVec.push_back(item);
	write2File(filename, readVec);
}
