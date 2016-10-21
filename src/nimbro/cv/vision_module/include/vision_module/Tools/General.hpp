// General Functions for Vision Module
// Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#pragma once
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/LineSegment.hpp>
#include <boost/timer/timer.hpp>
#include <ros/ros.h>
#include <sstream>
using namespace cv;
using namespace std;

#define HAF_LOG_THROTTLE(rate, level, name, ...) \
  do \
  { \
    ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
    static double last_hit = 0.0; \
    ::ros::WallTime now = ::ros::WallTime::now(); \
    if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && ROS_UNLIKELY(last_hit + rate <= now.toSec())) \
    { \
      last_hit = now.toSec(); \
      ROSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
    } \
  } while(0)

#define HAF_INFO_THROTTLE(rate, ...) HAF_LOG_THROTTLE(rate, ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define HAF_WARN_THROTTLE(rate, ...) HAF_LOG_THROTTLE(rate, ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define HAF_ERROR_THROTTLE(rate, ...) HAF_LOG_THROTTLE(rate, ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define HAF_DEBUG_THROTTLE(rate, ...) HAF_LOG_THROTTLE(rate, ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)

#define min_n(a,b) ((a) < (b) ? (a) : (b))
#define max_n(a,b) ((a) > (b) ? (a) : (b))
#define boundry_n(n,a,b) {n=min_n(b,n);n=max_n(a,n);}
#define DB() printf("Line number %d in file %s\n", __LINE__, __FILE__)
#define DT() boost::timer::auto_cpu_timer tmpAutoTimer
class CircleC
{
public:
	cv::Point2f Center;
	float Radius;
	CircleC()
	{
		Radius=0;
	}
	CircleC(cv::Point2f Center,float Radius):Center(Center),Radius(Radius)
	{

	}
	CircleC(const CircleC &_in):Center(_in.Center),Radius(_in.Radius)
	{

	}
};

CircleC mergeCircleC(CircleC A, CircleC B);
bool hasIntersection(CircleC A, CircleC B);
bool lineFormIntersectionWithCircleC(LineSegment l, CircleC c, Point2f &p1, Point2f &p2);

cv::Rect CircleCToRect(CircleC circ);
CircleC RectToCircleC(cv::Rect rec);
void RotateAroundPoint(Point2d pQuery, double alpha, Point2d &res);
void RotateCoordinateAxis(double alpha, Point2d p, Point2d &res);
double Radian2Degree(double r);
double Degree2Radian(double d);
void lowPass(Point3d newrec, Point3d &res, double coef = 0.01);
void lowPass(Point2d newrec, Point2d &res, double coef = 0.01);
void lowPass(double newrec, double &res, double coef = 0.01);
void lowPassAngleDegree180(double newrec, double &res, double coef = 0.01);
double AngleSubDegree180(double first, double second);
Point2d GetCenterMinBounding(vector<Point> con);
Point2d GetCenterBoundingRec(vector<Point> con);
double GetDistance(Point2d p);
float GetDistance(Point2f p);
double GetDistance(Point2d p, Point2d p2);

int Left(Rect rec);
//http://www.juergenwiki.de/work/wiki/doku.php?id=public:hog_descriptor_computation_and_visualization
Mat get_hogdescriptor_visual_image(Mat& origImg,
		vector<float>& descriptorValues, Size winSize, Size cellSize,
		int scaleFactor, double viz_factor);

/// <summary>
/// Rotates the coordinate axises and returns the new coordinate of insetreted point
/// </summary>
/// <param name="alpha">angle (in degree) to rotate the coordinate axises</param>
/// <param name="p">the point in old coordinate axis</param>
/// <returns>the point in new coordinate axis</returns>
cv::Point2f RotateCoordinateAxis(double alpha, cv::Point2f p);
/// <summary>
/// Rotates the a pQuery arount pCenter whith alpha degree
/// </summary>
cv::Point2f RotateAroundPoint(cv::Point2f pQuery, double alpha,
		cv::Point2f pCenter);

/// <summary>
/// Rotates the a pQuery arount pCenter whith alpha degree
/// </summary>
cv::Point2f RotateAroundPoint(cv::Point2f pQuery, double alpha);
int Top(Rect rec);
int Right(Rect rec);
int Bottom(Rect rec);
Scalar grayWhite();
cv::Scalar pinkColor();
cv::Scalar pinkMeloColor();
Scalar whiteColor();
Scalar redColor();
Scalar darkOrangeColor();
Scalar redMeloColor();
Scalar greenColor();
Scalar yellowColor();
Scalar blueColor();
Scalar blueMeloColor();
Scalar blackColor();
cv::Scalar blackGary();

Scalar randColor();
Point2d GetCenter(Rect rec);
bool MinXPoint(vector<Point2f> con, Point2f &res);
bool MinYPoint(vector<Point2f> con, Point2f &res);
bool MaxXPoint(vector<Point2f> con, Point2f &res);
bool MaxYPoint(vector<Point2f> con, Point2f &res);
bool MinXPoint(vector<Point> con, Point &res);
bool MinYPoint(vector<Point> con, Point &res);
bool MaxXPoint(vector<Point> con, Point &res);
bool MaxYPoint(vector<Point> con, Point &res);

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
//Normalize to [-180,180)
float AngleDiffDegree180(float first, float second);
//Normalize to [-180,180)
float AngleDiffRadian180(float first, float second);

bool MergeLinesMax(vector<LineSegment> resLinesReal, double maxDegree,
		double maxDistance, vector<LineSegment> &clusteredLines, Rect box,
		bool useBounding = false);

bool MergeLinesOnce(vector<LineSegment> resLinesReal, double maxDegree,
		double maxDistance, vector<LineSegment> &clusteredLines, Rect box,
		bool useBounding = false);
void drawLine(Mat img, Vec4f line, int thickness, Scalar color);
bool calcHist3Channels(cv::Mat &colorMatRoi, cv::Mat &justObjMask,
		cv::Mat hist_base[3]);

cv::Rect clipRect(cv::Rect _in, cv::Rect _box);
bool minDimentionRect(int _min, cv::Rect &_in);
bool maxDimentionRect(int _max, cv::Rect &_in);
bool mergeRect(cv::Rect &_in, cv::Rect _in2);
cv::Rect asymetricScaleRec(double _xLeftRatio, double _xRightRatio,
		double _yTopRatio, double _yDownRatio, cv::Rect _in);
cv::Rect asymetricScaleRec(double _xLeftRatio, double _xRightRatio,
		double _yTopRatio, double _yDownRatio, cv::Rect _in, cv::Rect _box);
cv::Rect scaleRec(double _xRatio, double _yRatio, cv::Rect _in);
cv::Rect scaleRec(double _xRatio, double _yRatio, cv::Rect _in, cv::Rect _box);

template<class _Tp> bool write2File(const string& filename,
		const vector<_Tp> &items)
{
	try
	{
		FileStorage fs(params.configPath + filename, FileStorage::WRITE);
//		ROS_INFO("write2File=%s%s",params.configPath.c_str(), filename.c_str());
//		ROS_INFO("write2File Size=%d",(int)items.size());
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
		return true;
	} catch (Exception &e)
	{
		ROS_ERROR("Problem writing %s |-> what = %s", filename.c_str(),
				e.what());
		return false;
	}
}

template<class _Tp> bool readFromFile(const string& filename,
		vector<_Tp> &myVec)
{
	try
	{
		FileStorage fs(params.configPath + filename, FileStorage::READ);

		ostringstream oss;
		int num = 0;
		num = (int) fs["Count"];
		myVec.resize(num);
		for (int i = 0; i < num; ++i)
		{
			oss << "Element" << i;
			fs[oss.str()] >> myVec[i];
			oss.clear();
			oss.str("");
		}
		fs.release();
		return num > 0;
	} catch (Exception &e)
	{
		ROS_ERROR("Problem parsing %s |-> what = %s", filename.c_str(),
				e.what());
		return false;
	}
}

template<class _Tp> bool add2File(const string& filename, const _Tp &item)
{
	vector<_Tp> readVec;
	bool res = readFromFile(filename, readVec);
	readVec.push_back(item);
	res &= write2File(filename, readVec);
	return res;
}

template<typename T>
string to_string(T pNumber)
{
	ostringstream oOStrStream;
	oOStrStream << pNumber;
	return oOStrStream.str();
}
