//LineSegment.h
// Created on: Apr 10, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <math.h>
#include <algorithm>    // std::max
using namespace std;
using namespace cv;
#define DEFFLEQEPSILON 0.001
/**
* @ingroup VisionModule
*
* @brief A class representing line segments
**/
class LineSegment
{
private:
	bool Within(float fl, float flLow, float flHi, float flEp =
	DEFFLEQEPSILON);
	double probability;
public:
	bool SortbyDistance(const Point2f & a, const Point2f &b);
	bool IsOnThis(const Point2f& ptTest, float flEp =
	DEFFLEQEPSILON);
	LineSegment(const Point2d p1, const Point2d p2,double _probability=0.0);
	LineSegment(const Point2d center, double angle,double length,double _probability=0.0);
	LineSegment(const LineSegment &l);
	LineSegment();
	Point2d P1, P2;
	inline void setProbability(double _in)
	{
		probability=std::max(0.0,std::min(_in,1.0));
	}
	inline double getProbability() const
	{
		return probability;
	}
	double GetLength() const;
	void Clip(Rect boundry);
	void GetDirection(Point2d &res) const;
	float GetYByX(float x) const;
	int GetSide(const Point2d point) const;
	double GetExteriorAngleDegree(const LineSegment otherLine) const;
	double GetAbsMinAngleDegree(const LineSegment otherLine) const;
	virtual ~LineSegment();
	Point2f GetClosestPointOnLineSegment(Point2f p);
	Point2f GetMiddle();
	float DistanceFromLine(Point2f p);
	bool Intersect(LineSegment L, Point2d &res);
	bool IntersectLineForm(LineSegment L, Point2d &res);
	LineSegment PerpendicularLineSegment(double scale = 1);
	LineSegment PerpendicularLineSegment(double len, cv::Point2d mid);
	bool GetSlope(double &slope)
	{
		if (abs(P2.x - P1.x) < 0.00001)
		{
			return false;
		}
		slope = (P2.y - P1.y) / (P2.x - P1.x);
		return true;
	}
	double GetRadianFromX()
	{
		double res = atan2f((P2.y - P1.y), (P2.x - P1.x));
		return res;
	}
	double GetDegreeFromX()
	{
		return (GetRadianFromX() / M_PI) * (180);;
	}
	LineSegment scale(double _in)
	{
		double angle=GetRadianFromX();
		Point2d mid=GetMiddle();
		double len=GetLength()/2;
		len*=_in;
		double x2=mid.x+cos(angle)*len;
		double y2=mid.y+sin(angle)*len;
		double x1=mid.x-cos(angle)*len;
		double y1=mid.y-sin(angle)*len;
		return LineSegment(Point2d(x1,y1),Point2d(x2,y2));
	}

	vector<LineSegment> GetMidLineSegments(
			int count /*Means that lines count will be 2^count*/)
	{
		vector<LineSegment> lsList, lsListTmp;
		lsList.push_back(LineSegment(P1, P2));

		for (int counter = 0; counter < count; counter++)
		{
			lsListTmp.clear();
			for (size_t i = 0; i < lsList.size(); i++)
			{
				LineSegment tmp = lsList[i];
				Point2d midPoint = tmp.GetMiddle();
				lsListTmp.push_back(LineSegment(tmp.P1, midPoint));
				lsListTmp.push_back(LineSegment(tmp.P2, midPoint));
			}
			lsList = lsListTmp;
		}
		return lsList;
	}

	vector<Point2d> GetMidPoints(
			int count /*Means that mid count will be (2^count)+1*/, bool sortthis =
					true)
	{
		//int resCount=pow (2, count)+1;
		vector<LineSegment> lsList, lsListTmp;
		lsList.push_back(LineSegment(P1, P2));
		vector<Point2d> res;
		res.push_back(P1);
		res.push_back(P2);
		for (int counter = 0; counter < count; counter++)
		{
			lsListTmp.clear();
			for (size_t i = 0; i < lsList.size(); i++)
			{
				LineSegment tmp = lsList[i];
				Point2d midPoint = tmp.GetMiddle();
				res.push_back(midPoint);
				lsListTmp.push_back(LineSegment(tmp.P1, midPoint));
				lsListTmp.push_back(LineSegment(tmp.P2, midPoint));
			}
			lsList = lsListTmp;
		}

		if (sortthis)
		{
			sort(res.begin(), res.end(),
					bind(&LineSegment::SortbyDistance, this, _1, _2));
		}

		return res;
	}
};
