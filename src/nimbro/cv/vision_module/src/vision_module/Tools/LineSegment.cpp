//LineSegment.cpp
// Created on: Apr 10, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#include <vision_module/Tools/LineSegment.hpp>

#define FLOAT_EQE(x,v,e)((((v)-(e))<(x))&&((x)<((v)+(e))))

bool LineSegment::SortbyDistance(const Point2f & a, const Point2f &b)
{
	double x = abs(P1.x - a.x);
	double y = abs(P1.y - a.y);
	double distanceA = sqrt(x * x + y * y);

	x = abs(P1.x - b.x);
	y = abs(P1.y - b.y);
	double distanceB = sqrt(x * x + y * y);

	return distanceA < distanceB;
}

bool LineSegment::Within(float fl, float flLow, float flHi, float flEp)
{
	if ((fl > flLow) && (fl < flHi))
	{
		return true;
	}
	if (FLOAT_EQE(fl,flLow,flEp) || FLOAT_EQE(fl, flHi, flEp))
	{
		return true;
	}
	return false;
}

LineSegment LineSegment::PerpendicularLineSegment(double len,cv::Point2d mid)
{
	double angle = GetRadianFromX();
	angle += M_PI / 2;
	double x1 = mid.x + cos(angle)*len;
	double y1 = mid.y + sin(angle)*len;
	double x2 = mid.x - cos(angle)*len;
	double y2 = mid.y - sin(angle)*len;
	return LineSegment(cv::Point2d(x1, y1), cv::Point2d(x2, y2));
}


void LineSegment::Clip(Rect boundry)
{
	P1.x = std::min((double) boundry.width-1, std::max((double) boundry.x, P1.x));
	P1.y = std::min((double) boundry.height-1,
			std::max((double) boundry.y, P1.y));
	P2.x = std::min((double) boundry.width-1, std::max((double) boundry.x, P2.x));
	P2.y = std::min((double) boundry.height-1,
			std::max((double) boundry.y, P2.y));
}

bool LineSegment::IsOnThis(const Point2f& ptTest, float flEp)
{
	bool bTestX = true;
	const float flX = P2.x - P1.x;
	if (FLOAT_EQE(flX, 0.0f, flEp))
	{
		// vertical line -- ptTest.X must equal ptL1.X to continue
		if (!FLOAT_EQE(ptTest.x, P1.x, flEp))
		{
			return false;
		}
		bTestX = false;
	}
	bool bTestY = true;
	const float flY = P2.y - P1.y;
	if (FLOAT_EQE(flY, 0.0f, flEp))
	{
		// horizontal line -- ptTest.Y must equal ptL1.Y to continue
		if (!FLOAT_EQE(ptTest.y, P1.y, flEp))
		{
			return false;
		}
		bTestY = false;
	}
	// found here: http://stackoverflow.com/a/7050309
	// x = x1 + (x2 - x1) * p
	// y = y1 + (y2 - y1) * p
	// solve for p:
	const float pX = bTestX ? ((ptTest.x - P1.x) / flX) : 0.5f;
	const float pY = bTestY ? ((ptTest.y - P1.y) / flY) : 0.5f;
	return Within(pX, 0.0f, 1.0f, flEp) && Within(pY, 0.0f, 1.0f, flEp);
}

// Get the length of the line segment
double LineSegment::GetLength() const
{
	float num1 = P1.x - P2.x;
	float num2 = P1.y - P2.y;
	return sqrt((double) num1 * (double) num1 + (double) num2 * (double) num2);
}

// The direction of the line, the norm of which is 1
void LineSegment::GetDirection(Point2d &res) const
{
	float num1 = P2.x - P1.x;
	float num2 = P2.y - P1.y;
	float num3 = (float) sqrt(
			(double) num1 * (double) num1 + (double) num2 * (double) num2);
	res = Point2d(num1 / num3, num2 / num3);
}

// Obtain the Y value from the X value using first degree interpolation
float LineSegment::GetYByX(float x) const
{
	Point2d pointF = P1;
	Point2d direction;
	this->GetDirection(direction);
	return (x - pointF.x) / direction.x * direction.y + pointF.y;
}

//Copied From http://stackoverflow.com/questions/1811549/perpendicular-on-a-line-from-a-given-point
Point2f LineSegment::GetClosestPointOnLineSegment(Point2f p)
{
	// first convert line to normalized unit vector
	double dx = P2.x - P1.x;
	double dy = P2.y - P1.y;
	double mag = sqrt(dx * dx + dy * dy);
	dx /= mag;
	dy /= mag;

	// translate the point and get the dot product
	double lambda = (dx * (p.x - P1.x)) + (dy * (p.y - P1.y));
	return Point2f((dx * lambda) + P1.x, (dy * lambda) + P1.y);
}

Point2f LineSegment::GetMiddle()
{
	return Point2f((P1.x + P2.x) / 2., (P1.y + P2.y) / 2.);
}

// Determin which side of the line the 2D point is at

// 1 if on the right hand side;
//         0 if on the line;
//        -1 if on the left hand side;
//
int LineSegment::GetSide(Point2d point) const
{
	float num = (float) (((double) P2.x - (double) P1.x)
			* ((double) point.y - (double) P1.y)
			- ((double) point.x - (double) P1.x)
					* ((double) P2.y - (double) P1.y));
	if ((double) num > 0.0)
		return 1;
	return (double) num >= 0.0 ? 0 : -1;
}
// Get the exterior angle between this line and otherLine
//Result is betwenn -180 ~ 180
// the order is important for the result sign
double LineSegment::GetExteriorAngleDegree(const LineSegment otherLine) const
{
	Point2d direction1;
	this->GetDirection(direction1);
	Point2d direction2;
	otherLine.GetDirection(direction2);
	double num = (atan2((double) direction2.y, (double) direction2.x)
			- atan2((double) direction1.y, (double) direction1.x))
			* 57.2957795130823;
	if (num <= -180.0)
		return num + 360.0;
	if (num <= 180.0)
		return num;
	return num - 360.0;
}
//Result is between 0 ~ 90
double LineSegment::GetAbsMinAngleDegree(const LineSegment otherLine) const
{
	double diffAngle = GetExteriorAngleDegree(otherLine);
	double res = min(abs(diffAngle), 180 - abs(diffAngle));
	if (res < 0 || res > 90)
	{
		ROS_ERROR("Error In Programming");
	}
	return res;
}

LineSegment::LineSegment()
{
	P1 = Point2f(0, 0);
	P2 = Point2f(0, 0);
}

LineSegment::LineSegment(const Point2d p1, const Point2d p2)
{
	P1 = p1;
	P2 = p2;
}

LineSegment::LineSegment(const LineSegment &l)
{
	P1 = l.P1;
	P2 = l.P2;
}

float LineSegment::DistanceFromLine(Point2f p)
{
	double x1 = P1.x;
	double y1 = P1.y;
	double x2 = P2.x;
	double y2 = P2.y;

	double dx = x2 - x1;
	double const Epsilon = 0.0001;
	if (abs(dx) < Epsilon)
	{
		return abs(p.x - x1);
	}

	double m = (y2 - y1) / (x2 - x1);
	double b = -m * x1 + y1;
	//y=mx+b

	double dist = (float) (abs(p.y - m * p.x - b) / sqrt(m * m + 1));
	return dist;
}

//Copy from: http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
bool LineSegment::Intersect(LineSegment L, Point2d &res)
{
	float s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom, t;
	s10_x = P2.x - P1.x;
	s10_y = P2.y - P1.y;
	s32_x = L.P2.x - L.P1.x;
	s32_y = L.P2.y - L.P1.y;

	denom = s10_x * s32_y - s32_x * s10_y;
	if (denom == 0)
		return false; // Collinear
	bool denomPositive = denom > 0;

	s02_x = P1.x - L.P1.x;
	s02_y = P1.y - L.P1.y;
	s_numer = s10_x * s02_y - s10_y * s02_x;
	if ((s_numer < 0) == denomPositive)
		return false; // No collision

	t_numer = s32_x * s02_y - s32_y * s02_x;
	if ((t_numer < 0) == denomPositive)
		return false; // No collision

	if (((s_numer > denom) == denomPositive)
			|| ((t_numer > denom) == denomPositive))
		return false; // No collision
	// Collision detected
	t = t_numer / denom;

	res.x = P1.x + (t * s10_x);

	res.y = P1.y + (t * s10_y);

	return true;
}

bool LineSegment::IntersectLineForm(LineSegment L, Point2d &res)
{

	Point2d x = L.P1 - P1;
	Point2d d1 = P2 - P1;
	Point2d d2 = L.P2 - L.P1;

	float cross = d1.x * d2.y - d1.y * d2.x;
	if (abs(cross) < /*EPS*/1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	res = P1 + (d1 * t1);
	return true;
}

LineSegment LineSegment::PerpendicularLineSegment(double scale)
{
	double angle=GetRadianFromX();
	angle+=M_PI/2;
	Point2d mid=GetMiddle();
	double len=GetLength()/2;
	len*=scale;
	double x1=mid.x+cos(angle)*len;
	double y1=mid.y+sin(angle)*len;
	double x2=mid.x-cos(angle)*len;
	double y2=mid.y-sin(angle)*len;
	return LineSegment(Point2d(x1,y1),Point2d(x2,y2));
}

LineSegment::~LineSegment()
{

}

