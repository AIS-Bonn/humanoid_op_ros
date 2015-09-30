// General Functions for Vision Module
// Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <vision_module/Tools/General.hpp>

RNG rng(12345);


cv::Point2f RotateCoordinateAxis(double alpha, cv::Point2f p) {
	alpha = Degree2Radian(alpha);
	return cv::Point2f((float) (p.x * cos(alpha) - p.y * sin(alpha)),
			(float) (p.x * sin(alpha) + p.y * cos(alpha)));
}


cv::Point2f RotateAroundPoint(cv::Point2f pQuery, double alpha,
		cv::Point2f pCenter) {

	cv::Point2f p = pQuery - pCenter;
	return RotateCoordinateAxis(-alpha, p) + pCenter;
}


cv::Point2f RotateAroundPoint(cv::Point2f pQuery, double alpha) {
	cv::Point2f pCenter;
	cv::Point2f p = pQuery - pCenter;
	return RotateCoordinateAxis(-alpha, p) + pCenter;
}
double dot(Point3f c1, Point3f c2) {
	return (c1.x * c2.x + c1.y * c2.y + c1.z * c2.z);
}

cv::Rect MyCircleToRect(MyCircle circ) {
	return cv::Rect(circ.Center.x - circ.radius, circ.Center.y - circ.radius,
			circ.radius * 2, circ.radius * 2);
}

MyCircle RectToMyCircle(cv::Rect rec) {
	MyCircle res;

	res.Center.x = rec.x + rec.width / 2.;
	res.Center.y = rec.y + rec.height / 2.;
	res.radius = min(rec.width / 2, rec.height / 2);
	return res;
}

cv::Scalar pinkColor() {
	return cv::Scalar(255, 0, 255);
}

cv::Scalar pinkMeloColor() {
	return cv::Scalar(183, 50, 130);
}

// Copyright 2001 softSurfer, 2012 Dan Sunday===============================
#define SMALL_NUM   0.00000001 // anything that avoids division overflow
// dot product (3D) which allows vector operations in arguments
#define d(u,v)     norm(u-v)        // distance = norm of difference
#define abs(x)     ((x) >= 0 ? (x) : -(x))   //  absolute value
#define sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )

double norm(Point3f c1) {
	return std::sqrt(dot(c1, c1));
}
// dist3D_Segment_to_Segment(): get the 3D minimum distance between 2 segments
//    Input:  two 3D line segments S1 and S2
//    Return: the shortest distance between S1 and S2
float dist3D_Segment_to_Segment(LineSegment S1, LineSegment S2) {

	Point3f u;
	u.x = S1.P2.x - S1.P1.x;
	u.y = S1.P2.y - S1.P1.y;
	u.z = 0;

	Point3f v;
	v.x = S2.P2.x - S2.P1.x;
	v.y = S2.P2.y - S2.P1.y;
	v.z = 0;
	cv::Point3f w;
	w.x = S1.P1.x - S2.P1.x;
	w.y = S1.P1.y - S2.P1.y;
	w.z = 0;

	float a = dot(u, u);         // always >= 0
	float b = dot(u, v);
	float c = dot(v, v);         // always >= 0
	float d = dot(u, w);
	float e = dot(v, w);
	float D = a * c - b * b;        // always >= 0
	float sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
	float tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

	// compute the line parameters of the two closest points
	if (D < SMALL_NUM) { // the lines are almost parallel
		sN = 0.0;         // force using point P0 on segment S1
		sD = 1.0;         // to prevent possible division by 0.0 later
		tN = e;
		tD = c;
	} else {                 // get the closest points on the infinite lines
		sN = (b * e - c * d);
		tN = (a * e - b * d);
		if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
			sN = 0.0;
			tN = e;
			tD = c;
		} else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
			sN = sD;
			tN = e + b;
			tD = c;
		}
	}

	if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
		tN = 0.0;
		// recompute sc for this edge
		if (-d < 0.0)
			sN = 0.0;
		else if (-d > a)
			sN = sD;
		else {
			sN = -d;
			sD = a;
		}
	} else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
		tN = tD;
		// recompute sc for this edge
		if ((-d + b) < 0.0)
			sN = 0;
		else if ((-d + b) > a)
			sN = sD;
		else {
			sN = (-d + b);
			sD = a;
		}
	}
	// finally do the division to get sc and tc
	sc = (abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
	tc = (abs(tN) < SMALL_NUM ? 0.0 : tN / tD);

	// get the difference of the two closest points
	Point3d dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)

	return norm(dP);   // return the closest distance
}
//End of CopyRight===================================================================

double GetDistance(Point2d p) {
	return sqrt((p.x * p.x) + (p.y * p.y));
}

double GetDistance(Point2d p, Point2d p2) {
	double x = abs(p.x - p2.x);
	double y = abs(p.y - p2.y);
	return sqrt(x * x + y * y);
}

Point2d GetCenterMinBounding(vector<Point> con) {
	return minAreaRect(con).center;
}

Point2d GetCenter(Rect rec) {
	Point2d b(rec.x + (rec.width / 2.), rec.y + (rec.height / 2.));
	return b;
}

Point2d GetCenterBoundingRec(vector<Point> con) {
	Rect bounding_rect = boundingRect(con);
	Point2d b(bounding_rect.x + (bounding_rect.width / 2.),
			bounding_rect.y + (bounding_rect.height / 2.));
	return b;
}

void RotateAroundPoint(Point2d pQuery, double alpha, Point2d &res) {
	Point2d pCenter(0, 0);
	Point2d p = pQuery - pCenter;
	Point2d resRot;
	RotateCoordinateAxis(-alpha, p, resRot);
	res = resRot + pCenter;
}

void RotateCoordinateAxis(double alpha, Point2d p, Point2d &res) {
	alpha = Degree2Radian(alpha);
	res = Point2d((float) (p.x * cos(alpha) - p.y * sin(alpha)),
			(float) (p.x * sin(alpha) + p.y * cos(alpha)));
}

//Normalize to [0,360):
double CorrectAngleDegree360(double x) {
	x = fmod(x, 360);
	if (x < 0)
		x += 360;
	return x;
}

//Normalize to [-180,180)
double CorrectAngleDegree180(double x) {
	x = fmod(x + 180, 360);
	if (x < 0)
		x += 360;
	return x - 180;
}

//Normalize to [0,360):
double CorrectAngleRadian360(double x) {
	return Degree2Radian((CorrectAngleDegree360((Radian2Degree(x)))));
}

//Normalize to [-180,180)
double CorrectAngleRadian180(double x) {
	return Degree2Radian((CorrectAngleDegree180((Radian2Degree(x)))));
}

void drawLine(Mat img, Vec4f line, int thickness, Scalar color) {
	double theMult = max(img.size().height, img.size().width);
// calculate start point
	Point startPoint;
	startPoint.x = line[2] - theMult * line[0]; // x0
	startPoint.y = line[3] - theMult * line[1]; // y0
// calculate end point
	Point endPoint;
	endPoint.x = line[2] + theMult * line[0];    //x[1]
	endPoint.y = line[3] + theMult * line[1];    //y[1]

// draw overlay of bottom lines on image
	clipLine(img.size(), startPoint, endPoint);
	cv::line(img, startPoint, endPoint, color, thickness);
}

double Radian2Degree(double r) {
	return (r / M_PI) * (180);
}

double Degree2Radian(double d) {
	return (d * M_PI) / (180);
}

void lowPass(Point3d newrec, Point3d &res, double coef) {
	double coefIn = 1 - coef;
	res.x = (coefIn * res.x) + (coef * newrec.x);
	res.y = (coefIn * res.y) + (coef * newrec.y);
	res.z = (coefIn * res.z) + (coef * newrec.z);
}

void lowPass(double newrec, double &res, double coef) {
	double coefIn = 1 - coef;
	res = (coefIn * res) + (coef * newrec);
}

int Left(Rect rec) {

	return rec.x;
}

int Top(Rect rec) {

	return rec.y;
}

int Right(Rect rec) {

	return rec.x + rec.width;
}

int Bottom(Rect rec) {

	return rec.y + rec.height;
}

bool MergeLinesMax(vector<LineSegment> resLinesReal, double maxDegree,
		double maxDistance, vector<LineSegment> &clusteredLines, Rect box,
		bool useBounding) {
//	ROS_WARN("()()*()()***** Start *****()()*()()");
	if (!MergeLinesOnce(resLinesReal, maxDegree, maxDistance, clusteredLines,
			box, useBounding)) {
		return false;
	}
	vector<LineSegment> before;
//	int i = 0;
	do {
//		ROS_WARN("()()*()()***** i th = %d *****()()*()()", i++);
		before = clusteredLines;
		if (!MergeLinesOnce(before, maxDegree, maxDistance, clusteredLines, box,
				useBounding)) {
			return false;
		}
	} while (before.size() > clusteredLines.size());
//	ROS_WARN("()()*()()***** Finish *****()()*()()");
	return true;
}

bool MergeLinesOnce(vector<LineSegment> resLinesReal, double maxDegree,
		double maxDistance, vector<LineSegment> &clusteredLines, Rect box,
		bool useBounding) {
	if (resLinesReal.size() < 1) {
		return false;
	}
	clusteredLines.clear();

	for (size_t i = 0; i < resLinesReal.size(); i++) {
		int index = -1;
		for (size_t j = 0; j < clusteredLines.size(); j++) {
			double diffAngle = resLinesReal[i].GetAbsMinAngleDegree(
					clusteredLines[j]);

			if ((diffAngle < maxDegree)
					&& (dist3D_Segment_to_Segment(resLinesReal[i],
							clusteredLines[j]) < maxDistance)) {
//				cout << "-- -- >   BGN Merge  < -- --" << endl;
//				cout << "DiffAngle = " << diffAngle << endl;
//				cout << "DiffDist  = "
//						<< dist3D_Segment_to_Segment(resLinesReal[i],
//								clusteredLines[j]) << endl;
//				cout << " line  = " << resLinesReal[i].P1 << " "
//						<< resLinesReal[i].P2 << endl;
//				cout << " cLine = " << clusteredLines[j].P1 << " "
//						<< clusteredLines[j].P2 << endl;
				index = j;
				break;
			} else {
//				cout << "DiffAngle = " << diffAngle << endl;
//				cout << "DiffDist  = "
//						<< dist3D_Segment_to_Segment(resLinesReal[i],
//								clusteredLines[j]) << endl;
//				cout<<" line  = "<<resLinesReal[i].P1<<" "<<resLinesReal[i].P2<< endl;
//				cout<<" cLine = "<<clusteredLines[j].P1<<" "<<clusteredLines[j].P2<<endl;
			}

		}

		if (index != -1)		//existed
				{

			vector<Point2f> pointVector;
			Point2f mergedMidPoint;
			LineSegment mergedLine;

			LineSegment queryLine1(clusteredLines[index]);
			LineSegment queryLine2(resLinesReal[i]);

			pointVector.push_back(queryLine1.P1);
			pointVector.push_back(queryLine1.P2);

			pointVector.push_back(queryLine2.P1);
			pointVector.push_back(queryLine2.P2);

			if (!useBounding) {
				float queryLineLen1 = queryLine1.GetLength();
				float queryLineLen2 = queryLine2.GetLength();

				float angle1 = queryLine1.GetRadianFromX();
				float angle2 = queryLine2.GetRadianFromX();

				mergedMidPoint = GetWeigthedAverage(queryLine1.GetMiddle(),
						queryLine2.GetMiddle(), queryLineLen1, queryLineLen2);

				float mergedAngle = CorrectAngleRadian180(
						(queryLineLen1 > queryLineLen2) ? angle1 : angle2);

				double theMult = std::max(box.width - box.x,
						box.height - box.y);

				LineSegment biggestDistanceLS;
				mergedLine.P1.x = mergedMidPoint.x - theMult * cos(mergedAngle);
				mergedLine.P1.y = mergedMidPoint.y - theMult * sin(mergedAngle);
				mergedLine.P2.x = mergedMidPoint.x + theMult * cos(mergedAngle);
				mergedLine.P2.y = mergedMidPoint.y + theMult * sin(mergedAngle);

				//Find max distance points on the new line
				double maxDistance = -9999999;
				for (int i = 0; i < 4; i++) {
					for (int j = i + 1; j < 4; j++) {
						Point2f p1 = mergedLine.GetClosestPointOnLineSegment(
								pointVector[i]);
						Point2f p2 = mergedLine.GetClosestPointOnLineSegment(
								pointVector[j]);
						double distance = LineSegment(p1, p2).GetLength();
						if (distance > maxDistance) {
							maxDistance = distance;
							biggestDistanceLS.P1 = p1;
							biggestDistanceLS.P2 = p2;
						}
					}
				}
				mergedLine = biggestDistanceLS;
			} else {
				RotatedRect minRec = minAreaRect(pointVector);
				Point2f allPoints[4];
				minRec.points(allPoints);
				if (LineSegment(allPoints[2], allPoints[1]).GetLength()
						> LineSegment(allPoints[2], allPoints[3]).GetLength()) {
					mergedLine.P1 = GetAverage(allPoints[2], allPoints[3]);
					mergedLine.P2 = GetAverage(allPoints[0], allPoints[1]);
				} else {
					mergedLine.P1 = GetAverage(allPoints[2], allPoints[1]);
					mergedLine.P2 = GetAverage(allPoints[0], allPoints[3]);
				}
			}
			mergedLine.Clip(box);

			clusteredLines[index].P1 = Point2f(mergedLine.P1.x,
					mergedLine.P1.y);
			clusteredLines[index].P2 = Point2f(mergedLine.P2.x,
					mergedLine.P2.y);

//			cout << " Merged Line = " << mergedLine.P1 << " " << mergedLine.P2
//					<< endl;
//			cout << "-- -- >      END     < -- --" << endl;
		} else {
			clusteredLines.push_back(resLinesReal[i]);
		}
	}

	return true;
}

Point2f AddP(Point2f a, float b) {
	return Point2f(a.x + b, a.y + b);
}

bool MinXPoint(vector<Point2f> con, Point2f &res) {
	float lx = 99999;
	int index = -1;
	for (size_t i = 0; i < con.size(); i++) //lowestPoint
			{
		if (con[i].x < lx) {
			index = i;
			lx = con[i].x;
		}
	}
	if (index == -1) {
		return false;
	}
	res = con[index];
	return true;
}

bool MinYPoint(vector<Point2f> con, Point2f &res) {
	float ly = 99999;
	int index = -1;
	for (size_t i = 0; i < con.size(); i++) //lowestPoint
			{
		if (con[i].y < ly) {
			index = i;
			ly = con[i].y;
		}
	}
	if (index == -1) {
		return false;
	}
	res = con[index];
	return true;
}

bool MaxXPoint(vector<Point2f> con, Point2f &res) {
	float mx = -99999;
	int index = -1;
	for (size_t i = 0; i < con.size(); i++) //lowestPoint
			{
		if (con[i].x > mx) {
			index = i;
			mx = con[i].x;
		}
	}
	if (index == -1) {
		return false;
	}
	res = con[index];
	return true;
}

bool MaxYPoint(vector<Point2f> con, Point2f &res) {
	float my = -99999;
	int index = -1;
	for (size_t i = 0; i < con.size(); i++) //lowestPoint
			{
		if (con[i].y > my) {
			index = i;
			my = con[i].y;
		}
	}
	if (index == -1) {
		return false;
	}
	res = con[index];
	return true;
}

Point2f GetAverage(Point2f p0, Point2f p1) {
	return Point2f((p0.x + p1.x) / 2., (p0.y + p1.y) / 2.);
}

Point2f GetWeigthedAverage(Point2f p0, Point2f p1, float w0, float w1) {
	float sumW = w0 + w1;
	return Point2f((p0.x * w0 + p1.x * w1) / sumW,
			(p0.y * w0 + p1.y * w1) / sumW);
}

float GetWeigthedAverage(float p0, float p1, float w0, float w1) {
	float sumW = w0 + w1;
	return (p0 * w0 + p1 * w1) / sumW;
}

double Distance2point(Point2f begin, Point2f end) {

	double x = (begin.x - end.x);
	double y = (begin.y - end.y);
	return sqrt((x * x) + (y * y));
}
// point distance from line segment
float DistanceFromLineSegment(LineSegment line, Point2f p) {
	Point2d segA = line.P1;
	Point2d segB = line.P2;

	Point2d p2(segB.x - segA.x, segB.y - segA.y);

	float something = p2.x * p2.x + p2.y * p2.y;
	float u = ((p.x - segA.x) * p2.x + (p.y - segA.y) * p2.y) / something;

	if (u > 1)
		u = 1;
	else if (u < 0)
		u = 0;

	float x = segA.x + u * p2.x;
	float y = segA.y + u * p2.y;

	float dx = x - p.x;
	float dy = y - p.y;

	float dist = (float) sqrt(dx * dx + dy * dy);

	return dist;
}

double AngleBetween0to180(RotatedRect calculatedRect) {
	if (calculatedRect.size.width < calculatedRect.size.height) {
		return calculatedRect.angle + 180;
	} else {
		return calculatedRect.angle + 90;
	}
}

bool checkBox_Size_Scale_Angle(const vector<Point2f> &con, hsvRangeC type) {
	RotatedRect rec = minAreaRect(con);
	double minlen = min(rec.size.height, rec.size.width);
	double maxlen = max(rec.size.height, rec.size.width);
	double ratio = maxlen / minlen;
	double angle = AngleBetween0to180(rec);

	if (type.maxAngle->get() >= type.minAngle->get()) {
		if (angle > type.maxAngle->get() || angle < type.minAngle->get()) {
			return false;
		}
	} else {
		if ((angle > type.maxAngle->get() && angle < type.minAngle->get())
				|| angle < 0 || angle > 180) {
			return false;
		}
	}

	if (ratio > type.maxRatio->get() || ratio < type.minRatio->get()) {
		return false;
	}

	if (maxlen > type.max_maxLen->get() || maxlen < type.min_maxLen->get()
			|| minlen < type.min_minLen->get()
			|| minlen > type.max_minLen->get()) {
		return false;
	}

	return true;
}

vector<Point> convert(const vector<Point2f> &con, vector<Point> res) {
	for (size_t i = 0; i < con.size(); i++) {
		res.push_back(convert(con[i]));
	}
	return res;
}

vector<Point2f> convert(const vector<Point> &con, vector<Point2f> res) {
	for (size_t i = 0; i < con.size(); i++) {
		res.push_back(convert(con[i]));
	}
	return res;
}

Point convert(const Point2f &in) {
	return Point(in.x, in.y);
}

Point2f convert(const Point &in) {
	return Point2f(in.x, in.y);
}

Scalar grayWhite() {
	return Scalar(255);
}
Scalar whiteColor() {
	return Scalar(255, 255, 255);
}

Scalar blackColor() {
	return Scalar(0, 0, 0);
}

Scalar redColor() {
	return Scalar(0, 0, 255);
}

Scalar randColor() {

	return Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
}
Scalar yellowColor() {
	return Scalar(0, 255, 255);
}

Scalar blueColor() {
	return Scalar(255, 0, 0);
}

Scalar greenColor() {
	return Scalar(0, 255, 0);
}

