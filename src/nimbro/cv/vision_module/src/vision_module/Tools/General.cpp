// General Functions for Vision Module
// Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <vision_module/Tools/General.hpp>

RNG rng(12345);

/// <summary>
/// Rotates the coordinate axises and returns the new coordinate of insetreted point
/// </summary>
/// <param name="alpha">angle (in degree) to rotate the coordinate axises</param>
/// <param name="p">the point in old coordinate axis</param>
/// <returns>the point in new coordinate axis</returns>
cv::Point2f RotateCoordinateAxis(double alpha, cv::Point2f p)
{
	alpha = Degree2Radian(alpha);
	return cv::Point2f((float) (p.x * cos(alpha) - p.y * sin(alpha)),
			(float) (p.x * sin(alpha) + p.y * cos(alpha)));
}

/// <summary>
/// Rotates the a pQuery arount pCenter whith alpha degree
/// </summary>
cv::Point2f RotateAroundPoint(cv::Point2f pQuery, double alpha,
		cv::Point2f pCenter)
{

	cv::Point2f p = pQuery - pCenter;
	return RotateCoordinateAxis(-alpha, p) + pCenter;
}

/// <summary>
/// Rotates the a pQuery arount pCenter whith alpha degree
/// </summary>
cv::Point2f RotateAroundPoint(cv::Point2f pQuery, double alpha)
{
	cv::Point2f pCenter;
	cv::Point2f p = pQuery - pCenter;
	return RotateCoordinateAxis(-alpha, p) + pCenter;
}

cv::Rect clipRect(cv::Rect _in/*Should be a positive Rect*/,
		cv::Rect _box/*Should be a positive Rect*/)
{
	double minXPossible = _box.x;
	double minYPossible = _box.y;
	double maxXPossible = _box.x + _box.width - 1;
	double maxYPossible = _box.y + _box.height - 1;

	double _inMaxX = min_n(maxXPossible, max_n(minXPossible, _in.x+_in.width));
	double _inMaxY = min_n(maxYPossible, max_n(minYPossible, _in.y+_in.height));

	_in.x = min_n(maxXPossible, max_n(minXPossible, _in.x));
	_in.y = min_n(maxYPossible, max_n( minYPossible, _in.y));

	_in.width = _inMaxX - _in.x;
	_in.height = _inMaxY - _in.y;
	return _in;
}

CircleC mergeCircleC(CircleC A, CircleC B)
{
	double angle = atan2(B.Center.y - A.Center.y, B.Center.x - A.Center.x);
	Point2f a = Point2f((B.Center.x + cos(angle) * B.Radius),
			(B.Center.y + sin(angle) * B.Radius));
	angle += M_PI;
	Point2f b = Point2f((A.Center.x + cos(angle) * A.Radius),
			(A.Center.y + sin(angle) * A.Radius));
	float rad = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2)) / 2.0;
	if (rad < A.Radius)
	{
		return A;
	}
	else if (rad < B.Radius)
	{
		return B;
	}
	else
	{
		return CircleC(Point2f(((a.x + b.x) / 2.0), ((a.y + b.y) / 2.0)), rad);
	}
}

bool hasIntersection(CircleC A, CircleC B)
{
	return (GetDistance(A.Center, B.Center) <= A.Radius + B.Radius);
}

bool lineFormIntersectionWithCircleC(LineSegment l, CircleC c, Point2f &pr1,
		Point2f &pr2)
{

	float Ax = l.P1.x;
	float Ay = l.P1.y;
	float Bx = l.P2.x;
	float By = l.P2.y;
	float Cx = c.Center.x;
	float Cy = c.Center.y;
	float R = c.Radius;

	// compute the euclidean distance between A and B
	float LAB = sqrt(pow(Bx - Ax, 2) + pow(By - Ay, 2));

	if(abs(LAB) < /*EPS*/1e-8)
	{
		return false;//l is very short
	}

	// compute the direction vector D from A to B
	float Dx = (Bx - Ax) / LAB;
	float Dy = (By - Ay) / LAB;

	// Now the line equation is x = Dx*t + Ax, y = Dy*t + Ay with 0 <= t <= 1.

	// compute the value t of the closest point to the circle center (Cx, Cy)
	float t = Dx * (Cx - Ax) + Dy * (Cy - Ay);

	// This is the projection of C on the line from A to B.

	// compute the coordinates of the point E on line and closest to C
	float Ex = t * Dx + Ax;
	float Ey = t * Dy + Ay;

	// compute the euclidean distance from E to C
	float LEC = sqrt(pow(Ex - Cx, 2) + pow(Ey - Cy, 2));

	// test if the line intersects the circle
	if (LEC < R)
	{
		// compute distance from t to circle intersection point
		float dt = sqrt(pow(R, 2) - pow(LEC, 2));

		// compute first intersection point
		float Fx = (t - dt) * Dx + Ax;
		float Fy = (t - dt) * Dy + Ay;

		// compute second intersection point
		float Gx = (t + dt) * Dx + Ax;
		float Gy = (t + dt) * Dy + Ay;

		pr1.x = Fx;
		pr1.y = Fy;

		pr2.x = Gx;
		pr2.y = Gy;
		return true;
	}

	// else test if the line is tangent to circle
	else if (LEC == R)
	{
		// tangent point to circle is E
		pr1.x = Ex;
		pr1.y = Ey;
		pr2 = pr1;
		return true;
	}
	else
	{
		// line doesn't touch circle
		return false;
	}

}

bool minDimentionRect(int _min, cv::Rect &_in/*Should be a positive Rect*/)
{
	bool didItChange = false;
	if (_in.width < _min)
	{
		double midX = _in.x + (_in.width / 2.);
		_in.x = midX - (_min / 2.);
		_in.width = _min;
		didItChange = true;
	}
	if (_in.height < _min)
	{
		double midY = _in.y + (_in.height / 2.);
		_in.y = midY - (_min / 2.);
		_in.height = _min;
		didItChange = true;
	}
	return didItChange;
}

bool maxDimentionRect(int _max, cv::Rect &_in/*Should be a positive Rect*/)
{
	bool didItChange = false;
	if (_in.width > _max)
	{
		double midX = _in.x + (_in.width / 2.);
		_in.x = midX - (_max / 2.);
		_in.width = _max;
		didItChange = true;
	}
	if (_in.height > _max)
	{
		double midY = _in.y + (_in.height / 2.);
		_in.y = midY - (_max / 2.);
		_in.height = _max;
		didItChange = true;
	}
	return didItChange;
}

bool mergeRect(cv::Rect &_in/*Should be a positive Rect*/,
		cv::Rect _in2/*Should be a positive Rect*/)
{
	bool didItChange = false;
	Point st = Point(min_n(_in.x, _in2.x), min_n(_in.y, _in2.y));
	Point en = Point(max_n(_in.x + _in.width, _in2.x + _in2.width),
			max_n(_in.y + _in.height, _in2.y + _in2.height));

	int width = (en.x - st.x);
	int heigth = (en.y - st.y);
	int mergeArea = width * heigth;
	if (mergeArea < (_in.width * _in.height + _in2.width * _in2.height))
	{
		_in.x = st.x;
		_in.y = st.y;
		_in.width = width;
		_in.height = heigth;
		didItChange = true;
	}

	return didItChange;
}

cv::Rect asymetricScaleRec(double _xLeftRatio, double _xRightRatio,
		double _yTopRatio, double _yDownRatio,
		cv::Rect _in/*Should be a positive Rect*/)
{
	double halfX = _in.width / 2.;
	double halfY = _in.height / 2.;

	double midX = _in.x + (halfX);
	double midY = _in.y + (halfY);

	_in.x = midX - (halfX * _xLeftRatio);
	_in.y = midY - (halfY * _yTopRatio);

	double rightX = midX + (halfX * _xRightRatio);
	double downY = midY + (halfY * _yDownRatio);

	_in.width = rightX - _in.x;
	_in.height = downY - _in.y;

	return _in;
}

cv::Rect asymetricScaleRec(double _xLeftRatio, double _xRightRatio,
		double _yTopRatio, double _yDownRatio,
		cv::Rect _in/*Should be a positive Rect*/,
		cv::Rect _box/*Should be a positive Rect*/)
{

	return clipRect(
			asymetricScaleRec(_xLeftRatio, _xRightRatio, _yTopRatio,
					_yDownRatio, _in), _box);
}

cv::Rect scaleRec(double _xRatio, double _yRatio,
		cv::Rect _in/*Should be a positive Rect*/)
{
	return asymetricScaleRec(_xRatio, _xRatio, _yRatio, _yRatio, _in);
}
cv::Rect scaleRec(double _xRatio, double _yRatio,
		cv::Rect _in/*Should be a positive Rect*/,
		cv::Rect _box/*Should be a positive Rect*/)
{
	return clipRect(scaleRec(_xRatio, _yRatio, _in), _box);
}

double dot(Point3f c1, Point3f c2)
{
	return (c1.x * c2.x + c1.y * c2.y + c1.z * c2.z);
}

cv::Rect CircleCToRect(CircleC circ)
{
	return cv::Rect(circ.Center.x - circ.Radius, circ.Center.y - circ.Radius,
			circ.Radius * 2, circ.Radius * 2);
}

CircleC RectToCircleC(cv::Rect rec)
{
	return CircleC(Point2f(rec.x + rec.width / 2., rec.y + rec.height / 2.),
			min(rec.width / 2., rec.height / 2.));
}
//http://www.juergenwiki.de/work/wiki/doku.php?id=public:hog_descriptor_computation_and_visualization
Mat get_hogdescriptor_visual_image(Mat& origImg,
		vector<float>& descriptorValues, Size winSize, Size cellSize,
		int scaleFactor, double viz_factor)
{
	Mat visual_image;
	resize(origImg, visual_image,
			Size(origImg.cols * scaleFactor, origImg.rows * scaleFactor));

	int gradientBinSize = 9;
	// dividing 180° into 9 bins, how large (in rad) is one bin?
	float radRangeForOneBin = 3.14 / (float) gradientBinSize;

	// prepare data structure: 9 orientation / gradient strenghts for each cell
	int cells_in_x_dir = winSize.width / cellSize.width;
	int cells_in_y_dir = winSize.height / cellSize.height;
	float*** gradientStrengths = new float**[cells_in_y_dir];
	int** cellUpdateCounter = new int*[cells_in_y_dir];
	for (int y = 0; y < cells_in_y_dir; y++)
	{
		gradientStrengths[y] = new float*[cells_in_x_dir];
		cellUpdateCounter[y] = new int[cells_in_x_dir];
		for (int x = 0; x < cells_in_x_dir; x++)
		{
			gradientStrengths[y][x] = new float[gradientBinSize];
			cellUpdateCounter[y][x] = 0;

			for (int bin = 0; bin < gradientBinSize; bin++)
				gradientStrengths[y][x][bin] = 0.0;
		}
	}

	// nr of blocks = nr of cells - 1
	// since there is a new block on each cell (overlapping blocks!) but the last one
	int blocks_in_x_dir = cells_in_x_dir - 1;
	int blocks_in_y_dir = cells_in_y_dir - 1;

	// compute gradient strengths per cell
	int descriptorDataIdx = 0;

	for (int blockx = 0; blockx < blocks_in_x_dir; blockx++)
	{
		for (int blocky = 0; blocky < blocks_in_y_dir; blocky++)
		{
			// 4 cells per block ...
			for (int cellNr = 0; cellNr < 4; cellNr++)
			{
				// compute corresponding cell nr
				int cellx = blockx;
				int celly = blocky;
				if (cellNr == 1)
					celly++;
				if (cellNr == 2)
					cellx++;
				if (cellNr == 3)
				{
					cellx++;
					celly++;
				}

				for (int bin = 0; bin < gradientBinSize; bin++)
				{
					float gradientStrength = descriptorValues[descriptorDataIdx];
					descriptorDataIdx++;

					gradientStrengths[celly][cellx][bin] += gradientStrength;

				} // for (all bins)

				// note: overlapping blocks lead to multiple updates of this sum!
				// we therefore keep track how often a cell was updated,
				// to compute average gradient strengths
				cellUpdateCounter[celly][cellx]++;

			} // for (all cells)

		} // for (all block x pos)
	} // for (all block y pos)

	// compute average gradient strengths
	for (int celly = 0; celly < cells_in_y_dir; celly++)
	{
		for (int cellx = 0; cellx < cells_in_x_dir; cellx++)
		{

			float NrUpdatesForThisCell = (float) cellUpdateCounter[celly][cellx];

			// compute average gradient strenghts for each gradient bin direction
			for (int bin = 0; bin < gradientBinSize; bin++)
			{
				gradientStrengths[celly][cellx][bin] /= NrUpdatesForThisCell;
			}
		}
	}

	//cout << "descriptorDataIdx = " << descriptorDataIdx << endl;

	// draw cells
	for (int celly = 0; celly < cells_in_y_dir; celly++)
	{
		for (int cellx = 0; cellx < cells_in_x_dir; cellx++)
		{
			int drawX = cellx * cellSize.width;
			int drawY = celly * cellSize.height;

			int mx = drawX + cellSize.width / 2;
			int my = drawY + cellSize.height / 2;

			rectangle(visual_image,
					Point(drawX * scaleFactor, drawY * scaleFactor),
					Point((drawX + cellSize.width) * scaleFactor,
							(drawY + cellSize.height) * scaleFactor),
					CV_RGB(100, 100, 100), 1);

			// draw in each cell all 9 gradient strengths
			for (int bin = 0; bin < gradientBinSize; bin++)
			{
				float currentGradStrength = gradientStrengths[celly][cellx][bin];

				// no line to draw?
				if (currentGradStrength == 0)
					continue;

				float currRad = bin * radRangeForOneBin + radRangeForOneBin / 2;

				float dirVecX = cos(currRad);
				float dirVecY = sin(currRad);
				float maxVecLen = cellSize.width / 2;
				float scale = viz_factor; // just a visual_imagealization scale,
										  // to see the lines better

				// compute line coordinates
				float x1 = mx
						- dirVecX * currentGradStrength * maxVecLen * scale;
				float y1 = my
						- dirVecY * currentGradStrength * maxVecLen * scale;
				float x2 = mx
						+ dirVecX * currentGradStrength * maxVecLen * scale;
				float y2 = my
						+ dirVecY * currentGradStrength * maxVecLen * scale;

				// draw gradient visual_imagealization
				line(visual_image, Point(x1 * scaleFactor, y1 * scaleFactor),
						Point(x2 * scaleFactor, y2 * scaleFactor),
						CV_RGB(0, 0, 255), 1);

			} // for (all bins)

		} // for (cellx)
	} // for (celly)

	// don't forget to free memory allocated by helper data structures!
	for (int y = 0; y < cells_in_y_dir; y++)
	{
		for (int x = 0; x < cells_in_x_dir; x++)
		{
			delete[] gradientStrengths[y][x];
		}
		delete[] gradientStrengths[y];
		delete[] cellUpdateCounter[y];
	}
	delete[] gradientStrengths;
	delete[] cellUpdateCounter;

	return visual_image;
}
cv::Scalar pinkColor()
{
	return cv::Scalar(255, 0, 255);
}

cv::Scalar blackGary()
{
	return cv::Scalar(0);
}

cv::Scalar pinkMeloColor()
{
	return cv::Scalar(183, 50, 130);
}

// Copyright 2001 softSurfer, 2012 Dan Sunday===============================
#define SMALL_NUM   0.00000001 // anything that avoids division overflow
// dot product (3D) which allows vector operations in arguments
#define d(u,v)     norm(u-v)        // distance = norm of difference
#define abs(x)     ((x) >= 0 ? (x) : -(x))   //  absolute value
#define sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )

double norm(Point3f c1)
{
	return std::sqrt(dot(c1, c1));
}
// dist3D_Segment_to_Segment(): get the 3D minimum distance between 2 segments
//    Input:  two 3D line segments S1 and S2
//    Return: the shortest distance between S1 and S2
float dist3D_Segment_to_Segment(LineSegment S1, LineSegment S2)
{

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
	if (D < SMALL_NUM)
	{ // the lines are almost parallel
		sN = 0.0;         // force using point P0 on segment S1
		sD = 1.0;         // to prevent possible division by 0.0 later
		tN = e;
		tD = c;
	}
	else
	{                 // get the closest points on the infinite lines
		sN = (b * e - c * d);
		tN = (a * e - b * d);
		if (sN < 0.0)
		{        // sc < 0 => the s=0 edge is visible
			sN = 0.0;
			tN = e;
			tD = c;
		}
		else if (sN > sD)
		{  // sc > 1  => the s=1 edge is visible
			sN = sD;
			tN = e + b;
			tD = c;
		}
	}

	if (tN < 0.0)
	{            // tc < 0 => the t=0 edge is visible
		tN = 0.0;
		// recompute sc for this edge
		if (-d < 0.0)
			sN = 0.0;
		else if (-d > a)
			sN = sD;
		else
		{
			sN = -d;
			sD = a;
		}
	}
	else if (tN > tD)
	{      // tc > 1  => the t=1 edge is visible
		tN = tD;
		// recompute sc for this edge
		if ((-d + b) < 0.0)
			sN = 0;
		else if ((-d + b) > a)
			sN = sD;
		else
		{
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

double GetDistance(Point2d p)
{
	return sqrt((p.x * p.x) + (p.y * p.y));
}
float GetDistance(Point2f p)
{
	return sqrt((p.x * p.x) + (p.y * p.y));
}

double GetDistance(Point2d p, Point2d p2)
{
	double x = abs(p.x - p2.x);
	double y = abs(p.y - p2.y);
	return sqrt(x * x + y * y);
}

Point2d GetCenterMinBounding(vector<Point> con)
{
	return minAreaRect(con).center;
}

Point2d GetCenter(Rect rec)
{
	Point2d b(rec.x + (rec.width / 2.), rec.y + (rec.height / 2.));
	return b;
}

Point2d GetCenterBoundingRec(vector<Point> con)
{
	Rect bounding_rect = boundingRect(con);
	Point2d b(bounding_rect.x + (bounding_rect.width / 2.),
			bounding_rect.y + (bounding_rect.height / 2.));
	return b;
}

void RotateAroundPoint(Point2d pQuery, double alpha, Point2d &res)
{
	Point2d pCenter(0, 0);
	Point2d p = pQuery - pCenter;
	Point2d resRot;
	RotateCoordinateAxis(-alpha, p, resRot);
	res = resRot + pCenter;
}

void RotateCoordinateAxis(double alpha, Point2d p, Point2d &res)
{
	alpha = Degree2Radian(alpha);
	res = Point2d((float) (p.x * cos(alpha) - p.y * sin(alpha)),
			(float) (p.x * sin(alpha) + p.y * cos(alpha)));
}

//Normalize to [0,360):
double CorrectAngleDegree360(double x)
{
	x = fmod(x, 360);
	if (x < 0)
		x += 360;
	return x;
}

//Normalize to [-180,180)
double CorrectAngleDegree180(double x)
{
	x = fmod(x + 180, 360);
	if (x < 0)
		x += 360;
	return x - 180;
}

//Normalize to [0,360):
double CorrectAngleRadian360(double x)
{
	return Degree2Radian((CorrectAngleDegree360((Radian2Degree(x)))));
}

//Normalize to [-180,180)
double CorrectAngleRadian180(double x)
{
	return Degree2Radian((CorrectAngleDegree180((Radian2Degree(x)))));
}

//Normalize to [-180,180)
float AngleDiffDegree180(float first, float second)
{
	return CorrectAngleDegree180(first - second);
}

//Normalize to [-180,180)
float AngleDiffRadian180(float first, float second)
{
	return CorrectAngleRadian180(first - second);
}

void drawLine(Mat img, Vec4f line, int thickness, Scalar color)
{
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

double Radian2Degree(double r)
{
	return (r / M_PI) * (180);
}

double Degree2Radian(double d)
{
	return (d * M_PI) / (180);
}

void lowPass(Point3d newrec, Point3d &res, double coef)
{
	double coefIn = 1 - coef;
	res.x = (coefIn * res.x) + (coef * newrec.x);
	res.y = (coefIn * res.y) + (coef * newrec.y);
	res.z = (coefIn * res.z) + (coef * newrec.z);
}
void lowPass(Point2d newrec, Point2d &res, double coef)
{
	double coefIn = 1 - coef;
	res.x = (coefIn * res.x) + (coef * newrec.x);
	res.y = (coefIn * res.y) + (coef * newrec.y);
}

void lowPass(double newrec, double &res, double coef)
{
	double coefIn = 1 - coef;
	res = (coefIn * res) + (coef * newrec);
}

void lowPassAngleDegree180(double newrec, double &res, double coef)
{
	double angleDiff = AngleSubDegree180(res, newrec);
	res = CorrectAngleDegree180(res - (coef * angleDiff));
}

double AngleSubDegree180(double first, double second)
{
	double angDiff = first - second;
	angDiff += (angDiff > 180) ? -360 : (angDiff < -180) ? 360 : 0;
	return angDiff;
}

int Left(Rect rec)
{

	return rec.x;
}

int Top(Rect rec)
{

	return rec.y;
}

int Right(Rect rec)
{

	return rec.x + rec.width;
}

int Bottom(Rect rec)
{

	return rec.y + rec.height;
}

bool MergeLinesMax(vector<LineSegment> resLinesReal, double maxDegree,
		double maxDistance, vector<LineSegment> &clusteredLines, Rect box,
		bool useBounding)
{
//	ROS_WARN("()()*()()***** Start *****()()*()()");
	if (!MergeLinesOnce(resLinesReal, maxDegree, maxDistance, clusteredLines,
			box, useBounding))
	{
		return false;
	}
	vector<LineSegment> before;
	before.reserve(resLinesReal.size());
//	int i = 0;
	do
	{
//		ROS_WARN("()()*()()***** i th = %d *****()()*()()", i++);
		before = clusteredLines;
		if (!MergeLinesOnce(before, maxDegree, maxDistance, clusteredLines, box,
				useBounding))
		{
			return false;
		}
	} while (before.size() > clusteredLines.size());
//	ROS_WARN("()()*()()***** Finish *****()()*()()");
	return true;
}

bool MergeLinesOnce(vector<LineSegment> resLinesReal, double maxDegree,
		double maxDistance, vector<LineSegment> &clusteredLines, Rect box,
		bool useBounding)
{
	if (resLinesReal.size() < 1)
	{
		return false;
	}
	clusteredLines.clear();
	clusteredLines.reserve(resLinesReal.size());
	vector<Point2f> pointVector(4);
	for (size_t i = 0; i < resLinesReal.size(); i++)
	{
		int index = -1;
		for (size_t j = 0; j < clusteredLines.size(); j++)
		{
			double diffAngle = resLinesReal[i].GetAbsMinAngleDegree(
					clusteredLines[j]);

			if ((diffAngle < maxDegree)
					&& (dist3D_Segment_to_Segment(resLinesReal[i],
							clusteredLines[j]) < maxDistance))
			{
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
			}
			else
			{
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
			Point2f mergedMidPoint;
			LineSegment mergedLine;

			LineSegment queryLine1(clusteredLines[index]);
			LineSegment queryLine2(resLinesReal[i]);

			pointVector[0] = (queryLine1.P1);
			pointVector[1] = (queryLine1.P2);

			pointVector[2] = (queryLine2.P1);
			pointVector[3] = (queryLine2.P2);
			float queryLineLen1 = queryLine1.GetLength();
			float queryLineLen2 = queryLine2.GetLength();
			if (!useBounding)
			{

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
				for (int i = 0; i < 4; i++)
				{
					for (int j = i + 1; j < 4; j++)
					{
						Point2f p1 = mergedLine.GetClosestPointOnLineSegment(
								pointVector[i]);
						Point2f p2 = mergedLine.GetClosestPointOnLineSegment(
								pointVector[j]);
						double distance = LineSegment(p1, p2).GetLength();
						if (distance > maxDistance)
						{
							maxDistance = distance;
							biggestDistanceLS.P1 = p1;
							biggestDistanceLS.P2 = p2;
						}
					}
				}
				mergedLine = biggestDistanceLS;
			}
			else
			{
				RotatedRect minRec = minAreaRect(pointVector);
				Point2f allPoints[4];
				minRec.points(allPoints);
				if (LineSegment(allPoints[2], allPoints[1]).GetLength()
						> LineSegment(allPoints[2], allPoints[3]).GetLength())
				{
					mergedLine.P1 = GetAverage(allPoints[2], allPoints[3]);
					mergedLine.P2 = GetAverage(allPoints[0], allPoints[1]);
				}
				else
				{
					mergedLine.P1 = GetAverage(allPoints[2], allPoints[1]);
					mergedLine.P2 = GetAverage(allPoints[0], allPoints[3]);
				}
			}

			mergedLine.setProbability(
					(queryLine1.getProbability() * queryLineLen1
							+ queryLine2.getProbability() * queryLineLen2)
							/ (queryLineLen1 + queryLineLen2));

			mergedLine.Clip(box);

			clusteredLines[index].P1 = Point2f(mergedLine.P1.x,
					mergedLine.P1.y);
			clusteredLines[index].P2 = Point2f(mergedLine.P2.x,
					mergedLine.P2.y);

//			cout << " Merged Line = " << mergedLine.P1 << " " << mergedLine.P2
//					<< endl;
//			cout << "-- -- >      END     < -- --" << endl;
		}
		else
		{
			clusteredLines.push_back(resLinesReal[i]);
		}
	}

	return true;
}

Point2f AddP(Point2f a, float b)
{
	return Point2f(a.x + b, a.y + b);
}

bool MinXPoint(vector<Point2f> con, Point2f &res)
{
	float lx = 99999;
	int index = -1;
	for (size_t i = 0; i < con.size(); i++)
	{
		if (con[i].x < lx)
		{
			index = i;
			lx = con[i].x;
		}
	}
	if (index == -1)
	{
		return false;
	}
	res = con[index];
	return true;
}

bool MinYPoint(vector<Point2f> con, Point2f &res)
{
	float ly = 99999;
	int index = -1;
	for (size_t i = 0; i < con.size(); i++)
	{
		if (con[i].y < ly)
		{
			index = i;
			ly = con[i].y;
		}
	}
	if (index == -1)
	{
		return false;
	}
	res = con[index];
	return true;
}

bool MaxXPoint(vector<Point2f> con, Point2f &res)
{
	float mx = -99999;
	int index = -1;
	for (size_t i = 0; i < con.size(); i++)
	{
		if (con[i].x > mx)
		{
			index = i;
			mx = con[i].x;
		}
	}
	if (index == -1)
	{
		return false;
	}
	res = con[index];
	return true;
}

bool MaxYPoint(vector<Point2f> con, Point2f &res)
{
	float my = -99999;
	int index = -1;
	for (size_t i = 0; i < con.size(); i++) //lowestPoint
	{
		if (con[i].y > my)
		{
			index = i;
			my = con[i].y;
		}
	}
	if (index == -1)
	{
		return false;
	}
	res = con[index];
	return true;
}

bool MinXPoint(vector<Point> con, Point &res)
{
	int lx = 99999;
	int index = -1;
	for (size_t i = 0; i < con.size(); i++)
	{
		if (con[i].x < lx)
		{
			index = i;
			lx = con[i].x;
		}
	}
	if (index == -1)
	{
		return false;
	}
	res = con[index];
	return true;
}

bool MinYPoint(vector<Point> con, Point &res)
{
	int ly = 99999;
	int index = -1;
	for (size_t i = 0; i < con.size(); i++)
	{
		if (con[i].y < ly)
		{
			index = i;
			ly = con[i].y;
		}
	}
	if (index == -1)
	{
		return false;
	}
	res = con[index];
	return true;
}

bool MaxXPoint(vector<Point> con, Point &res)
{
	int mx = -99999;
	int index = -1;
	for (size_t i = 0; i < con.size(); i++)
	{
		if (con[i].x > mx)
		{
			index = i;
			mx = con[i].x;
		}
	}
	if (index == -1)
	{
		return false;
	}
	res = con[index];
	return true;
}

bool MaxYPoint(vector<Point> con, Point &res)
{
	int my = -99999;
	int index = -1;
	for (size_t i = 0; i < con.size(); i++) //lowestPoint
	{
		if (con[i].y > my)
		{
			index = i;
			my = con[i].y;
		}
	}
	if (index == -1)
	{
		return false;
	}
	res = con[index];
	return true;
}

Point2f GetAverage(Point2f p0, Point2f p1)
{
	return Point2f((p0.x + p1.x) / 2., (p0.y + p1.y) / 2.);
}

Point2f GetWeigthedAverage(Point2f p0, Point2f p1, float w0, float w1)
{
	float sumW = w0 + w1;
	return Point2f((p0.x * w0 + p1.x * w1) / sumW,
			(p0.y * w0 + p1.y * w1) / sumW);
}

float GetWeigthedAverage(float p0, float p1, float w0, float w1)
{
	float sumW = w0 + w1;
	return (p0 * w0 + p1 * w1) / sumW;
}

double Distance2point(Point2f begin, Point2f end)
{

	double x = (begin.x - end.x);
	double y = (begin.y - end.y);
	return sqrt((x * x) + (y * y));
}
// point distance from line segment
float DistanceFromLineSegment(LineSegment line, Point2f p)
{
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

double AngleBetween0to180(RotatedRect calculatedRect)
{
	if (calculatedRect.size.width < calculatedRect.size.height)
	{
		return calculatedRect.angle + 180;
	}
	else
	{
		return calculatedRect.angle + 90;
	}
}

bool checkBox_Size_Scale_Angle(const vector<Point2f> &con, double maxAngle,
		double minAngle, double maxRatio, double minRatio, double max_maxLen,
		double min_minLen, double max_minLen, double min_maxLen)
{
	RotatedRect rec = minAreaRect(con);
	double minlen = min(rec.size.height, rec.size.width);
	double maxlen = max(rec.size.height, rec.size.width);
	double ratio = maxlen / minlen;
	double angle = AngleBetween0to180(rec);

	if (maxAngle >= minAngle)
	{
		if (angle > maxAngle || angle < minAngle)
		{
			return false;
		}
	}
	else
	{
		if ((angle > maxAngle && angle < minAngle) || angle < 0 || angle > 180)
		{
			return false;
		}
	}

	if (ratio > maxRatio || ratio < minRatio)
	{
		return false;
	}

	if (maxlen > max_maxLen || maxlen < min_maxLen || minlen < min_minLen
			|| minlen > max_minLen)
	{
		return false;
	}

	return true;
}

bool calcHist3Channels(cv::Mat &colorMatRoi, cv::Mat &justObjMask,
		cv::Mat hist_base[3])
{
	// hue varies from 0 to 179, saturation from 0 to 255
	float ranges_fix[3][2] =
	{
	{ 0, 180 },
	{ 0, 256 },
	{ 0, 256 } };
	/// Using  bins
	int bins[3] =
	{ 32, 32, 32 };

	for (int cnlIdx = 0; cnlIdx < 3; cnlIdx++) //HSV channels
	{
		int channelss[] =
		{ cnlIdx };
		const float* ranges[] =
		{ ranges_fix[cnlIdx] };
		int histSize[] =
		{ bins[cnlIdx] };

		cv::calcHist(&colorMatRoi, 1, channelss, justObjMask, hist_base[cnlIdx],
				1, histSize, ranges, true, false);
		cv::normalize(hist_base[cnlIdx].clone(), hist_base[cnlIdx], 0, 1,
				cv::NORM_MINMAX, -1, cv::Mat());

	}

	return true;
}

vector<Point> convert(const vector<Point2f> &con, vector<Point> res)
{
	res.resize(con.size());
	for (size_t i = 0; i < con.size(); i++)
	{
		res[i] = convert(con[i]);
	}
	return res;
}

vector<Point2f> convert(const vector<Point> &con, vector<Point2f> res)
{
	res.resize(con.size());
	for (size_t i = 0; i < con.size(); i++)
	{
		res[i] = convert(con[i]);
	}
	return res;
}

Point convert(const Point2f &in)
{
	return Point(in.x, in.y);
}

Point2f convert(const Point &in)
{
	return Point2f(in.x, in.y);
}

Scalar grayWhite()
{
	return Scalar(255);
}
Scalar whiteColor()
{
	return Scalar(255, 255, 255);
}

Scalar blackColor()
{
	return Scalar(0, 0, 0);
}

Scalar darkOrangeColor()
{
	return Scalar(48, 163, 201);
}

Scalar redColor()
{
	return Scalar(0, 0, 255);
}

Scalar redMeloColor()
{
	return Scalar(51, 123, 255);
}

Scalar randColor()
{

	return Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
}
Scalar yellowColor()
{
	return Scalar(0, 255, 255);
}

Scalar blueColor()
{
	return Scalar(255, 0, 0);
}

Scalar blueMeloColor()
{
	return Scalar(255, 153, 51);
}

Scalar greenColor()
{
	return Scalar(0, 255, 0);
}

