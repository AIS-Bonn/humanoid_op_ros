// Compute IPM
// Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <vision_module/Projections/IPM.hpp>

bool IPM::GetHomographyUseYaw(float diagonalAngleView,Point3d cameraLocation, Point3d cameraOrintation,
		Mat &realHomoFor, Mat &realHomoBack, Point2d outerCornetrsRes[4],
		Point2d gPoints[4], Point3d transformedPoints[4],
		Point2d physicalCornersRes[4])
{
	InitIPM(diagonalAngleView);
	Point2d cPoints[4];

	if (!GetPoints(cameraLocation, cameraOrintation, gPoints, cPoints,
			transformedPoints, true))
	{
		return false;
	}

	Point2d _upRightPoint = gPoints[2];
	Point2d _upLeftPoint = gPoints[3];
	Point2d _downRightPoint = gPoints[1];
	Point2d _downLeftPoint = gPoints[0];

	int tmpOffset = (int) ((params.topView.width->get()
			/ params.topView.scale->get()) / 2.);

	_upRightPoint.y += tmpOffset;
	_downRightPoint.y += tmpOffset;
	_downLeftPoint.y += tmpOffset;
	_upLeftPoint.y += tmpOffset;

	vector<Point2d> outerCornetrs(4), physicalCorners(4);

	physicalCorners[0] = _downLeftPoint;
	physicalCorners[1] = _downRightPoint;
	physicalCorners[2] = _upRightPoint;
	physicalCorners[3] = _upLeftPoint;

	double scale = params.camera.widthUnDistortion->get()
			/ abs(cPoints[1].y - cPoints[0].y);

	outerCornetrs[0].x = 0;
	outerCornetrs[0].y = params.camera.heightUnDistortion->get() - 1;

	outerCornetrs[1].x = params.camera.widthUnDistortion->get() - 1;
	outerCornetrs[1].y = params.camera.heightUnDistortion->get() - 1;

	outerCornetrs[2].x = params.camera.widthUnDistortion->get() - 1;
	outerCornetrs[2].y = params.camera.heightUnDistortion->get() - 1
			- scale * abs(cPoints[2].x - cPoints[1].x);

	outerCornetrs[3].x = 0;
	outerCornetrs[3].y = params.camera.heightUnDistortion->get() - 1
			- scale * abs(cPoints[3].x - cPoints[0].x);

	for (int i = 0; i < 4; i++)
	{
		if (outerCornetrs[i].x < 0
				|| outerCornetrs[i].x >= params.camera.widthUnDistortion->get())
		{
			return false;
		}
		if (outerCornetrs[i].y < 0
				|| outerCornetrs[i].y
						>= params.camera.heightUnDistortion->get())
		{
			return false;
		}
	}
	vector<Point2d> physicalCornersCalib(4);

	physicalCornersCalib[0] = _downLeftPoint;
	physicalCornersCalib[1] = _downRightPoint;
	physicalCornersCalib[2] = _upRightPoint;
	physicalCornersCalib[3] = _upLeftPoint;
	double ofY = 0;
	double ofX = ((params.topView.width->get() / params.topView.scale->get())
			/ 2.) - 0;
	physicalCornersCalib[0].y += ofY;
	physicalCornersCalib[1].y += ofY;
	physicalCornersCalib[2].y += ofY;
	physicalCornersCalib[3].y += ofY;

	physicalCornersCalib[0].x += ofX;
	physicalCornersCalib[1].x += ofX;
	physicalCornersCalib[2].x += ofX;
	physicalCornersCalib[3].x += ofX;

	realHomoFor = findHomography(outerCornetrs, physicalCornersCalib);
	realHomoBack = findHomography(physicalCornersCalib, outerCornetrs);

	for (int i = 0; i <= 3; i++)
	{
		outerCornetrsRes[i] = outerCornetrs[i];
	}
	return true;
}

bool IPM::GetHomographyNoYaw(float diagonalAngleView,Point3d cameraLocation, Point3d cameraOrintation,
		Mat &topHomoFor, Mat &topHomoBack, double &picYawDegree)
{
	InitIPM(diagonalAngleView);
	Point2d cPoints[4];
	Point2d gPoints[4];
	Point3d transformedPoints[4];
	if (!GetPoints(cameraLocation, cameraOrintation, gPoints, cPoints,
			transformedPoints, false))
	{
		return false;
	}

	LineSegment lowerLine(Point2d(gPoints[0].x, -gPoints[0].y),
			Point2d(gPoints[1].x, -gPoints[1].y));

	LineSegment horLine(Point2d(0, 0), Point2d(0, 100));

	double rotateAngle = lowerLine.GetExteriorAngleDegree(horLine);

	picYawDegree = rotateAngle + Radian2Degree(cameraOrintation.z);

	for (int i = 0; i < 4; i++)
	{
		Point2d tmp(gPoints[i].x, -gPoints[i].y);
		Point2d res;
		RotateAroundPoint(tmp, -rotateAngle, res);
		gPoints[i] = res;
	}

	Point2d _upRightPoint = gPoints[2];
	Point2d _upLeftPoint = gPoints[3];
	Point2d _downRightPoint = gPoints[1];
	Point2d _downLeftPoint = gPoints[0];

	int tmpOffset = (int) ((params.topView.width->get()
			/ params.topView.scale->get()) / 2.);

	_upRightPoint.y += tmpOffset;
	_downRightPoint.y += tmpOffset;
	_downLeftPoint.y += tmpOffset;
	_upLeftPoint.y += tmpOffset;

	vector<Point2d> outerCornetrs(4), physicalCorners(4);

	physicalCorners[0] = _downLeftPoint;
	physicalCorners[1] = _downRightPoint;
	physicalCorners[2] = _upRightPoint;
	physicalCorners[3] = _upLeftPoint;

	double scale = params.camera.widthUnDistortion->get()
			/ abs(cPoints[1].y - cPoints[0].y);

	outerCornetrs[0].x = 0;
	outerCornetrs[0].y = params.camera.heightUnDistortion->get() - 1;

	outerCornetrs[1].x = params.camera.widthUnDistortion->get() - 1;
	outerCornetrs[1].y = params.camera.heightUnDistortion->get() - 1;

	outerCornetrs[2].x = params.camera.widthUnDistortion->get() - 1;
	outerCornetrs[2].y = params.camera.heightUnDistortion->get() - 1
			- scale * abs(cPoints[2].x - cPoints[1].x);

	outerCornetrs[3].x = 0;
	outerCornetrs[3].y = params.camera.heightUnDistortion->get() - 1
			- scale * abs(cPoints[3].x - cPoints[0].x);
	for (int i = 0; i < 4; i++)
	{
		if (outerCornetrs[i].x < 0
				|| outerCornetrs[i].x >= params.camera.widthUnDistortion->get())
		{
			return false;
		}
		if (outerCornetrs[i].y < 0
				|| outerCornetrs[i].y
						>= params.camera.heightUnDistortion->get())
		{
			return false;
		}
	}
	topHomoFor = findHomography(outerCornetrs, physicalCorners);
	topHomoBack = findHomography(physicalCorners, outerCornetrs);

	return true;
}

Point3d IPM::GetPointOnZPlain(Point3d point1, Point3d point2, double z)
{
	point1.z -= z;
	point2.z -= z;
	Point3d res;

	double t = (point1.z) / (point2.z - point1.z);
	res.x = point1.x - (point2.x - point1.x) * t;
	res.y = point1.y - (point2.y - point1.y) * t;
	res.z = z;
	return res;
}

bool IPM::GetPoints(Point3d cameraLocation, Point3d cameraOrintation,
		Point2d gPoints[4], Point2d cPoints[4], Point3d transformedPoints[4],
		bool useYaw)
{
	if (!useYaw)
	{
		cameraOrintation.z = 0;
	}
	cameraLocation.x /= params.topView.scale->get() * 0.01;
	cameraLocation.y /= params.topView.scale->get() * 0.01;
	cameraLocation.z /= params.topView.scale->get() * 0.01;

	Mat _points = Mat(3, 4, DataType<double>::type, -1.0);
	_points.at<double>(0, 0) = -ta;
	_points.at<double>(1, 0) = +tb;

	_points.at<double>(0, 1) = -ta;
	_points.at<double>(1, 1) = -tb;

	_points.at<double>(0, 2) = +ta;
	_points.at<double>(1, 2) = -tb;

	_points.at<double>(0, 3) = +ta;
	_points.at<double>(1, 3) = +tb;

	for (int i = 0; i < 4; i++)
	{
		cPoints[i].x = _points.at<double>(0, i);
		cPoints[i].y = _points.at<double>(1, i);
	}

	Mat m = GetMultiplyExtrinsicRotationMatrix(cameraOrintation, false);
	Mat mInverse = GetMultiplyExtrinsicRotationMatrix(cameraOrintation * -1,
			true);

	for (int i = 0; i < 4; i++)
	{
		Mat subMat = _points(Rect(i, 0, 1, 3));
		Mat resTmp = m * subMat;
		transformedPoints[i] = Point3d(resTmp);
		transformedPoints[i] += cameraLocation;
	}

	//0cut capture picture after 90 degree

	Point3d up = transformedPoints[3];
	Point3d dp = transformedPoints[0];
	if (up.z - cameraLocation.z >= 0)
	{
		// lp: limmited upper-left point
		Point3d lp = GetPointOnZPlain(up, dp, cameraLocation.z - 0.0001);
		transformedPoints[3] = lp;

		lp -= cameraLocation;

		Mat lpRes = mInverse * Mat(lp);
		cPoints[3].x = lpRes.at<double>(0, 0);
		cPoints[3].y = lpRes.at<double>(0, 1);
		if (cPoints[3].x < cPoints[0].x)
		{
			return false;
		}
	}
	up = transformedPoints[2];
	dp = transformedPoints[1];
	if (up.z - cameraLocation.z >= 0)
	{
		// lp: limmited upper-right point
		Point3d lp = GetPointOnZPlain(up, dp, cameraLocation.z - 0.0001);
		transformedPoints[2] = lp;

		lp -= cameraLocation;

		Mat lpRes = mInverse * Mat(lp);
		cPoints[2].x = lpRes.at<double>(0, 0);
		cPoints[2].y = lpRes.at<double>(0, 1);

		if (cPoints[2].x < cPoints[1].x)
		{
			return false;
		}
	}

	Point3d groundPoints[4];

	for (int i = 0; i < 4; i++)
	{
		groundPoints[i] = GetPointOnZPlain(cameraLocation,
				transformedPoints[i]);
	}

	for (int i = 0; i < 4; i++)
	{
		gPoints[i].x = groundPoints[i].x;
		gPoints[i].y = groundPoints[i].y;
	}
	return true;
}

Mat IPM::GetMultiplyExtrinsicRotationMatrix(Point3d vec, bool inv)
{
	double cosx = cos(vec.x);
	double cosy = cos(vec.y);
	double cosz = cos(vec.z);
	double sinx = sin(vec.x);
	double siny = sin(vec.y);
	double sinz = sin(vec.z);
	/*
	 * m11  m12 m13
	 * m21  m22 m23
	 * m31  m32 m33
	 */
	Mat mat = Mat(3, 3, DataType<double>::type, 1);
	setIdentity(mat);

	Mat rX = Mat(3, 3, DataType<double>::type, 1);
	Mat rY = Mat(3, 3, DataType<double>::type, 1);
	Mat rZ = Mat(3, 3, DataType<double>::type, 1);

	setIdentity(rX);
	setIdentity(rY);
	setIdentity(rZ);

	rX.at<double>(1, 1) = cosx;
	rX.at<double>(1, 2) = -sinx;
	rX.at<double>(2, 1) = sinx;
	rX.at<double>(2, 2) = cosx;

	rY.at<double>(0, 0) = cosy;
	rY.at<double>(0, 2) = siny;
	rY.at<double>(2, 0) = -siny;
	rY.at<double>(2, 2) = cosy;

	rZ.at<double>(0, 0) = cosz;
	rZ.at<double>(0, 1) = -sinz;
	rZ.at<double>(1, 0) = sinz;
	rZ.at<double>(1, 1) = cosz;
	if (!inv)
	{
		mat = rZ * rX;
		mat = mat * rY;
	}
	else
	{
		mat = rY * rX;
		mat = mat * rZ;
	}

	return mat;
}

bool IPM::InitIPM(float diagonalAngleView)
{
	double w = params.camera.aspW->get();
	double h = params.camera.aspH->get();

	double w_2 = w / 2.0;
	double h_2 = h / 2.0;

	double d = 1.0;

	double gama = Degree2Radian(diagonalAngleView)	/ 2.0;
	tg = tan(gama);

	double nr = tg;

	double nx = nr / sqrt(w_2 * w_2 + h_2 * h_2);
	double nw = nx * w_2;
	double nh = nx * h_2;

	double alfa = atan(nh / d);
	double beta = atan(nw / d);

	ta = tan(alfa);
	tb = tan(beta);

	return true;
}
