// Compute IPM
// Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#pragma once
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <boost/iterator/iterator_concepts.hpp>
#include <vector>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/LineSegment.hpp>
#include <vision_module/Inputs/Camera.hpp>
#include <vision_module/Tools/General.hpp>
using namespace std;
using namespace cv;

class IPM
{
public:
	inline IPM(float diagonalAngleView)
	{
		InitIPM( diagonalAngleView);
	}
	virtual ~IPM()
	{

	}
	bool GetHomographyNoYaw(float diagonalAngleView,Point3d cameraLocation, Point3d cameraOrintation,
			Mat &topHomoFor, Mat &topHomoBack,
			double &picYawDegree);
	bool GetHomographyUseYaw(float diagonalAngleView,Point3d cameraLocation, Point3d cameraOrintation,
			 Mat &realHomoFor,Mat &realHomoBack,
		 Point2d outerCornetrsRes[4], Point2d gPoints[4],
			Point3d transformedPoints[4], Point2d physicalCornersRes[4]);
private:
	double ta, tb, tg;
	Point3d GetPointOnZPlain(Point3d point1, Point3d point2, double z = 0);
	Mat GetMultiplyExtrinsicRotationMatrix(Point3d vec, bool inv);
	bool GetPoints(Point3d cameraLocation, Point3d cameraOrintation,
			Point2d gPoints[4], Point2d cPoints[4],
			Point3d transformedPoints[4],bool useYaw);

	bool InitIPM(float diagonalAngleView);

};
