//DistortionModel.hpp
// Created on: May 12, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/General.hpp>
using namespace cv;

class DistortionModel
{
private:
	void undistortP_slow(const vector<Point> contour, vector<Point> &resCountour);
	void ModifoedOpenCVUndistortPoint( const CvMat* _src, CvMat* _dst, const CvMat* _cameraMatrix,
	                   const CvMat* _distCoeffs,
	                   const CvMat* matR, const CvMat* matP );

	void ModifoedOpenCVUndistortPoint( InputArray _src, OutputArray _dst,
	                          InputArray _cameraMatrix,
	                          InputArray _distCoeffs,
	                          InputArray _Rmat,
	                          InputArray _Pmat );
	Mat distortionModel;

	bool undistortP_normalized_slow(const vector<Point> contour,
			vector<Point2f> &resCountour);
	bool distortP_normalized_slow(const vector<Point3f> contour,
			vector<Point2f> &resCountour);

public:
	 float getDiagonalAngleView();
	Mat cameraMatrix, distCoeffs;
	bool Init();
	void CreateUndistort(const Mat &rawImg,Mat &res);
	void CreateUndistortFull(const Mat &rawImg,Mat &res,Scalar bg);
	bool UndistortP(const vector<Point> contour, vector<Point> &resCountour);

	bool DistortP(const vector<Point> contour,
			vector<Point> &resCountour);
	bool DistortPFull(const vector<Point> contour,
			vector<Point> &resCountour);
};


