#include <vision_module/Tools/Kalman.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <vision_module/Tools/General.hpp>
using namespace cv;
using namespace std;

KalmanFilterC::KalmanFilterC(Point2f pt)
{
	kalman = new KalmanFilter(6, 2, 0);
	Mat processNoise(6, 1, CV_32F);

	kalman->statePre.at<float>(0) = pt.x;
	kalman->statePre.at<float>(1) = pt.y;
	kalman->statePre.at<float>(2) = 0;
	kalman->statePre.at<float>(3) = 0;
	kalman->statePre.at<float>(4) = 0;
	kalman->statePre.at<float>(5) = 0;
	kalman->transitionMatrix =
			(Mat_<float>(6, 6) << 1, 0, 1, 0, 0.5, 0, 0, 1, 0, 1, 0, 0.5, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
	kalman->measurementMatrix =
			(Mat_<float>(2, 6) << 1, 0, 1, 0, 0.5, 0, 0, 1, 0, 1, 0, 0.5);
	setIdentity(kalman->measurementMatrix);
	setIdentity(kalman->processNoiseCov, Scalar::all(1e-4));
	setIdentity(kalman->measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(kalman->errorCovPost, Scalar::all(.1));

}
KalmanFilterC::~KalmanFilterC()
{
	delete kalman;
}

Point2f KalmanFilterC::GetPrediction()
{
	Mat prediction = kalman->predict();
	return Point2f(prediction.at<float>(0), prediction.at<float>(1));
}

Point2f KalmanFilterC::Update(Point2f p)
{
	Mat measurement(2, 1, CV_32FC1);

	measurement.at<float>(0) = p.x;
	measurement.at<float>(1) = p.y;

	Mat estimated = kalman->correct(measurement);
	return Point2f(estimated.at<float>(0), estimated.at<float>(1));
}
