#pragma once
#include "opencv2/opencv.hpp"
#include <opencv/cv.h>
using namespace cv;
using namespace std;
/**
* @ingroup VisionModule
*
* @brief A class for Kalman filter
**/
class KalmanFilterC
{
public:
	KalmanFilter* kalman;
	KalmanFilterC(Point2f p);
	~KalmanFilterC();
	Point2f GetPrediction();
	Point2f Update(Point2f p);
};

