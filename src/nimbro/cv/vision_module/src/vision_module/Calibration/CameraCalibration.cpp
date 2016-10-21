#include <iostream>
#include <sstream>
#include <time.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <config_server/parameter.h>
#include <vision_module/Inputs/CameraDummy.hpp>
#include <vision_module/Inputs/Camera.hpp>
#include <vision_module/Inputs/ICamera.hpp>
#include <vision_module/Tools/Parameters.hpp>

using namespace cv;
using namespace std;

class Settings
{
public:
	enum Pattern
	{
		NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
	};
	Settings() :

			showUndistorsed(false), calibMode(CHESSBOARD)
	{

	}

	int getFlag()
	{
		int flag = 0;
//		flag |= CV_CALIB_USE_INTRINSIC_GUESS; // keep camera matrix as zero
//		flag |= CV_CALIB_FIX_ASPECT_RATIO;
//		flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
//		flag |= CV_CALIB_ZERO_TANGENT_DIST;
//		flag |= CV_CALIB_FIX_FOCAL_LENGTH;
//		flag |= CV_CALIB_FIX_K1;
//		flag |= CV_CALIB_FIX_K2;
		flag |= CV_CALIB_FIX_K3;
		if (!params.calib->high_dimension())
		{
			flag |= CV_CALIB_FIX_K4;
			flag |= CV_CALIB_FIX_K5;
			flag |= CV_CALIB_FIX_K6;
		}
		flag |= CV_CALIB_RATIONAL_MODEL;
		return flag;
	}

	Size getBoardSize()
	{
		return Size(params.calib->boardWidth(), params.calib->boardHeight());
	}
	bool showUndistorsed;
	Pattern calibMode;

};

enum
{
	DETECTION = 0, CAPTURING = 1, CALIBRATED = 2
};

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix,
		Mat& distCoeffs, vector<vector<Point2f> > imagePoints);

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "camera_calibration_node");

	ros::NodeHandle nodeHandle;
	bool dummy = false;
	for (int i = 0; i < argc; i++)
	{
		string tmp(argv[i]);
		if (tmp.find("dummy") != std::string::npos)
		{
			dummy = true;
		}
	}

	image_transport::ImageTransport it(nodeHandle);
	image_transport::Publisher calibImg_pub = it.advertise(
			"/vision/camCalibImg", 1);
	cv_bridge::CvImage msgRawImg;
	params.Init(nodeHandle);
	Settings s;

	vector<vector<Point2f> > imagePoints;
	Mat cameraMatrix, distCoeffs;
	Size imageSize;

	int mode = CAPTURING;

	clock_t prevTimestamp = 0;
	const Scalar RED(0, 0, 255), GREEN(0, 255, 0);

	ICamera *cam;
	if (!dummy)
	{
		cam = new Camera();

	}
	else
	{
		cam = new CameraDummy();
	}

	cam->InitCameraDevice(true);

	while (ros::ok())
	{
		if (cam->TakeCapture() > 0.5)
		{
			Mat view = cam->rawImage.clone();

			bool blinkOutput = false;

			if (mode == CAPTURING
					&& imagePoints.size() >= (unsigned) params.calib->frameCount())
			{
				if (runCalibrationAndSave(s, imageSize, cameraMatrix,
						distCoeffs, imagePoints))
				{
					mode = CALIBRATED;
					cv::FileStorage fs(params.configPath+params.calib->filePath(),
							cv::FileStorage::WRITE);
					fs << "cameraMatrix" << cameraMatrix << "distCoeffs"
							<< distCoeffs;
					fs.release();
				}
				else
				{
					mode = DETECTION;
				}
			}
			if (view.empty())
			{
				cout << "Failed to get capture! (view is empty)" << endl;
				if (imagePoints.size() > 0)
					runCalibrationAndSave(s, imageSize, cameraMatrix,
							distCoeffs, imagePoints);
				break;
			}

			imageSize = view.size();

			vector<Point2f> pointBuf;

			bool found;

			switch (s.calibMode)
			// Find feature points on the input format
			{
			case Settings::CHESSBOARD:
				found = findChessboardCorners(view, s.getBoardSize(), pointBuf,
						CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK
								| CALIB_CB_NORMALIZE_IMAGE);
				break;
			case Settings::CIRCLES_GRID:
				found = findCirclesGrid(view, s.getBoardSize(), pointBuf);
				break;
			case Settings::ASYMMETRIC_CIRCLES_GRID:
				found = findCirclesGrid(view, s.getBoardSize(), pointBuf,
						CALIB_CB_ASYMMETRIC_GRID);
				break;
			default:
				found = false;
				break;
			}

			if (found)                // If done with success,
			{
				// improve the found corners' coordinate accuracy for chessboard
				if (s.calibMode == Settings::CHESSBOARD)
				{
					Mat viewGray;
					cvtColor(view, viewGray, COLOR_BGR2GRAY);
					cornerSubPix(viewGray, pointBuf, Size(11, 11), Size(-1, -1),
							TermCriteria(
									TermCriteria::EPS + TermCriteria::COUNT, 30,
									0.1));
				}

				if (mode == CAPTURING && // For camera only take new samples after delay time
						(clock() - prevTimestamp
								> params.calib->delay() * 1e-3 * CLOCKS_PER_SEC))
				{
					imagePoints.push_back(pointBuf);
					prevTimestamp = clock();
					blinkOutput = true;
				}

				// Draw the corners.
				drawChessboardCorners(view, s.getBoardSize(), Mat(pointBuf),
						found);
			}

			string msg = (mode == CAPTURING) ? "100/100" : "Calibrated";
			int baseLine = 0;
			Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
			Point textOrigin(view.cols - 2 * textSize.width - 10,
					view.rows - 2 * baseLine - 10);

			if (mode == CAPTURING)
			{
				if (s.showUndistorsed)
					msg = format("%d/%d Undist", (int) imagePoints.size(),
							params.calib->frameCount());
				else
					msg = format("%d/%d", (int) imagePoints.size(),
							params.calib->frameCount());
			}

			putText(view, msg, textOrigin, 1, 1,
					mode == CALIBRATED ? GREEN : RED);

			if (blinkOutput)
				bitwise_not(view, view);

			if (mode == CALIBRATED && s.showUndistorsed)
			{
				Mat temp = view.clone();
				undistort(temp, view, cameraMatrix, distCoeffs);

			}

			waitKey(1);

			if (mode == CALIBRATED)
			{
				s.showUndistorsed = true;
			}
			msgRawImg.header.stamp = ros::Time::now();
			msgRawImg.encoding = sensor_msgs::image_encodings::BGR8;
			msgRawImg.image = view;
		}

		calibImg_pub.publish(msgRawImg.toImageMsg());
		ros::spinOnce();
		params.update();
	}

	return 0;
}

static double computeReprojectionErrors(
		const vector<vector<Point3f> >& objectPoints,
		const vector<vector<Point2f> >& imagePoints, const vector<Mat>& rvecs,
		const vector<Mat>& tvecs, const Mat& cameraMatrix,
		const Mat& distCoeffs, vector<float>& perViewErrors)
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int) objectPoints.size(); ++i)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
				distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);

		int n = (int) objectPoints[i].size();
		perViewErrors[i] = (float) std::sqrt(err * err / n);
		totalErr += err * err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

static void calcBoardCornerPositions(Size boardSize, float squareSize,
		vector<Point3f>& corners, Settings::Pattern patternType)
{
	corners.clear();

	switch (patternType)
	{
	case Settings::CHESSBOARD:
	case Settings::CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; ++i)
			for (int j = 0; j < boardSize.width; ++j)
				corners.push_back(
						Point3f(float(j * squareSize), float(i * squareSize),
								0));
		break;

	case Settings::ASYMMETRIC_CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(
						Point3f(float((2 * j + i % 2) * squareSize),
								float(i * squareSize), 0));
		break;
	default:
		break;
	}
}

static bool runCalibration(Settings& s, Size& imageSize, Mat& cameraMatrix,
		Mat& distCoeffs, vector<vector<Point2f> > imagePoints,
		vector<Mat>& rvecs, vector<Mat>& tvecs, vector<float>& reprojErrs,
		double& totalAvgErr)
{

	cameraMatrix = Mat::eye(3, 3, CV_64F);
	if (s.getFlag() & CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0, 0) = 1.0;

	distCoeffs = Mat::zeros(8, 1, CV_64F);

	vector<vector<Point3f> > objectPoints(1);
	calcBoardCornerPositions(s.getBoardSize(), params.calib->squareSize(), objectPoints[0],
			s.calibMode);

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	//Find intrinsic and extrinsic camera parameters
	double rms = calibrateCamera(objectPoints, imagePoints, imageSize,
			cameraMatrix, distCoeffs, rvecs, tvecs, s.getFlag());

	cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs,
			tvecs, cameraMatrix, distCoeffs, reprojErrs);

	return ok;
}

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix,
		Mat& distCoeffs, vector<vector<Point2f> > imagePoints)
{
	vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;
	double totalAvgErr = 0;

	bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs,
			imagePoints, rvecs, tvecs, reprojErrs, totalAvgErr);
	cout << (ok ? "Calibration succeeded" : "Calibration failed")
			<< ". avg re projection error = " << totalAvgErr;

	return ok;
}
