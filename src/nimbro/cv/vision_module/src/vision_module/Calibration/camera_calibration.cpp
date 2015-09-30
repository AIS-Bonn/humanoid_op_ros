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

			showUndistorsed(false), calibMode(CHESSBOARD), boardWidth(
					"/vision/calib/boardWidth", 1, 1, 15, 7), boardHeight(
					"/vision/calib/boardHeight", 1, 1, 15, 10), squareSize(
					"/vision/calib/squareSize", 1, 1, 200, 58), delay(
					"/vision/calib/delay", 1, 1, 3000, 200), cameraCalibFile(
					"/vision/calib/filePath", "cCFile.yml"), flipHor(
					"/vision/camera/flipHor", false), flipVer(
					"/vision/camera/flipVer", false), devNumber(
					"/vision/camera/devNumber", 0, 1, 3, 0), nrFrames(
					"/vision/calib/frameCount", 0, 1, 200, 60)
	{

	}

	int getFlag()
	{
		int flag = 0;
//		flag |= CV_CALIB_USE_INTRINSIC_GUESS; // camera matrix ro sefr mikone
//		flag |= CV_CALIB_FIX_ASPECT_RATIO;
//		flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
//		flag |= CV_CALIB_ZERO_TANGENT_DIST;
//		flag |= CV_CALIB_FIX_FOCAL_LENGTH;
//		flag |= CV_CALIB_FIX_K1;
//		flag |= CV_CALIB_FIX_K2;
		flag |= CV_CALIB_FIX_K3;
//		flag |= CV_CALIB_FIX_K4;
//		flag |= CV_CALIB_FIX_K5;
//		flag |= CV_CALIB_FIX_K6;
		flag |= CV_CALIB_RATIONAL_MODEL;
		return flag;
	}

	Size getBoardSize()
	{
		return Size(boardWidth(), boardHeight());
	}
	bool showUndistorsed;
	Pattern calibMode;
	config_server::Parameter<int> boardWidth;
	config_server::Parameter<int> boardHeight;
	config_server::Parameter<int> squareSize;
	config_server::Parameter<int> delay;
	config_server::Parameter<std::string> cameraCalibFile;
	config_server::Parameter<bool> flipHor;
	config_server::Parameter<bool> flipVer;
	config_server::Parameter<int> devNumber;
	config_server::Parameter<int> nrFrames;
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

	image_transport::ImageTransport it(nodeHandle);
	image_transport::Publisher rawImg_pub = it.advertise("/vision/rawImg", 1);
	cv_bridge::CvImage msgRawImg;

	Settings s;

	vector<vector<Point2f> > imagePoints;
	Mat cameraMatrix, distCoeffs;
	Size imageSize;

	int mode = CAPTURING;

	clock_t prevTimestamp = 0;
	const Scalar RED(0, 0, 255), GREEN(0, 255, 0);

	VideoCapture cap(s.devNumber());
	while (ros::ok())
	{
		Mat view;
		bool blinkOutput = false;

		cap >> view;

		if (mode == CAPTURING && imagePoints.size() >= (unsigned) s.nrFrames())
		{
			if (runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs,
					imagePoints))
			{
				mode = CALIBRATED;
				cv::FileStorage fs(s.cameraCalibFile(), cv::FileStorage::WRITE);
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
			cout << "Failed to get capture!" << endl;
			if (imagePoints.size() > 0)
				runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs,
						imagePoints);
			break;
		}

		imageSize = view.size();

		if (s.flipHor() && s.flipVer())
		{
			flip(view, view, -1);
		}
		else
		{
			if (s.flipVer())
			{
				flip(view, view, 0);
			}
			else if (s.flipHor())
			{
				flip(view, view, 1);
			}
		}

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
						TermCriteria(TermCriteria::EPS + TermCriteria::COUNT,
								30, 0.1));
			}

			if (mode == CAPTURING && // For camera only take new samples after delay time
					(!cap.isOpened()
							|| clock() - prevTimestamp
									> s.delay() * 1e-3 * CLOCKS_PER_SEC))
			{
				imagePoints.push_back(pointBuf);
				prevTimestamp = clock();
				blinkOutput = cap.isOpened();
			}

			// Draw the corners.
			drawChessboardCorners(view, s.getBoardSize(), Mat(pointBuf), found);
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
						s.nrFrames());
			else
				msg = format("%d/%d", (int) imagePoints.size(), s.nrFrames());
		}

		putText(view, msg, textOrigin, 1, 1, mode == CALIBRATED ? GREEN : RED);

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

		rawImg_pub.publish(msgRawImg.toImageMsg());
		ros::spinOnce();

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
	calcBoardCornerPositions(s.getBoardSize(), s.squareSize(), objectPoints[0],
			s.calibMode);

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	//Find intrinsic and extrinsic camera parameters
	double rms = calibrateCamera(objectPoints, imagePoints, imageSize,
			cameraMatrix, distCoeffs, rvecs, tvecs,
			s.getFlag() );

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
