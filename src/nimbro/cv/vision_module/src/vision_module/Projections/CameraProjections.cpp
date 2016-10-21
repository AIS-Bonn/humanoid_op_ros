//CameraProjections.cpp
// Created on: Apr 24, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#define VERBOSE false

#include <vision_module/Projections/CameraProjections.hpp>

bool CameraProjections::GetOnImageCordinate(const vector<LineSegment> &inLine,
		vector<LineSegment> &resLines)
{
	if (inLine.size() < 1)
	{
		ROS_ERROR("Error In Programming");
		return false;
	}
	resLines.resize(inLine.size()); //allocate result
	vector<Point2f> contour(inLine.size() * 2);
	vector<Point> resCountour;
	for (uint32_t i = 0; i < inLine.size(); i++)
	{
		contour[i * 2] = inLine[i].P1;
		contour[i * 2 + 1] = inLine[i].P2;
	}
	if (!GetOnImageCordinate(contour, resCountour))
	{
		return false;
	}

	for (uint32_t i = 0; i < inLine.size(); i++)
	{
		int tmpI = i * 2;
		resLines[i] = LineSegment(resCountour[tmpI], resCountour[tmpI + 1],
				inLine[i].getProbability());
	}
	return true;
}

vector<Point2f> CameraProjections::RotateTowardHeading(
		const vector<Point2f> &in)
{
	vector<Point2f> out(in.size());
	for (size_t i = 0; i < in.size(); i++)
	{
		out[i] = RotateTowardHeading(in[i]);
	}
	return out;
}
Point2d CameraProjections::RotateTowardHeading(const Point2d &in)
{
	return RotateAroundPoint(in, -Radian2Degree(getHeading()));
}

Point2f CameraProjections::RotateTowardHeading(const Point2f &in)
{
	return RotateAroundPoint(in, -Radian2Degree(getHeading()));
}
vector<LineSegment> CameraProjections::RotateTowardHeading(
		const vector<LineSegment> &in)
{

	vector<LineSegment> out(in.size());
	for (size_t i = 0; i < in.size(); i++)
	{
		out[i] = LineSegment(RotateTowardHeading(in[i].P1),
				RotateTowardHeading(in[i].P2), in[i].getProbability());
	}
	return out;
}

bool CameraProjections::GetOnRealCordinate(const vector<LineSegment> &inLine,
		vector<LineSegment> &resLines)
{
	if (inLine.size() < 1)
	{
		ROS_ERROR("Error In Programming");
		return false;
	}
	resLines.resize(inLine.size()); //allocate result

	if (params.tfP->precalculateProjection())
	{
		for (uint32_t i = 0; i < resLines.size(); i++)
		{
			resLines[i].P1 = realCoordinateVector[resLines[i].P1.y * IMGWIDTH
					+ resLines[i].P1.x];
			resLines[i].P2 = realCoordinateVector[resLines[i].P2.y * IMGWIDTH
					+ resLines[i].P2.x];
			resLines[i].setProbability(inLine[i].getProbability());
		}
		return true;
	}

	vector<Point> contour(inLine.size() * 2);
	vector<Point2f> resCountour;
	for (uint32_t i = 0; i < inLine.size(); i++)
	{
		contour[i * 2] = inLine[i].P1;
		contour[i * 2 + 1] = inLine[i].P2;
	}
	if (!GetOnRealCordinate(contour, resCountour))
	{
		return false;
	}

	for (uint32_t i = 0; i < inLine.size(); i++)
	{
		int tmpI = i * 2;
		resLines[i] = LineSegment(resCountour[tmpI], resCountour[tmpI + 1],
				inLine[i].getProbability());
	}
	return true;
}

bool CameraProjections::GetOnImageCordinate(const vector<Point2f> &contour,
		vector<Point> &resCountour)
{
	if (contour.size() < 1)
	{
		ROS_ERROR("Error In Programming");
		return false;
	}

	vector<Point2f> contourShiftAndScale(contour.size());
	vector<Point> resCI(contour.size());
	float TVW2 = ((params.topView->width() / params.topView->scale()) / 2.);
	for (uint32_t i = 0; i < contour.size(); i++)
	{
		Point2f f = Point2d(contour[i].x * 100., contour[i].y * 100.);
		f = Point2f(f.x / params.topView->scale() + TVW2,
				(f.y / params.topView->scale()) + TVW2);
		contourShiftAndScale[i] = f;
	}

	perspectiveTransform(contourShiftAndScale, contourShiftAndScale,
			realHomoBack);

	for (uint32_t i = 0; i < contourShiftAndScale.size(); i++)
	{
		resCI[i] = Point((int) contourShiftAndScale[i].x,
				(int) contourShiftAndScale[i].y);
	}
	return distorionModel.DistortP(resCI, resCountour);

}

bool CameraProjections::GetOnRealCordinate_single(const Point &pointIn,
		Point2f &resPoint)
{
	if (params.tfP->precalculateProjection())
	{
		resPoint = realCoordinateVector[pointIn.y * IMGWIDTH + pointIn.x];
		return true;
	}
	vector<Point> contour(1, pointIn);
	vector<Point2f> resCountour;
	if (!GetOnRealCordinate(contour, resCountour))
	{
		return false;
	}
	resPoint = resCountour[0];
	return true;
}

bool CameraProjections::GetOnImageCordinate_slow(const Point2f &pointIn,
		Point &resPoint)
{
	vector<Point2f> contour(1, pointIn);
	vector<Point> resCountour;
	if (!GetOnImageCordinate(contour, resCountour))
	{
		return false;
	}
	resPoint = resCountour[0];
	return true;
}

void CameraProjections::CalcFullRealCordinate()
{
	double TVW2 = (params.topView->width() / 2.);
	perspectiveTransform(distorionModel.distortionVector, realCoordinateVector,
			realHomoFor);
	for (uint32_t i = 0; i < realCoordinateVector.size(); i++)
	{
		double y = (realCoordinateVector[i].y * params.topView->scale())
				- (TVW2);
		double x = (realCoordinateVector[i].x * params.topView->scale())
				- (TVW2);
		realCoordinateVector[i] = Point2f(x / 100., y / 100.);
	}
}

bool CameraProjections::GetOnRealCordinate(const vector<Point> &contour,
		vector<Point2f> &resCountour)
{
	if (contour.size() < 1)
	{
		ROS_ERROR("Error In Programming");
		return false;
	}

	if (params.tfP->precalculateProjection())
	{
		resCountour.resize(contour.size()); //allocate result
		for (uint32_t i = 0; i < resCountour.size(); i++)
		{
			resCountour[i] = realCoordinateVector[contour[i].y * IMGWIDTH
					+ contour[i].x];
		}
		return true;
	}

	if (!distorionModel.UndistortP(contour, resCountour))
	{
		return false;
	}

	perspectiveTransform(resCountour, resCountour, realHomoFor);
	for (uint32_t i = 0; i < resCountour.size(); i++)
	{
		double y = (resCountour[i].y * params.topView->scale())
				- ((params.topView->width() / 2.));
		double x = (resCountour[i].x * params.topView->scale())
				- ((params.topView->width() / 2.));

		resCountour[i] = Point2f(x / 100., y / 100.);
	}
	return true;
}

bool CameraProjections::GetOnRealCordinate_FromUndistorted(
		const vector<Point2f> &contour, vector<Point2f> &resCountour)
{
	if (contour.size() < 1)
	{
		ROS_ERROR("Error In Programming");
		return false;
	}

	perspectiveTransform(contour, resCountour, realHomoFor);
	for (uint32_t i = 0; i < resCountour.size(); i++)
	{
		double y = (resCountour[i].y * params.topView->scale())
				- ((params.topView->width() / 2.));
		double x = (resCountour[i].x * params.topView->scale())
				- ((params.topView->width() / 2.));

		resCountour[i] = Point2f(x / 100., y / 100.);
	}
	return true;
}

bool CameraProjections::GetOnRealCordinate_FromUndistorted_Single(
		Point2f &pointIn, Point2f &resPoint)
{
	vector<Point2f> contour(1, pointIn);
	vector<Point2f> resCountour;
	if (!GetOnRealCordinate_FromUndistorted(contour, resCountour))
	{
		return false;
	}
	resPoint = resCountour[0];
	return true;
}

bool CameraProjections::Init(bool _dummy)
{

	realCoordinateVector.resize(IMGWIDTH * IMGHEIGHT);
	dummy = _dummy;
	egoGPoints.setType(visualization_msgs::Marker::LINE_STRIP);
	egoCPoints.setType(visualization_msgs::Marker::SPHERE_LIST);

	egoCPoints.setOrientation(1, 0, 0, 0);
	egoGPoints.setOrientation(1, 0, 0, 0);

	egoCPoints.setScale(0.005);
	egoGPoints.setScale(0.15);

	egoCPoints.setColor(1, 0, 0);
	egoGPoints.setColor(0, 1, 1);

	kinCalibRvizPoints.setType(visualization_msgs::Marker::CUBE_LIST);

	kinCalibRvizPoints.setOrientation(1, 0, 0, 0);

	kinCalibRvizPoints.setScale(0.1);

	kinCalibRvizPoints.setColor(0, 1, 1);

	kinCalibResPoints.setType(visualization_msgs::Marker::CUBE_LIST);

	kinCalibResPoints.setOrientation(1, 0, 0, 0);

	kinCalibResPoints.setScale(0.1);

	kinCalibResPoints.setColor(1, 0, 1);

	kinCalibRobotLoc.setType(visualization_msgs::Marker::ARROW);

	kinCalibRobotLoc.setOrientation(1, 1, 0, 0);

	kinCalibRobotLoc.setScale(0.1);

	kinCalibRobotLoc.setColor(0, 0, 0);

	if( !distorionModel.Init() )
	{
		return false;
	}

	float suggestedDiagnalAngleView = distorionModel.getDiagonalAngleView();
	diagnalAngleView = params.camera->diagonalAngleView();
	if (abs(suggestedDiagnalAngleView - diagnalAngleView) > 1)
	{
		ROS_WARN("Suggested Diagonal Camera Angle = %7.3f In config = %7.3f",
				suggestedDiagnalAngleView, diagnalAngleView);
	}
	else
	{
		ROS_INFO("Suggested Diagonal Camera Angle = %7.3f In config = %7.3f",
				suggestedDiagnalAngleView, diagnalAngleView);
	}

	uint32_t timeSec = ros::Time::now().sec;
	uint32_t timeNSec = ros::Time::now().nsec;

	usleep(500000);
	if (!m_tf.waitForRelativeToNowTransform("/ego_rot", "/camera_optical",
			params.tfP->timeToShift(), 2.0))
	{
		ROS_ERROR("No tf information found within 2 seconds!");
	}
	if (ros::Time::now().sec == 0)
	{
		ROS_ERROR("Ros time is zero!");
	}
	if (!(ros::Time::now().sec != timeSec || ros::Time::now().nsec != timeNSec))
	{
		ROS_ERROR("Ros time is not moving!");
	}

	return true;
}

bool CameraProjections::CalculateProjection()
{
	if (ipm.GetHomographyUseYaw(diagnalAngleView, cameraLocation,
			cameraOrintation, realHomoFor, realHomoBack,
			outerCornetrsUndistortedImg, gPointRes, transformedPointsRes,
			physicalCorners))
	{
		vector<Point> outerCornetrsResV(4), distortedOuterCorners;
		for (int i = 0; i < 4; i++)
		{
			outerCornetrsResV[i] = Point(outerCornetrsUndistortedImg[i].x,
					outerCornetrsUndistortedImg[i].y);
		}
		distorionModel.DistortP(outerCornetrsResV, distortedOuterCorners);
		for (int i = 0; i < 4; i++)
		{
			outerCornetrsRawImg[i] = Point(distortedOuterCorners[i].x,
					distortedOuterCorners[i].y);
		}

		_downLeftPoint = physicalCorners[0];
		_downRightPoint = physicalCorners[1];
		_upRightPoint = physicalCorners[2];
		_upRightPointt = physicalCorners[3];
		lastProjectionIsValid = true;
		if (params.tfP->precalculateProjection())
		{
			CalcFullRealCordinate();
		}
		return true;
	}
	lastProjectionIsValid = false;
	return false;
}

float CameraProjections::ComputeError(const vector<float> &d)
{
	double totalDis = 0;
	double maxDis = -9999999;
	int worseCoef = 3;
	int totalCount = 0;
	for (size_t i = 0; i < params.camCalibrator.opticalAngle.size(); i++)
	{
		cameraLocation = params.camCalibrator.cameraLocation[i];

		cameraOrintation = params.camCalibrator.opticalAngle[i]
				+ Point3d(d[0], d[1], d[2]);
		diagnalAngleView = d[3];
		CalculateProjection();

		Point2f res;
		if (GetOnRealCordinate_single(params.camCalibrator.clicked[i], res))
		{
			Point2f pointInRealToRobot = RotateAroundPoint(res,
					Radian2Degree(-params.tfP->robotZ()));
			pointInRealToRobot += Point2f(params.tfP->robotX(),
					params.tfP->robotY());
			double t = Distance2point(pointInRealToRobot,
					params.camCalibrator.rvizClicked[i]);
			totalDis += t;
			maxDis = std::max(maxDis, t);
			totalCount++;
		}
		else
		{
			ROS_ERROR("Programming Error!");
			return 10000000000;
		}
	}

	return (totalDis + (maxDis * worseCoef)) / (totalCount + worseCoef);
}

float CameraProjections::ComputeError(const vector<OptimizorParam> &d)
{
	vector<float> dN(d.size());
	for (size_t i = 0; i < d.size(); i++)
	{
		dN[i] = d[i].data;
	}
	return ComputeError(dN);
}

void CameraProjections::Calibrate()
{
	if (params.camCalibrator.IsReady())
	{
		if (params.tfP->calibrateKinematicHill()
				|| params.tfP->calibrateKinematicSimplex())
		{
			ros::Time startTime = ros::Time::now();
			ROS_INFO("Calibration Started!");
			calibrating = true;

			initialParameters.clear();
			initialParameters.push_back(
					OptimizorParam(params.tfP->orientationX()));
			initialParameters.push_back(
					OptimizorParam(params.tfP->orientationY()));
			initialParameters.push_back(
					OptimizorParam(params.tfP->orientationZ()));
			initialParameters.push_back(
					OptimizorParam(params.camera->diagonalAngleView(), 0.05));
			vector<OptimizorParam> p;
			if (params.tfP->calibrateKinematicHill())
			{
				ROS_INFO("Hill Climbing Optimizing ... ");
				hilloptimizer.setParameters(initialParameters);
				if (!hilloptimizer.TuneForBest(params.tfP->maxIteration()))
				{
					ROS_WARN(
							"Not complete success in converging so the result might be not the best");
				}
				p = hilloptimizer.getParameters();
			}
			else
			{
				ROS_INFO("Down Simplex Optimizing ... ");
				simplexoptimizer.setParameters(initialParameters);
				if (!simplexoptimizer.TuneForBest(params.tfP->maxIteration()))
				{
					ROS_WARN(
							"Not complete success in converging so the result might be not the best");
				}
				p = simplexoptimizer.getParameters();
			}
			params.tfP->orientationX.set(p[0].data);
			params.tfP->orientationY.set(p[1].data);
			params.tfP->orientationZ.set(p[2].data);
			params.camera->diagonalAngleView.set(p[3].data);
			params.tfP->calibrateKinematicHill.set(false);
			params.tfP->calibrateKinematicSimplex.set(false);
			Update(startTime);
			CalculateProjection();
			calibrating = false;
			ROS_INFO("Calibration Finished!");
		}
	}
}

bool CameraProjections::Update(ros::Time capTime)
{
	int lineNum = __LINE__;
	ros::Time actualTimeShift;
	lineNum = __LINE__;
	if (capTime.sec <= 0)
	{
		HAF_ERROR_THROTTLE(1, "Ros time (%d) is strange!", capTime.sec);
		return false;
	}
	try
	{
		lineNum = __LINE__;
		diagnalAngleView = params.camera->diagonalAngleView();
		lineNum = __LINE__;
		if (ros::Time::now() < lastTime)
		{
			lineNum = __LINE__;
			ROS_WARN("Vision Module detected a jump backward in time.");
			lineNum = __LINE__;
			ros::Time del = ros::Time::now();
			lineNum = __LINE__;
			uint32_t timeSec = ros::Time::now().sec;
			lineNum = __LINE__;
			uint32_t timeNSec = ros::Time::now().nsec;
			lineNum = __LINE__;
			reset_time_pub.publish(std_msgs::Empty());
			lineNum = __LINE__;
			reset_clock_pub.publish(std_msgs::Empty());
			lineNum = __LINE__;
			if (!m_tf.waitForRelativeToNowTransform("/ego_rot",
					"/camera_optical", params.tfP->timeToShift(), 1.0))
			{
				lineNum = __LINE__;
				ROS_ERROR("No tf information found within 1 second!");
			}
			lineNum = __LINE__;
			usleep(10000);
			lineNum = __LINE__;
			if (ros::Time::now().sec == timeSec
					&& ros::Time::now().nsec == timeNSec)
			{
				lineNum = __LINE__;
				ROS_ERROR("Ros time is not moving!");
			}
			lineNum = __LINE__;
			//headingOffset = 0;//It is restarting heading offset when bag play in restarted.
			lineNum = __LINE__;
			lastTime = ros::Time::now();
			lineNum = __LINE__;
			return false; //To force capTime be updated to the new time (after bag jump)
		}
		lineNum = __LINE__;
		tf::StampedTransform tfLastData;
		lineNum = __LINE__;
		m_tf.lookupTransform("/ego_rot", "/camera_optical", ros::Time(0),
				tfLastData);
		lineNum = __LINE__;
		lastTime = ros::Time::now();
		lineNum = __LINE__;
		ros::Time idealTimeShift = capTime
				- ros::Duration(params.tfP->timeToShift()); //To get the 60ms before
		lineNum = __LINE__;
		actualTimeShift = tfLastData.stamp_;
		ros::Time finalTimeShift =
				(actualTimeShift > idealTimeShift) ?
						idealTimeShift : actualTimeShift;
		lineNum = __LINE__;
		lastAvalibleTF = (capTime - finalTimeShift).sec
				+ ((capTime - finalTimeShift).nsec / 1000000000.);
		lineNum = __LINE__;
		//cout<<"ros::Time::now() = "<<ros::Time::now()<<" ros::Time(0) = "<<ros::Time(0)<<" ros::WallTime::now()"<<ros::WallTime::now()<< " past = "<<past<<endl;
		lineNum = __LINE__;
		tf::StampedTransform tfOpticalRot, tfOpticalPos;
		lineNum = __LINE__;
		m_tf.lookupTransform("/ego_rot", "/camera_optical", finalTimeShift,
				tfOpticalRot);
		lineNum = __LINE__;
		m_tf.lookupTransform("/ego_floor", "/camera_optical", finalTimeShift,
				tfOpticalPos);
		lineNum = __LINE__;
		tfScalar roll, pitch, yaw;
		lineNum = __LINE__;
		tfOpticalRot.getBasis().getEulerYPR(yaw, pitch, roll);
		lineNum = __LINE__;
		opticalAngle.x = pitch;
		opticalAngle.y = -(roll + M_PI);
		opticalAngle.z = yaw + M_PI / 2.;
		opticalLocation.x = tfOpticalPos.getOrigin().getX();
		opticalLocation.y = tfOpticalPos.getOrigin().getY();
		opticalLocation.z = tfOpticalPos.getOrigin().getZ();
		if (VERBOSE)
		{
			HAF_INFO_THROTTLE(0.33, "Rot | X= %1.3f Y= %1.3f Z=%1.3f",
					Radian2Degree(opticalAngle.x),
					Radian2Degree(opticalAngle.y),
					Radian2Degree(opticalAngle.z));

			HAF_INFO_THROTTLE(0.33, "Pos | X= %1.3f Y= %1.3f Z=%1.3f",
					opticalLocation.x, opticalLocation.y, opticalLocation.z);
		}

		cameraLocation.x = opticalLocation.x + params.tfP->locationX();
		cameraLocation.y = opticalLocation.y + params.tfP->locationY();
		cameraLocation.z = opticalLocation.z + params.tfP->locationZ();

		ballExtraDistance = params.ball->radius() / (double) cameraLocation.z;
		cameraOrintation.x = CorrectAngleRadian360(
				opticalAngle.x + params.tfP->orientationX());
		cameraOrintation.y = CorrectAngleRadian360(
				opticalAngle.y + params.tfP->orientationY());
		cameraOrintation.z = CorrectAngleRadian360(
				opticalAngle.z + params.tfP->orientationZ());
		lineNum = __LINE__;
	} catch (tf::TransformException &ex)
	{
		std::string errorMsg(ex.what());
		try
		{
			errorMsg += "\n " + to_string(lineNum)
					+ " |-> Suggested timeToShift = "
					+ to_string((capTime - actualTimeShift))
					+ "   Current timeToShift = "
					+ to_string(params.tfP->timeToShift());
			reset_time_pub.publish(std_msgs::Empty());
			reset_clock_pub.publish(std_msgs::Empty());
		} catch (tf::TransformException &ex)
		{
			errorMsg += " |-> Nothing published yet";
		}
		HAF_ERROR_THROTTLE(1, "%s", errorMsg.c_str());
		return false;
	}
	return true;
}

Point2f CameraProjections::convertToBallProjection(Point2f _in)
{
	float alpha = atan2(_in.y, _in.x);

	float dist = -GetDistance(_in) * ballExtraDistance;
	float x = _in.x + dist * cos(alpha);
	float y = _in.y + dist * sin(alpha);
	return Point2f(x, y);
}

void CameraProjections::Publish(Mat &guiRawImg, bool shouldPublish)
{
	if (params.gui->rvizSetCalibTF() || params.gui->rvizSetRobotTFOrigin())
	{
		if (params.camCalibrator.clicked.size() > 0)
		{
			vector<Point2f> realClicked;
			if (GetOnRealCordinate(params.camCalibrator.clicked, realClicked))
			{
				for (size_t i = 0; i < realClicked.size(); i++)
				{
					Point2f point = RotateAroundPoint(realClicked[i],
							Radian2Degree(-params.tfP->robotZ()));
					point += Point2f(params.tfP->robotX(),
							params.tfP->robotY());
					geometry_msgs::Point p;
					p.x = point.x;
					p.y = point.y;
					p.z = 0.0;
					kinCalibResPoints.marker.points.push_back(p);
				}
			}
		}

		for (size_t i = 0; i < params.camCalibrator.rvizClicked.size(); i++)
		{
			geometry_msgs::Point p;
			p.x = params.camCalibrator.rvizClicked[i].x;
			p.y = params.camCalibrator.rvizClicked[i].y;
			p.z = 0.0;
			kinCalibRvizPoints.marker.points.push_back(p);
		}

		kinCalibRobotLoc.marker.pose.position.x = params.tfP->robotX();
		kinCalibRobotLoc.marker.pose.position.y = params.tfP->robotY();
		kinCalibRobotLoc.marker.pose.position.z = 0;
		tf::Quaternion q = tf::createQuaternionFromRPY(M_PI / 2, 0,
				params.tfP->robotZ());
		kinCalibRobotLoc.marker.pose.orientation.w = q.getW();
		kinCalibRobotLoc.marker.pose.orientation.x = q.getX();
		kinCalibRobotLoc.marker.pose.orientation.y = q.getY();
		kinCalibRobotLoc.marker.pose.orientation.z = q.getZ();
		kinCalibRvizPoints.updateAdd();
		kinCalibResPoints.updateAdd();
		kinCalibRobotLoc.updateAdd();
		kinCalibMarker.publish();
		kinCalibRobotLoc.marker.points.clear();
		kinCalibRobotLoc.marker.pose.position.z = -100;
		kinCalibRvizPoints.marker.points.clear();
		kinCalibResPoints.marker.points.clear();
		kinCalibRobotLoc.marker.points.clear();
		kinCalibRobotLoc.marker.pose.position.z = -100;
		kinCalibMarker.clear();
	}
	if (!lastProjectionIsValid || guiRawImg.empty() || !shouldPublish)
		return;

	ros::Time now = ros::Time::now();
	if (params.gui->tfCalib())
	{
		for (size_t i = 0; i < params.camCalibrator.clicked.size(); i++)
		{
			circle(guiRawImg, params.camCalibrator.clicked[i], 3, blueColor(),
					2);
		}
	}

	if (params.debug->showTopView())
	{

		Mat UnDistRawImg;
		//Mat tmpp;
		//_distorionModel.CreateUndistortFull(guiRawImg, tmpp);
		distorionModel.CreateUndistort(guiRawImg, UnDistRawImg);
		unDistortedImg_pub.publish(UnDistRawImg, MatPublisher::bgr);

		int sizeOfTopViewW = params.topView->width() / params.topView->scale();
		int sizeOfTopViewH = params.topView->width() / params.topView->scale();

		if (topImg_pub.thereAreListeners(true))
		{
			Mat topHomoFor, topHomoBack;
			double picYawDegree;

			if (ipm.GetHomographyNoYaw(diagnalAngleView, cameraLocation,
					cameraOrintation, topHomoFor, topHomoBack, picYawDegree))
			{

				warpPerspective(UnDistRawImg, topViewBGR, topHomoFor,
						Size(sizeOfTopViewW, sizeOfTopViewH),
						CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);
				topImg_pub.publish(topViewBGR, MatPublisher::bgr);
			}
		}

		if ((params.debug->showPCL() && pcl_pub.getNumSubscribers() > 0)
				|| mmap_pub.getNumSubscribers() > 0)
		{
			warpPerspective(UnDistRawImg, calibViewBGR, realHomoFor,
					Size(sizeOfTopViewW, sizeOfTopViewH),
					CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);
//			calibImg_pub.publish(calibViewBGR, MatPublisher::bgr);

		}
		if (pcl_pub.getNumSubscribers() > 0 && params.debug->showPCL())
		{
			pcl::PointCloud < pcl::PointXYZRGB > cloud;
			// Fill in the cloud data
			cloud.width = sizeOfTopViewW * sizeOfTopViewH;
			cloud.height = 1;
			cloud.points.resize(cloud.width * cloud.height);
			int idxP = 0;
			for (uint16_t y = 0; y < sizeOfTopViewW; y++)
			{
				for (uint16_t x = 0; x < sizeOfTopViewH; x++)
				{
					double rx = (x - (sizeOfTopViewW / 2.))
							* (params.topView->scale() * 0.01);
					double ry = (y - (sizeOfTopViewH / 2.))
							* (params.topView->scale() * 0.01);
					cloud.points[idxP].x = rx;
					cloud.points[idxP].y = ry;
					cloud.points[idxP].z = 0;
					Vec3b bgr = calibViewBGR.at<Vec3b>(y, x);
					cloud.points[idxP].r = bgr[2];
					cloud.points[idxP].b = bgr[0];
					cloud.points[idxP].g = bgr[1];
					idxP++;
				}
			}

			//Convert the cloud to ROS message
			pcl::toROSMsg(cloud, output);
			output.header.frame_id = "/ego_floor";
			output.header.stamp = now;
			output.is_dense = true;
			pcl_pub.publish(output);
			params.debug->showPCL.set(false);
		}
		else if (mmap_pub.getNumSubscribers() > 0)
		{
			Mat mapMonoImg(calibViewBGR.size(), CV_8UC1);
			cvtColor(calibViewBGR, mapMonoImg, CV_BGR2GRAY);
			std_msgs::Header head;
			nav_msgs::MapMetaData mapMeta;
			head.frame_id = "/ego_floor";
			head.stamp = now;
			mapMeta.height = mapMonoImg.size().height;
			mapMeta.width = mapMonoImg.size().width;
			mapMeta.map_load_time = now;
			mapMeta.resolution = 0.01 * params.topView->scale();
			mapMeta.origin.orientation.w = 1;
			mapMeta.origin.position.x = -(int) (params.topView->width() / 2.)
					/ 100.;
			mapMeta.origin.position.y = -(int) (params.topView->width() / 2.)
					/ 100.;
			mapMeta.origin.position.z = 0.1;

			for (uint16_t y = 0; y < mapMonoImg.size().height; y++)
			{
				for (uint16_t x = 0; x < mapMonoImg.size().width; x++)
				{
					mmap.data.push_back(
							(mapMonoImg.at<uchar>(y, x) / 255.) * 100.);
				}
			}
			mmap.header = head;
			mmap.info = mapMeta;

			mmap_pub.publish(mmap);
			mmap.data.clear();
		}
	}

	if (params.debug->showCameraPoints())
	{
		for (int i = 0; i <= 4; i++)
		{
			int k = i;
			if (i == 4)
			{
				k = 0;
			}
			geometry_msgs::Point p;
			p.x = transformedPointsRes[k].x / 100.;
			p.y = transformedPointsRes[k].y / 100.;
			p.z = transformedPointsRes[k].z / 100.;
			egoCPoints.marker.points.push_back(p);
		}
		geometry_msgs::Point p;

		p.x = cameraLocation.x;
		p.y = cameraLocation.y;
		p.z = cameraLocation.z;
		egoCPoints.marker.points.push_back(p);
		for (int i = 0; i <= 4; i++)
		{
			int k = i;
			if (i == 4)
			{
				k = 0;
			}
			geometry_msgs::Point p;
			p.x = gPointRes[k].x / 100.;
			p.y = gPointRes[k].y / 100.;
			p.z = -0.0;
			egoGPoints.marker.points.push_back(p);

		}
		egoCPoints.updateAdd();
		egoGPoints.updateAdd();
		topViewMarker.publish();
		egoCPoints.marker.points.clear();
		egoGPoints.marker.points.clear();
		topViewMarker.clear();
	}
	if (params.debug->showHorizonBox())
	{
		for (int i = 0; i <= 3; i++)
		{
			int p1 = i;
			int p2 = i + 1;
			if (i == 3)
			{
				p2 = 0;
			}

			char msg[255];
			sprintf(msg, "(%d)", i);
			putText(guiRawImg, msg, outerCornetrsRawImg[i], 1, 1, greenColor());
			line(guiRawImg, outerCornetrsRawImg[p1], outerCornetrsRawImg[p2],
					redColor(), 3);
		}
		circle(guiRawImg, Point2d(320, 240), 5, redColor(), 5);
	}
}

