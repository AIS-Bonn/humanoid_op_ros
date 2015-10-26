//CameraProjections.cpp
// Created on: Apr 24, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#define VERBOSE false

#include <vision_module/Projections/CameraProjections.hpp>

bool CameraProjections::GetOnImageCordinate(const vector<LineSegment> inLine,
		vector<LineSegment> &resLines)
{
	if (inLine.size() < 1)
	{
		ROS_ERROR("Error In Programming");
		return false;
	}
	vector<Point2f> contour;
	vector<Point> resCountour;
	for (uint32_t i = 0; i < inLine.size(); i++)
	{
		contour.push_back(inLine[i].P1);
		contour.push_back(inLine[i].P2);
	}
	if (!GetOnImageCordinate(contour, resCountour))
	{
		return false;
	}

	for (uint32_t i = 0; i < inLine.size(); i++)
	{
		int tmpI = i * 2;
		resLines.push_back(
				LineSegment(resCountour[tmpI], resCountour[tmpI + 1]));
	}
	return true;
}

vector<Point2f> CameraProjections::RotateTowardHeading(vector<Point2f> in)
{
	vector<Point2f> out;
	for(size_t i=0;i<in.size();i++)
	{
		out.push_back( RotateTowardHeading(in[i]));
	}
	return out;
}
Point2d CameraProjections::RotateTowardHeading(Point2d in)
{
	return RotateAroundPoint(in,-Radian2Degree(getHeading()));
}

Point2f CameraProjections::RotateTowardHeading(Point2f in)
{
	return RotateAroundPoint(in,-Radian2Degree(getHeading()));
}
vector<LineSegment> CameraProjections::RotateTowardHeading(
		vector<LineSegment> in)
{

	vector<LineSegment> out;
	for(size_t i=0;i<in.size();i++)
	{
		out.push_back(LineSegment(RotateTowardHeading(in[i].P1),RotateTowardHeading(in[i].P2)));
	}
	return out;
}

bool CameraProjections::GetOnRealCordinate(const vector<LineSegment> inLine,
		vector<LineSegment> &resLines)
{
	if (inLine.size() < 1)
	{
		ROS_ERROR("Error In Programming");
		return false;
	}
	vector<Point> contour;
	vector<Point2f> resCountour;
	for (uint32_t i = 0; i < inLine.size(); i++)
	{
		contour.push_back(inLine[i].P1);
		contour.push_back(inLine[i].P2);
	}
	if (!GetOnRealCordinate(contour, resCountour))
	{
		return false;
	}

	for (uint32_t i = 0; i < inLine.size(); i++)
	{
		int tmpI = i * 2;
		resLines.push_back(
				LineSegment(resCountour[tmpI], resCountour[tmpI + 1]));
	}
	return true;
}

bool CameraProjections::GetOnImageCordinate(const vector<Point2f> contour,
		vector<Point> &resCountour)
{
	if (contour.size() < 1)
	{
		ROS_ERROR("Error In Programming");
		return false;
	}
	vector<Point2f> resC, resCountourd, contourShiftAndScale;
	vector<Point> resCI;

	for (uint32_t i = 0; i < contour.size(); i++)
	{
		Point2f f = Point2d(contour[i].x * 100., contour[i].y * 100.);
		f = Point2d(
				f.x / params.topView.scale->get()
						+ ((params.topView.width->get()
								/ params.topView.scale->get()) / 2.),
				(f.y / params.topView.scale->get())
						+ ((params.topView.width->get()
								/ params.topView.scale->get()) / 2.));
		contourShiftAndScale.push_back(f);
	}

	perspectiveTransform(contourShiftAndScale, resC, realHomoBack);

	for (uint32_t i = 0; i < resC.size(); i++)
	{
		resCI.push_back(Point((int) resC[i].x, (int) resC[i].y));
	}
	return _distorionModel.DistortP(resCI, resCountour);

}

bool CameraProjections::GetOnRealCordinate(const Point pointIn,
		Point2f &resPoint)
{
	vector<Point> contour;
	contour.push_back(pointIn);
	vector<Point2f> resCountour;
	if (!GetOnRealCordinate(contour, resCountour))
	{
		return false;
	}
	resPoint = resCountour[0];
	return true;
}

bool CameraProjections::GetOnImageCordinate(Point2f pointIn, Point &resPoint)
{
	vector<Point2f> contour;
	contour.push_back(pointIn);
	vector<Point> resCountour;
	if (!GetOnImageCordinate(contour, resCountour))
	{
		return false;
	}
	resPoint = resCountour[0];
	return true;
}

bool CameraProjections::GetOnRealCordinate(const vector<Point> contour,
		vector<Point2f> &resCountour)
{
	if (contour.size() < 1)
	{
		ROS_ERROR("Error In Programming");
		return false;
	}

	vector<Point> resC;
	if (!_distorionModel.UndistortP(contour, resC))
	{
		return false;
	}

	std::vector<Point2f> resCD;
	for (uint32_t i = 0; i < resC.size(); i++)
	{
		resCD.push_back(Point2f(resC[i].x, resC[i].y));
	}
	perspectiveTransform(resCD, resCD, realHomoFor);
	for (uint32_t i = 0; i < resCD.size(); i++)
	{
		double y = (resCD[i].y * params.topView.scale->get())
				- ((params.topView.width->get() / 2.));
		double x = (resCD[i].x * params.topView.scale->get())
				- ((params.topView.width->get() / 2.));

		resCountour.push_back(Point2f(x / 100., y / 100.));
	}
	return true;
}

bool CameraProjections::Init(bool _dummy)
{
	dummy = _dummy;
	topViewGPoints.setType(visualization_msgs::Marker::LINE_STRIP);
	topViewCPoints.setType(visualization_msgs::Marker::SPHERE_LIST);

	topViewCPoints.setOrientation(1, 0, 0, 0);
	topViewGPoints.setOrientation(1, 0, 0, 0);

	topViewCPoints.setScale(0.005);
	topViewGPoints.setScale(0.15);

	topViewCPoints.setColor(1, 0, 0);
	topViewGPoints.setColor(0, 1, 1);
	bool res = _distorionModel.Init();
	diagnalAngleView = _distorionModel.getDiagonalAngleView();
	params.camera.diagonalAngleView->set(diagnalAngleView);
	ROS_INFO(" Diagonal Camera Angle = %7.3f", diagnalAngleView);
	ros::Time past = ros::Time::now() - ros::Duration(0.033);
	if (dummy)
	{
		past = ros::Time(0); //To get latest
	}
	m_tf->waitForTransform("/ego_rot", "/camera_optical", past,
			ros::Duration(2));
	return res;
}

bool CameraProjections::CalculateProjection()
{
	if (ipm.GetHomographyUseYaw(diagnalAngleView, cameraLocation,
			cameraOrintation, realHomoFor, realHomoBack,
			outerCornetrsUndistortedImg, gPointRes, transformedPointsRes,
			physicalCorners))
	{
		vector<Point> outerCornetrsResV, distortedOuterCorners;
		for (int i = 0; i < 4; i++)
		{
			outerCornetrsResV.push_back(
					Point(outerCornetrsUndistortedImg[i].x,
							outerCornetrsUndistortedImg[i].y));
		}
		_distorionModel.DistortP(outerCornetrsResV, distortedOuterCorners);
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
		return true;
	}
	lastProjectionIsValid = false;
	return false;
}

float CameraProjections::ComputeError(vector<OptimizorParam> &d)
{
	double totalDis = 0;
	double maxDis = -9999999;
	int worseCoef = 3;
	int totalCount = 0;
	for (size_t i = 0; i < params.camCalibrator.opticalAngle.size(); i++)
	{
		cameraLocation = params.camCalibrator.cameraLocation[i];

		cameraOrintation = params.camCalibrator.opticalAngle[i]
				+ Point3d(d[0].data, d[1].data, d[2].data);
		diagnalAngleView = d[3].data;
		CalculateProjection();

		Point2f res;
		if (GetOnRealCordinate(params.camCalibrator.clicked[i], res))
		{
			double t = Distance2point(res, params.camCalibrator.rvizClicked[i]);
			totalDis += t;
			maxDis = std::max(maxDis, t);
			totalCount++;
		}
		else
		{
			return 10000000000;
		}
	}

	return (totalDis + (maxDis * worseCoef)) / (totalCount + worseCoef);
}

void CameraProjections::Calibrate()
{
	if (params.camCalibrator.IsReady())
	{
		if (params.topView.calibrateKinematic->get())
		{
			ROS_INFO("Calibration Started!");
			calibrating = true;

			initialParameters.clear();
			initialParameters.push_back(
					OptimizorParam(params.orientation.x->get()));
			initialParameters.push_back(
					OptimizorParam(params.orientation.y->get()));
			initialParameters.push_back(
					OptimizorParam(params.orientation.z->get()));
			initialParameters.push_back(
					OptimizorParam(params.camera.diagonalAngleView->get(),
							0.05));

			optimizer.setParameters(initialParameters);

			if (!optimizer.TuneForBest(50, 0.0000001))
			{
				ROS_WARN("Not complete success in converging");
			}

			vector<OptimizorParam> p = optimizer.getParameters();
			params.orientation.x->set(p[0].data);
			params.orientation.y->set(p[1].data);
			params.orientation.z->set(p[2].data);
			params.camera.diagonalAngleView->set(p[3].data);
			params.topView.calibrateKinematic->set(false);
			Update();
			CalculateProjection();
			calibrating = false;
			ROS_INFO("Calibration Finished!");
		}
	}
}

bool CameraProjections::Update()
{
	try
	{
		diagnalAngleView = params.camera.diagonalAngleView->get();
		if (dummy && (ros::Time::now() < lastTime))
		{
			ROS_WARN(
					"Vision Module detected a backward time. Cleared all cached tf data.");
			m_tf->clear();
			delete m_tf;
			m_tf = new tf::TransformListener(ros::Duration(10));
			m_tf->waitForTransform("/ego_rot", "/camera_optical",
					ros::Time::now()
							- ros::Duration(params.debug.timeToShift->get()),
					ros::Duration(1.0));

		}
		lastTime = ros::Time::now();
		topViewMarker.clear();
		ros::Time past = ros::Time::now() - ros::Duration(0.060); //To get the 40ms before
		if (dummy)
		{
			past = ros::Time(0); //To get latest
		}
		m_tf->lookupTransform("/ego_rot", "/camera_optical",
				ros::Time::now()
						- ros::Duration(params.debug.timeToShift->get()),
				tfOptical);
		tfScalar roll, pitch, yaw;
		tfOptical.getBasis().getEulerYPR(yaw, pitch, roll);
		//	cout<< " X = "<<tfOptical.getOrigin().getX() << " Y= "<<	tfOptical.getOrigin().getY()<< " Z="<<	tfOptical.getOrigin().getZ()<<endl;
		OpticalAngle.x = pitch;
		OpticalAngle.y = -(roll + M_PI);
		OpticalAngle.z = yaw + M_PI / 2.;

		if (VERBOSE)
		{
			ROS_INFO_THROTTLE(0.33, "x= %1.2f y= %1.2f z=%1.2f",
					Radian2Degree(OpticalAngle.x),
					Radian2Degree(OpticalAngle.y),
					Radian2Degree(OpticalAngle.z));
		}

		cameraLocation.x = params.location.x->get(); //todo: it seems that the actual height published in odometry
		cameraLocation.y = params.location.y->get();
		cameraLocation.z = params.location.z->get();

		cameraOrintation.x = OpticalAngle.x + params.orientation.x->get();
		cameraOrintation.y = OpticalAngle.y + params.orientation.y->get();
		cameraOrintation.z = CorrectAngleRadian360(
				OpticalAngle.z + params.orientation.z->get());

	} catch (tf::TransformException &ex)
	{
		std::string errorMsg(ex.what());
//		if(errorMsg.find("Lookup would require extrapolation")!=std::string::npos)
//		{
//			reset_time_pub.publish(std_msgs::Empty());
//			reset_clock_pub.publish(std_msgs::Empty());
//		}
		ROS_ERROR("%s", errorMsg.c_str());
		return false;
	}
	return true;
}

void CameraProjections::Publish(Mat &guiRawImg, bool showHorizonBox)
{

	if (!lastProjectionIsValid || guiRawImg.empty())
		return;

	ros::Time now = ros::Time::now();
	if (params.debug.showTopView->get())
	{

		Mat UnDistRawImg;
		//Mat tmpp;
		//_distorionModel.CreateUndistortFull(guiRawImg, tmpp);
		_distorionModel.CreateUndistort(guiRawImg, UnDistRawImg);
		unDistortedImg_pub.publish(UnDistRawImg, MatPublisher::bgr);

		int sizeOfTopViewW = params.topView.width->get()
				/ params.topView.scale->get();
		int sizeOfTopViewH = params.topView.width->get()
				/ params.topView.scale->get();

		if (topImg_pub.thereAreListeners())
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

		if (mmap_pub.getNumSubscribers() > 0)
		{
			warpPerspective(UnDistRawImg, calibViewBGR, realHomoFor,
					Size(sizeOfTopViewW, sizeOfTopViewH),
					CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);
			calibImg_pub.publish(calibViewBGR, MatPublisher::bgr);
			Mat mapMonoImg(calibViewBGR.size(), CV_8UC1);
			cvtColor(calibViewBGR, mapMonoImg, CV_BGR2GRAY);
			std_msgs::Header head;
			nav_msgs::MapMetaData mapMeta;
			head.frame_id = "/ego_floor";
			head.stamp = now;
			mapMeta.height = mapMonoImg.size().height;
			mapMeta.width = mapMonoImg.size().width;
			mapMeta.map_load_time = now;
			mapMeta.resolution = 0.01 * params.topView.scale->get();
			mapMeta.origin.orientation.w = 1;
			mapMeta.origin.position.x =
					-(int) (params.topView.width->get() / 2.) / 100.;
			mapMeta.origin.position.y =
					-(int) (params.topView.width->get() / 2.) / 100.;
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

	if (params.debug.showCameraPoints->get())
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
			topViewCPoints.marker.points.push_back(p);
		}
		geometry_msgs::Point p;

		p.x = cameraLocation.x;
		p.y = cameraLocation.y;
		p.z = cameraLocation.z;
		topViewCPoints.marker.points.push_back(p);
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
			topViewGPoints.marker.points.push_back(p);

		}
		topViewCPoints.update();
		topViewGPoints.update();
		topViewMarker.publish();
		topViewCPoints.marker.points.clear();
		topViewGPoints.marker.points.clear();
	}
	if (showHorizonBox && params.debug.showHorizonBox->get())
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

