//Localization.cpp
// Created on: Apr 20, 2016
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <vision_module/Localization/Localization.hpp>

bool Localization::Update(CameraProjections &projection)
{
	_cameraProjections = &projection;
	if (params.locIsClicked)
	{
		params.locIsClicked = false;
		ROS_INFO("Set location received.");
		this->location.x = params.locClicked.x;
		this->location.y = params.locClicked.y;
		this->locationKalman.x = params.locClicked.x;
		this->locationKalman.y = params.locClicked.y;
	}

	kalmanI.GetPrediction();
	Point2d kalRes = kalmanI.Update(Point2d(location.x, location.y));
	locationKalman.x = kalRes.x;
	locationKalman.y = kalRes.y;

	{
		boundry_n(location.x, -A2 - I, A2 + I);
		boundry_n(location.y, -B2 - I, B2 + I);
		boundry_n(locationKalman.x, -A2 - I, A2 + I);
		boundry_n(locationKalman.y, -B2 - I, B2 + I);
	}

	return true;
}

bool Localization::Calculate(vector<LineSegment> &clusteredLines,
		bool circleDetected, const Point2f &FieldHullRealCenter,
		const vector<cv::Point2f> &FieldHullReal,
		const Point2d &resultCircleRotated, const vector<Point2f> &goalPosition,
		const bool &confiused, vector<LineContainer> &AllLines,
		vector<FeatureContainer> &AllFeatures)
{
	if (_cameraProjections == NULL)
	{
		ROS_ERROR("Error in programming");
		return false;
	}

	AllLines.reserve(clusteredLines.size());
	AllFeatures.reserve(5);

	const double MAX_FASHER = 200.;
	atLeastOneObservation = false;

	double UPDATENORMAL = params.loc->UPDATENORMAL()
			* params.loc->TOTALGAIN();
	double UPDATESTRONG = params.loc->UPDATESTRONG()
			* params.loc->TOTALGAIN();
	double UPDATEWEAK = params.loc->UPDATEWEAK() * params.loc->TOTALGAIN();

	LineSegment HorLine(cv::Point(0, -10), cv::Point(0, 10));
	LineSegment VerLine(cv::Point(10, 0), cv::Point(-10, 0));

	for (size_t i = 0; i < clusteredLines.size(); i++)
	{
		LineSegment lineSeg = clusteredLines[i];

		if (lineSeg.GetLength() > params.loc->minLineLen())
		{
			cv::Point2d mid = lineSeg.GetMiddle();

			if (lineSeg.GetAbsMinAngleDegree(VerLine) < 45)
			{
				LineType thisType = VerUndef;
				double angleDiffVer = lineSeg.GetExteriorAngleDegree(VerLine);
				if (angleDiffVer < -90)
					angleDiffVer += 180;
				if (angleDiffVer > 90)
					angleDiffVer += -180;

				if (lineSeg.DistanceFromLine(cv::Point(0, 0))
						> params.loc->VerLineMinDistanceToUpdate())
				{
					LandmarkType ltype = CenterL;
					double estimatedY = 0;
					if (mid.y > 0)
					{
						thisType = VerLeft;
						estimatedY = B2 - mid.y;
						ltype = LeftL;
					}
					else
					{
						thisType = VerRight;
						estimatedY = -B2 + abs(mid.y);
						ltype = RightL;
					}
					addObservation(Point2d(0, estimatedY), 0,
							MAX_FASHER * getUpdateCoef(UPDATENORMAL, lineSeg),
							ltype);
				}
				else if (lineSeg.DistanceFromLine(FieldHullRealCenter)
						> (params.loc->VerLineMinDistanceToUpdate() / 2.)
						&& cv::contourArea(FieldHullReal) > 4)
				{
					LandmarkType ltype = CenterL;
					LineSegment l2Test = lineSeg;
					if (lineSeg.P1.x > lineSeg.P2.x)
					{
						l2Test.P1 = lineSeg.P2;
						l2Test.P2 = lineSeg.P1;
					}

					double estimatedY = 0;
					if (l2Test.GetSide(FieldHullRealCenter) < 0)
					{
						thisType = VerLeftNear;
						estimatedY = B2 - mid.y;
						ltype = LeftL;
					}
					else
					{
						thisType = VerRightNear;
						estimatedY = -B2 + abs(mid.y);
						ltype = RightL;
					}
					addObservation(Point2d(0, estimatedY), 0,
							MAX_FASHER * getUpdateCoef(UPDATENORMAL, lineSeg),
							ltype);
				}
				AllLines.push_back(LineContainer(lineSeg, thisType));
			}
			else
			{
				LineType thisType = HorUndef;
				double angleDiffHor = lineSeg.GetExteriorAngleDegree(HorLine);
				if (angleDiffHor < -90)
					angleDiffHor += 180;
				if (angleDiffHor > 90)
					angleDiffHor += -180;

				if (circleDetected
						&& DistanceFromLineSegment(lineSeg, resultCircleRotated)
								< 1)
				{
					thisType = HorCenter;
					double estimatedX = -mid.x;

					addObservation(Point2d(estimatedX, 0),
							MAX_FASHER * UPDATENORMAL, 0, CenterL);
				}

				if (goalPosition.size() >= 2
						&& lineSeg.DistanceFromLine(goalPosition[0]) < 0.5
						&& lineSeg.DistanceFromLine(goalPosition[1]) < 0.5)
				{

					LandmarkType typel = CenterL;
					double estimatedX = 0;
					if (mid.x > 0)
					{
						typel = FrontL;
						estimatedX = A2 - mid.x;
					}
					else
					{
						typel = BackL;
						estimatedX = -A2 + abs(mid.x);
					}
					double lowPC = getUpdateCoef(
							(goalPosition.size() == 2 ?
									UPDATESTRONG : UPDATENORMAL), lineSeg);
					addObservation(Point2d(estimatedX, 0), MAX_FASHER * lowPC,
							0, typel);
					thisType = HorGoal;

				}

				AllLines.push_back(LineContainer(lineSeg, thisType));
			}
		}
	}

	for (size_t i = 0; i < AllLines.size(); i++)
	{
		LineContainer hI = AllLines[i];
		if (hI.type != HorUndef)
			continue;
		for (size_t j = i; j < AllLines.size(); j++)
		{
			LineContainer hJ = AllLines[j];
			if (hJ.type != HorUndef)
				continue;
			cv::Point2d midI = hI.lineTransformed.GetMiddle();
			cv::Point2d midJ = hJ.lineTransformed.GetMiddle();
			double distanceToEachOther = dist3D_Segment_to_Segment(
					hI.lineTransformed, hJ.lineTransformed);

			double midPointsLSAngleToOne =
					hI.lineTransformed.GetAbsMinAngleDegree(
							LineSegment(midI, midJ));
			if (distanceToEachOther < E * 1.5 && distanceToEachOther > E * 0.5
					&& hI.lineTransformed.GetAbsMinAngleDegree(
							hJ.lineTransformed) < 30
					&& midPointsLSAngleToOne > 25)
			{

				bool hJ_Is_Goal_Line = hJ.lineTransformed.DistanceFromLine(
						cv::Point(0, 0))
						> hI.lineTransformed.DistanceFromLine(cv::Point(0, 0));
				cv::Point2d mid = hJ_Is_Goal_Line ? midJ : midI;
				double estimatedX = 0;
				if ((hJ_Is_Goal_Line ? hJ.lineTransformed : hI.lineTransformed).DistanceFromLine(
						cv::Point(0, 0)) > 1.2)
				{
					LandmarkType typel = CenterL;
					if (mid.x > 0)
					{
						estimatedX = A2 - mid.x;
						typel = FrontL;
					}
					else
					{
						estimatedX = -A2 + abs(mid.x);
						typel = BackL;
					}
					double lowPC = getUpdateCoef(UPDATESTRONG,
							hJ_Is_Goal_Line ?
									hJ.lineTransformed : hI.lineTransformed);
					addObservation(Point2d(estimatedX, 0), MAX_FASHER * lowPC,
							0, typel);
				}
			}
		}
	}

	if (circleDetected)
	{

		double estimatedX = -resultCircleRotated.x;
		double estimatedY = -resultCircleRotated.y;

		addObservation(Point2d(estimatedX, estimatedY),
				MAX_FASHER * UPDATEWEAK, MAX_FASHER * UPDATEWEAK, CenterL);

	}

	if (atLeastOneObservation)
	{
		updateVertexIdx();
		if ((nodeCounter % 30 == 0) && PreviousVertexId > 0)
		{
			optimizer.initializeOptimization();
			optimizer.optimize(1);
			Vector3d tmpV;
			optimizer.vertex(PreviousVertexId)->getEstimateData(tmpV.data());
			location.x = tmpV(0);
			location.y = tmpV(1);
		}
	}
	return true;
}

void Localization::InitG2OGraph()
{
	optimizer.clear();
	LandmarkCount = 0;

	{
		VertexSE2 * l = new VertexSE2;
		l->setEstimate(Eigen::Vector3d(0, 0, 0));
		l->setFixed(true);
		l->setId(CenterL);
		optimizer.addVertex(l);
		LandmarkCount++;
	}
	{
		VertexSE2 * l = new VertexSE2;
		l->setEstimate(Eigen::Vector3d(A2, 0, 0));
		l->setFixed(true);
		l->setId(FrontL);
		optimizer.addVertex(l);
		LandmarkCount++;
	}
	{
		VertexSE2 * l = new VertexSE2;
		l->setEstimate(Eigen::Vector3d(-A2, 0, 0));
		l->setFixed(true);
		l->setId(BackL);
		optimizer.addVertex(l);
		LandmarkCount++;
	}
	{
		VertexSE2 * l = new VertexSE2;
		l->setEstimate(Eigen::Vector3d(0, -B2, 0));
		l->setFixed(true);
		l->setId(RightL);
		optimizer.addVertex(l);
		LandmarkCount++;
	}
	{
		VertexSE2 * l = new VertexSE2;
		l->setEstimate(Eigen::Vector3d(0, B2, 0));
		l->setFixed(true);
		l->setId(LeftL);
		optimizer.addVertex(l);
		LandmarkCount++;
	}
	PreviousVertexId = -1;
	CurrentVertexId = LandmarkCount;
	{
		VertexSE2 * l = new VertexSE2;
		l->setEstimate(
				Eigen::Vector3d(location.x, location.y, 0));
		l->setFixed(false);
		l->setId(LandmarkCount);
		optimizer.addVertex(l);
	}
}

// Send the required robot pose TF transform
void Localization::SendTransform(ros::Time now)
{
	Point3f RobotPose = GetLocalization();
	// Update the required transform
	tfLocField.stamp_ = now;
	tfLocField.setOrigin(tf::Vector3(RobotPose.x, RobotPose.y, 0.0));
	tfLocField.setRotation(
			tf::Quaternion(0.0, 0.0, sin(0.5 * RobotPose.z),
					cos(0.5 * RobotPose.z)));
	tfLocField.setData(tfLocField.inverse());

	// Send the required transform
	tfBroadcaster.sendTransform(tfLocField);
}

bool Localization::Init(string rName)
{
	if (rName != "unknown")
	{
		string topicN = "/remote/xs0/robottracker/" + rName + "/location";
		string topicN2 = "/remote/xs6/robottracker/" + rName + "/location";
		robotTracker_sub = nodeHandle.subscribe<geometry_msgs::PointStamped>(
				topicN, 1, &Localization::setremoteloc_callback, this);
		robotTracker2_sub = nodeHandle.subscribe<geometry_msgs::PointStamped>(
				topicN2, 1, &Localization::setremoteloc_callback, this);
	}

	InitG2OGraph();
	optimizer.setAlgorithm(optimizationAlgorithm);
	optimizer.setVerbose(false);

	return true;
}

