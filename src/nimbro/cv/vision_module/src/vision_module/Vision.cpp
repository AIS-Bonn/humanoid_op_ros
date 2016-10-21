//Vision.cpp
// Created on: Apr 20, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#include <vision_module/Vision.hpp>

using namespace cv;
using namespace std;

bool Vision::update()
{
	if (!cam->IsReady() || cam->TakeCapture() < 0.75)
	{
		HAF_ERROR_THROTTLE(1, "Camera is not ready!");
		return false;
	}

	ros::Time capTime = cam->rawImageTime;
	visionCounter++;
	updateGuiImg = false;
	bool publishWebImg = false;
	bool publishGuiImg = false;
	if (cam->ShouldPublish() && webImg_pub.thereAreListeners(true))
	{
		updateGuiImg = true;
		publishWebImg = true;
	}
	if ((visionCounter % params.debug->publishTime.get() == 0)
			&& guiImg_pub.thereAreListeners(visionCounter % 30))
	{
		updateGuiImg = true;
		publishGuiImg = true;
	}

	ProjectionObj.Calibrate();
	if (!ProjectionObj.Update(capTime))
	{
		return false;
	}
	if (!loc.Update(ProjectionObj))
	{
		HAF_ERROR_THROTTLE(1, "Cannot update localization!");
		return false;
	}

	RawHSVImg = Mat(cam->rawImage.size(), CV_8UC3);
	cvtColor(cam->rawImage, RawHSVImg, CV_BGR2HSV);
	cvtColor(cam->rawImage, GrayImg, CV_BGR2GRAY);
	guiManager.Update(RawHSVImg, cam->rawImage);

	if (updateGuiImg)
	{
		guiImg = cam->rawImage.clone();
	}

	if (ProjectionObj.CalculateProjection())
	{
		Process(capTime);
	}
	else
	{
		HAF_ERROR_THROTTLE(1, "Cannot CalculateProjection!");
	}
	Point3d locLast = loc.GetLocalization();

	outputs.location.position.x = locLast.x;
	outputs.location.position.y = locLast.y;
	outputs.location.position.z = locLast.z;
	outputs.location.probability = 1;
	plotM.plotScalar(ProjectionObj.headingOffset, PM_HEADING_OFFSET);
	plotM.plotScalar(ProjectionObj.lastAvalibleTF, PM_LAST_AVALIBLE_TF);
	hsvPresenter.Update();

	if (updateGuiImg)
	{
		hsvPresenter.DrawOnInputMat(guiImg, updateGuiImg);
		ProjectionObj.Publish(guiImg, updateGuiImg);
		guiManager.Publish(GrayImg, guiImg, updateGuiImg);
		edgeImg_pub.publish(cannyImgInField, MatPublisher::gray);
		if (publishWebImg)
		{
			Mat tmpGuiImg;
			pyrDown(guiImg, tmpGuiImg);
			webImg_pub.publish(tmpGuiImg, MatPublisher::bgr);
		}
		if (publishGuiImg)
		{
			guiImg_pub.publish(guiImg, MatPublisher::bgr);
		}
	}

	loc.SendTransform(capTime);
	locM.update(0, 0, 0);
	locPhiM.marker.pose.position.x = 0.3;
	locPhiM.marker.pose.position.y = 0;
	locPhiM.marker.pose.position.z = 0;
	tf::Quaternion q = tf::createQuaternionFromRPY(M_PI / 2, 0, 0 + M_PI / 2);
	locPhiM.marker.pose.orientation.w = q.getW();
	locPhiM.marker.pose.orientation.x = q.getX();
	locPhiM.marker.pose.orientation.y = q.getY();
	locPhiM.marker.pose.orientation.z = q.getZ();
	egoCircleM.updateAdd();
	egoGoalPostLM.updateAdd();
	egoGoalPostRM.updateAdd();
	locPhiM.updateAdd();
	egoFieldM.updateAdd();
	egoLinesM.updateAdd();
	egoDetectionMarker.publish();
	localizationMarker.publish();
	egoBallM.marker.points.clear();
	locM.marker.points.clear();
	egoFieldM.marker.points.clear();
	egoLinesM.marker.points.clear();
	egoLinesM.marker.colors.clear();
	egoCircleM.marker.points.clear();
	egoCircleM.marker.pose.position.x = -100;
	egoCircleM.marker.pose.position.y = -100;
	egoCircleM.marker.pose.position.z = -100;
	egoGoalPostLM.marker.points.clear();
	egoGoalPostLM.marker.pose.position.z = -100;
	egoGoalPostRM.marker.points.clear();
	egoGoalPostRM.marker.pose.position.z = -100;
	locPhiM.marker.points.clear();
	locPhiM.marker.pose.position.z = -100;
	egoDetectionMarker.clear();
	egoDetectionMarker.reset();
	localizationMarker.clear();
	visionOutputs_pub.publish(outputs);
	if (params.ball->printBallDetection())
	{
		guiManager.writeGuiConsoleFormat(
				outputs.ball.detected ? pinkColor() : yellowColor(),
				" Ball = [X=%.2f, Y=%.2f] -> %.2f", outputs.ball.position.x,
				outputs.ball.position.y,
				GetDistance(
						Point2f(outputs.ball.position.x,
								outputs.ball.position.y)));
	}
	plotM.publish();
	plotM.clear();
	return true;
}

void Vision::Process(ros::Time capTime)
{
	int markerIdx = 0;
	outputs.obstacles.clear();
	//Update Obstacle
	{
		for (vector<ObstacleC>::iterator it = obstacles.begin();
				it != obstacles.end();)
		{

			if (!it->decayConfidence())
			{
				it = obstacles.erase(it);
			}
			else
			{
				++it;
			}
		}

		outputs.obstacles.resize(obstacles.size());
		vis_utils::CylinderMarker tmpMarker(&egoDetectionMarker, "/ego_floor",
				0.6, 0.1, "obstacle", true);

		for (size_t obsIdx = 0; obsIdx < obstacles.size(); obsIdx++)
		{

			outputs.obstacles[obsIdx].position.x = obstacles[obsIdx].Position.x;
			outputs.obstacles[obsIdx].position.y = obstacles[obsIdx].Position.y;
			outputs.obstacles[obsIdx].position.z = 0;
			outputs.obstacles[obsIdx].probability =
					obstacles[obsIdx].getConfidence();
			outputs.obstacles[obsIdx].id = obstacles[obsIdx].id;

			tmpMarker.setColor(0, 0, 0, obstacles[obsIdx].getConfidence());
			tmpMarker.setPosition(obstacles[obsIdx].Position.x,
					obstacles[obsIdx].Position.y, 0.2);
			egoDetectionMarker.updateDynamicMarker(markerIdx++, tmpMarker);
		}
	}
	Point3d locLast = loc.GetLocalization();

	fieldHullReal.clear();
	fieldHullRealRotated.clear();
	outputs.header.stamp = capTime;
	outputs.header.frame_id = "/ego_floor";

	Point2f lastBall = loc.getBall();
	outputs.ball.position.x = lastBall.x;
	outputs.ball.position.y = lastBall.y;
	outputs.ball.position.z = 0;

	outputs.ball.probability -= 0.01;
	if (outputs.ball.probability < 0)
	{
		outputs.ball.probability = 0;
	}

	outputs.ball.detected = false;
	outputs.goals.clear();

	if (params.ball->loadCascadeFile())
	{
		if (ballDetector.LoadCascade())
		{
			ROS_INFO("Cascade reloaded from file for ballDetector");
		}
		else
		{
			ROS_ERROR("Cascade reload failed for ballDetector");
		}
		if (guiManager.LoadCascade())
		{
			ROS_INFO("Cascade reloaded from file for guiManager");
		}
		else
		{
			ROS_ERROR("Cascade reload failed for guiManager");
		}
		params.ball->loadCascadeFile.set(false);
	}

	fieldBinary = Mat::zeros(RawHSVImg.size(), CV_8UC1);
	Mat fieldBinaryRaw = Mat::zeros(RawHSVImg.size(), CV_8UC1);
	inRange(RawHSVImg,
			Scalar(params.field->h0.get(), params.field->s0.get(),
					params.field->v0.get()),
			Scalar(params.field->h1.get(), params.field->s1.get(),
					params.field->v1.get()), fieldBinaryRaw);
	fieldBinary = fieldBinaryRaw.clone();

	if (params.field->showMask.get() && updateGuiImg)
	{
		Mat darker = Mat::zeros(guiImg.size(), CV_8UC3);
		darker.copyTo(guiImg, 255 - fieldBinaryRaw);
	}

	vector<Point> fieldPoints;
	vector<Point> fieldContourUndistort;
	vector<vector<Point> > allFieldContours;


	if (params.field->enable.get()
			&& fieldDetector.GetPoints(fieldBinary, fieldPoints,
					allFieldContours))
	{
		if (params.field->showDebug.get() && updateGuiImg)
		{
			drawContours(guiImg, allFieldContours, -1, Scalar(70, 220, 70), 2,
					8);
		}

		if (ProjectionObj.distorionModel.UndistortP(fieldPoints,
				fieldContourUndistort))
		{
			vector<Point> hullUndistort, hullUndistortMidP, hullField;
			convexHull(fieldContourUndistort, hullUndistort, false);

			const int NUM_MID_P = 3;
			const int COUNT_MID_P = pow(2, NUM_MID_P) + 1; //17
			hullUndistortMidP.reserve(COUNT_MID_P * hullUndistort.size());
			vector<Point2f> undistortedPointPool, realPointPool;
			undistortedPointPool.resize(COUNT_MID_P * hullUndistort.size());
			for (size_t i = 0; i < hullUndistort.size(); i++)
			{
				size_t cur = i;
				size_t next = (i >= hullUndistort.size() - 1) ? 0 : i + 1;
				LineSegment ls(hullUndistort[cur], hullUndistort[next]);
				vector<Point2d> resMP = ls.GetMidPoints(NUM_MID_P, true);
				for (size_t j = 0; j < COUNT_MID_P; j++)
				{
					undistortedPointPool[i * COUNT_MID_P + j] = resMP[j];
				}
			}

			if (ProjectionObj.GetOnRealCordinate_FromUndistorted(
					undistortedPointPool, realPointPool))
			{

				for (size_t i = 0; i < realPointPool.size(); i++)
				{
					Point2d unroatedCameraYaw = ProjectionObj.unroateCameraYaw(
							realPointPool[i]);
					if (unroatedCameraYaw.x > params.field->minAcceptX()
							&& GetDistance(realPointPool[i])
									< params.field->maxAcceptDistance())
					{
						hullUndistortMidP.push_back(
								Point(undistortedPointPool[i].x,
										undistortedPointPool[i].y));
					}
				}

				if (ProjectionObj.distorionModel.DistortP(hullUndistortMidP,
						hullField))
				{
					if (params.field->showDebug() && updateGuiImg)
					{
						for (size_t i = 0; i < hullField.size(); i++)
						{
							circle(guiImg, hullField[i], 4, redColor(), 3);
						}
					}

					vector<vector<Point> > hulls = vector<vector<Point> >(1,
							hullField);
					fieldConvectHull = Mat::zeros(fieldBinary.size(), CV_8UC1);

					vector<cv::Point> tmpBodyMaskContour =
							fieldDetector.getBodyMaskContourInRaw(
									-Radian2Degree(0)); //todo
					bool considerBodyMask = (tmpBodyMaskContour.size() >= 6);


					drawContours(fieldConvectHull, hulls, -1, grayWhite(),
					CV_FILLED, 8);

					if (considerBodyMask)
					{

						cv::Mat bodyMaskMat = cv::Mat::zeros(RawHSVImg.size(),
						CV_8UC1);
						vector<vector<cv::Point> > hullstmpBodyMaskContour =
								vector<vector<cv::Point> >(1,
										tmpBodyMaskContour);
						drawContours(bodyMaskMat, hullstmpBodyMaskContour, -1,
								grayWhite(), CV_FILLED, 8);
						fieldConvectHull -= bodyMaskMat;
					}

					if (hullField.size() > 1)
					{

						vector<Point2f> realHP;
						if (!ProjectionObj.GetOnRealCordinate(hullField,
								realHP))
						{
							ROS_ERROR("Error in programming!");
						}

						fieldHullRealCenter.x = 0;
						fieldHullRealCenter.y = 0;
						fieldHullReal.resize(realHP.size());
						for (size_t fI = 0; fI < realHP.size(); fI++)
						{
							fieldHullReal[fI] = realHP[fI];
							fieldHullRealCenter.x += realHP[fI].x;
							fieldHullRealCenter.y += realHP[fI].y;
						}
						fieldHullRealCenter.x /= fieldHullReal.size();
						fieldHullRealCenter.y /= fieldHullReal.size();
					}

					if (fieldHullReal.size() > 3)
					{
						cv::approxPolyDP(fieldHullReal, fieldHullReal,
								cv::arcLength(fieldHullReal, true)
										* params.field->approxPoly.get(), true);
						fieldHullRealRotated =
								ProjectionObj.RotateTowardHeading(
										fieldHullReal);
						for (size_t i = 0; i < fieldHullRealRotated.size(); i++)
						{

							geometry_msgs::Point ploc1;
							ploc1.x = fieldHullReal[i].x;
							ploc1.y = fieldHullReal[i].y;
							ploc1.z = 0.0;

							int next = (
									(i == fieldHullReal.size() - 1) ? 0 : i + 1);
							geometry_msgs::Point ploc2;
							ploc2.x = fieldHullReal[next].x;
							ploc2.y = fieldHullReal[next].y;
							ploc2.z = 0.0;

							egoFieldM.marker.points.push_back(ploc1);
							egoFieldM.marker.points.push_back(ploc2);

						}
					}

					if (params.field->showResult.get() && updateGuiImg)
					{
						drawContours(guiImg, hulls, -1, yellowColor(), 2, 8);
					}
					const int OBJ_COUNT = 3;
					Mat binaryImgs[OBJ_COUNT];
					binaryImgs[0] = ballBinary = Mat::zeros(RawHSVImg.size(),
					CV_8UC1); //ball
					binaryImgs[1] = goalBinary = Mat::zeros(RawHSVImg.size(),
					CV_8UC1); //goal
					binaryImgs[2] = obstacleBinary = Mat::zeros(
							RawHSVImg.size(),
							CV_8UC1); //obstacle
					hsvRangeC ranges[OBJ_COUNT];
					ranges[0] = params.ball->GetHSVRange();
					ranges[1] = params.goal->GetHSVRange();
					ranges[2] = params.obstacle->GetHSVRange();
					bool inTemplate[OBJ_COUNT];
					inTemplate[0] = true;
					inTemplate[1] = false;
					inTemplate[2] = true;

					fieldDetector.FindInField(RawHSVImg, fieldConvectHull,
							binaryImgs, ranges, inTemplate, OBJ_COUNT);

					if (params.ball->showMask.get() && updateGuiImg)
					{
						Mat darker = Mat::zeros(guiImg.size(), CV_8UC3);
						darker.copyTo(guiImg, 255 - ballBinary);
					}
					if (params.goal->showMask.get() && updateGuiImg)
					{
						Mat darker = Mat::zeros(guiImg.size(), CV_8UC3);
						darker.copyTo(guiImg, 255 - goalBinary);
					}

					if (params.obstacle->mask.get() && updateGuiImg)
					{
						Mat darker = Mat::zeros(guiImg.size(), CV_8UC3);
						darker = whiteColor();
						darker.copyTo(guiImg, 255 - obstacleBinary);
					}

					if (params.ball->erode.get() > 0)
					{
						erode(ballBinary, ballBinary, Mat(), Point(-1, -1),
								params.ball->erode.get());
					}
					if (params.ball->dilate.get() > 0)
					{
						dilate(ballBinary, ballBinary, Mat(), Point(-1, -1),
								params.ball->dilate.get());
					}
					if (params.ball->erode2.get() > 0)
					{
						erode(ballBinary, ballBinary, Mat(), Point(-1, -1),
								params.ball->erode2.get());
					}
					if (params.ball->dilate2.get() > 0)
					{
						dilate(ballBinary, ballBinary, Mat(), Point(-1, -1),
								params.ball->dilate2.get());
					}

					if (params.obstacle->erode.get() > 0)
					{
						erode(obstacleBinary, obstacleBinary, Mat(),
								Point(-1, -1), params.obstacle->erode.get());
					}
					if (params.obstacle->dilate.get() > 0)
					{
						dilate(obstacleBinary, obstacleBinary, Mat(),
								Point(-1, -1), params.obstacle->dilate.get());
					}

					Mat channels[3], cannyImg;
					split(RawHSVImg, channels);
					cannyImgInField = Mat::zeros(cannyImgInField.size(),
					CV_8UC1);
					blur(channels[2], cannyImg,
							Size(params.line->blurSize.get(),
									params.line->blurSize.get()));

					Canny(cannyImg, cannyImg,
							params.line->cannyThreadshold.get(),
							params.line->cannyThreadshold.get() * 3,
							params.line->cannyaperture.get());
					//Obstacle
					if (params.obstacle->enable.get())
					{
						cpu_timer timer_obst;
						vector<cv::Point2f> obsInReal;
						if (obstacleDetector.GetObstacleContours(obstacleBinary,
								obsInReal, guiImg, ProjectionObj, updateGuiImg))
						{
							for (size_t i = 0; i < obsInReal.size(); i++)
							{
								bool registered = false;
								for (size_t obsIdx = 0;
										obsIdx < obstacles.size(); obsIdx++)
								{
									if (obstacles[obsIdx].update(obsInReal[i],
											1))
									{
										registered = true;
										break;
									}
								}

								if (!registered)
								{
									long unsigned int id = 1;
									if (obstacles.size() > 0)
									{
										id = obstacles[obstacles.size() - 1].id
												+ 1;
									}
									obstacles.push_back(
											ObstacleC(obsInReal[i], 1, id));
								}
							}
						}
						plotM.plotScalar(timer_obst.elapsed().wall / 1000000.0,
								PM_OBSTACLE_TIME);
					}

					if (params.ball->enable.get())
					{
						cpu_timer timer_ball;
						vector<BallCircleC> ballResults = ballDetector.GetBall(
								RawHSVImg, hullField, fieldBinaryRaw, GrayImg,
								ballBinary, fieldConvectHull, ProjectionObj,
								cannyImg, &guiManager, guiImg, updateGuiImg);
						if (ballResults.size() > 0)
						{
							loc.setBall(ballResults[0].RealPos);
							egoBallM.update(ballResults[0].RealPos.x,
									ballResults[0].RealPos.y, 0);
							outputs.ball.position.x = ballResults[0].RealPos.x;
							outputs.ball.position.y = ballResults[0].RealPos.y;
							outputs.ball.position.z = 0;
							outputs.ball.probability = 1;
							outputs.ball.detected = true;
						}
						plotM.plotScalar(timer_ball.elapsed().wall / 1000000.0,
								PM_BALL_TIME);
					}
					cannyImg.copyTo(cannyImgInField, fieldConvectHull);

					vector<LineSegment> clusteredLines;
					vector<LineSegment> clusteredLinesRotated;
					bool LinedetectionOK = false;

					if (params.line->enable.get())
					{
						cpu_timer timer_line;
						Rect topViewBox;
						topViewBox.x = -1 * params.topView->width.get();
						topViewBox.y = -1 * params.topView->width.get();
						topViewBox.width = 2 * params.topView->width.get();
						topViewBox.height = 2 * params.topView->width.get();
						vector<LineSegment> resLines;
						if (lineDetector.GetLines(RawHSVImg, fieldBinaryRaw,
								guiImg, ProjectionObj, updateGuiImg,
								cannyImgInField,
								IMGBOX, resLines))
						{
							vector<LineSegment> resLinesReal;
							if (ProjectionObj.GetOnRealCordinate(resLines,
							resLinesReal))
							{
								if (MergeLinesMax(resLinesReal,
								params.line->AngleToMerge.get(),
								params.line->DistanceToMerge.get(),
								clusteredLines, topViewBox))
								{
									clusteredLinesRotated =
									ProjectionObj.RotateTowardHeading(
									clusteredLines);
									LinedetectionOK = true;
									egoLinesM.marker.colors.resize(
									clusteredLines.size() * 2);
									egoLinesM.marker.points.resize(
									clusteredLines.size() * 2);
									for (size_t i = 0; i < clusteredLines.size();
									i++)
									{
										egoLinesM.marker.points[i * 2].x =
										clusteredLines[i].P1.x;
										egoLinesM.marker.points[i * 2].y =
										clusteredLines[i].P1.y;
										egoLinesM.marker.points[i * 2].z = 0.0;

										egoLinesM.marker.points[i * 2 + 1].x =
										clusteredLines[i].P2.x;
										egoLinesM.marker.points[i * 2 + 1].y =
										clusteredLines[i].P2.y;
										egoLinesM.marker.points[i * 2 + 1].z =
										0.0;

										double lenLine =
										clusteredLinesRotated[i].GetLength();
										std_msgs::ColorRGBA colLine;

										if (lenLine > params.loc->minLineLen.get())
										{
											colLine.a =
											clusteredLinesRotated[i].getProbability();
											colLine.r = 0.1;
											colLine.g = 0.9;
											colLine.b = 0.2;
										}
										else if (lenLine
										> params.circle->maxLineLen.get()
										|| lenLine
										< params.circle->minLineLen.get())
										{
											colLine.a = 0.2;
											colLine.r = 0.1;
											colLine.g = 0.2;
											colLine.b = 0.9;
										}
										else
										{
											colLine.a =
											clusteredLinesRotated[i].getProbability();
											colLine.r = 0.1;
											colLine.g = 0.2;
											colLine.b = 0.9;
										}
										egoLinesM.marker.colors[i * 2] =
										egoLinesM.marker.colors[i * 2
										+ 1] = colLine;
									}

									if (params.line->showResult.get()
									&& updateGuiImg)
									{
										vector<LineSegment> clusteredLinesImg;
										if (ProjectionObj.GetOnImageCordinate(
										clusteredLines, clusteredLinesImg))
										{
											for (size_t i = 0;
											i < clusteredLinesImg.size();
											i++)
											{
												line(guiImg,
												clusteredLinesImg[i].P1,
												clusteredLinesImg[i].P2,
												greenColor(), 3, 8);
												circle(guiImg,
												clusteredLinesImg[i].P1, 2,
												blueColor(), 2, 8);
												circle(guiImg,
												clusteredLinesImg[i].P2, 2,
												blueColor(), 2, 8);
											}
										}
									}
								}
								if (params.line->showAllLine.get() && updateGuiImg)
								{
									for (size_t i = 0; i < resLines.size(); i++)
									{
										line(guiImg, resLines[i].P1, resLines[i].P2,
										blueMeloColor(), 1, 8);
									}
								}
							}
						}
						plotM.plotScalar(timer_line.elapsed().wall / 1000000.0,
								PM_LINE_TIME);
					}

					Point2d resultCircleRotated;
					bool confiused = false;
					Point2d resultCircle;
					bool circleDetected = false;

					if (LinedetectionOK && params.circle->enable.get())
					{
						cpu_timer timer_circle;
						if (circleDetector.GetCircle(loc.H2, clusteredLines,
								confiused, resultCircle, resultCircleRotated,
								ProjectionObj))
						{
							circleDetected = true;

							if (confiused)
							{
								egoCircleM.setColor(1, 1, 0, 0.7);
							}
							else
							{
								egoCircleM.setColor(1, 1, 1, 0.7);
							}
							egoCircleM.marker.pose.position.x = resultCircle.x;
							egoCircleM.marker.pose.position.y = resultCircle.y;
							egoCircleM.marker.pose.position.z = 0.01;
						}
						plotM.plotScalar(
								timer_circle.elapsed().wall / 1000000.0,
								PM_CIRCLE_TIME);
					}

					if (params.line->showMask.get() && updateGuiImg)
					{
						Mat lineBinary = Mat::zeros(RawHSVImg.size(), CV_8UC1);
						inRange(RawHSVImg,
								Scalar(params.line->h0.get(),
										params.line->s0.get(),
										params.line->v0.get()),
								Scalar(params.line->h1.get(),
										params.line->s1.get(),
										params.line->v1.get()), lineBinary);
						Mat darker = Mat::zeros(guiImg.size(), CV_8UC3);
						darker.copyTo(guiImg, 255 - lineBinary);
					}
					else if (params.line->showCanny.get() && updateGuiImg)
					{
						Mat darker = Mat::zeros(guiImg.size(), CV_8UC3);
						darker.copyTo(guiImg, 255 - cannyImg);
					}

					vector<Point2f> goalPositionOnReal;
					vector<Point2f> goalPositionOnRealRotated;

					if (params.goal->enable.get())
					{
						cpu_timer timer_goal;
						vector<LineSegment> resLines, alllL;

						bool goalRes = goalDetector.GetPosts(cannyImg,
								RawHSVImg, GrayImg, goalBinary.clone(),
								ProjectionObj, hullField, resLines, alllL,
								goalPositionOnReal, updateGuiImg, guiImg);

						if (params.goal->showAllLines.get() && updateGuiImg)
						{
							for (size_t i = 0; i < alllL.size(); i++)
							{
								line(guiImg, alllL[i].P1, alllL[i].P2,
										Scalar(255, 0, 0), 2, 8);
							}
						}
						if (goalRes)
						{
							if (params.goal->showResLine.get() && updateGuiImg)
							{
								for (size_t i = 0; i < resLines.size(); i++)
								{
									line(guiImg, resLines[i].P1, resLines[i].P2,
											yellowColor(), 3, 8);
								}
							}
							goalPositionOnRealRotated =
									ProjectionObj.RotateTowardHeading(
											goalPositionOnReal);
							outputs.goals.resize(goalPositionOnReal.size());
							for (size_t i = 0; i < goalPositionOnReal.size();
									i++)
							{
								outputs.goals[i].position.x =
										goalPositionOnReal[i].x;
								outputs.goals[i].position.y =
										goalPositionOnReal[i].y;
								outputs.goals[i].position.z = 0;
								outputs.goals[i].probability = 1;
							}

							if (goalPositionOnReal.size() > 1)
							{

								egoGoalPostRM.marker.pose.position.x =
										goalPositionOnReal[0].x;
								egoGoalPostRM.marker.pose.position.y =
										goalPositionOnReal[0].y;
								egoGoalPostRM.marker.pose.position.z = 0.6;

								egoGoalPostLM.marker.pose.position.x =
										goalPositionOnReal[1].x;
								egoGoalPostLM.marker.pose.position.y =
										goalPositionOnReal[1].y;
								egoGoalPostLM.marker.pose.position.z = 0.6;

							}
							else if (goalPositionOnReal.size() > 0)
							{
								egoGoalPostRM.marker.pose.position.x =
										goalPositionOnReal[0].x;
								egoGoalPostRM.marker.pose.position.y =
										goalPositionOnReal[0].y;
								egoGoalPostRM.marker.pose.position.z = 0.6;
							}
						}
						plotM.plotScalar(timer_goal.elapsed().wall / 1000000.0,
								PM_GOAL_TIME);
					}


					if (params.loc->enable.get())
					{
						cpu_timer timer_loc;
						vector<LineContainer> AllLines;
						vector<FeatureContainer> AllFeature;
						if (loc.Calculate(clusteredLinesRotated, circleDetected,
								fieldHullRealCenter, fieldHullReal,
								resultCircleRotated, goalPositionOnRealRotated,
								confiused, AllLines, AllFeature))
						{

							vis_utils::TextMarker tmpMarker(&egoDetectionMarker,
									"/loc_field", 0.2, "lineLabel", true);
							for (size_t i = 0; i < AllLines.size(); i++)
							{

								tmpMarker.setColor(
										1.
												* (AllLines[i].type
														/ (double) LTRES),
										1.
												* (1
														- (AllLines[i].type
																/ (double) LTRES)),
										0 ? AllLines[i].type >= VerUndef : 1,
										1);
								tmpMarker.setText(
										LineTypeName[AllLines[i].type]);
								Point2f mid =
										AllLines[i].lineTransformed.GetMiddle();
								tmpMarker.setPosition(mid.x + locLast.x,
										mid.y + locLast.y, 0.2);
								egoDetectionMarker.updateDynamicMarker(
										markerIdx++, tmpMarker);
							}
							for (size_t i = 0; i < AllFeature.size(); i++)
							{
								tmpMarker.setColor(1, 0, 1, 1);
								tmpMarker.setText(AllFeature[i].type);
								tmpMarker.setPosition(
										locLast.x + AllFeature[i].position.x,
										locLast.y + AllFeature[i].position.y,
										0.2);
								egoDetectionMarker.updateDynamicMarker(
										markerIdx++, tmpMarker);
							}

						}
						plotM.plotScalar(timer_loc.elapsed().wall / 1000000.0,
								PM_LOCALIZATION_TIME);
					}
				}
			}
		}
	}

}

bool Vision::Init()
{
	ROS_INFO("vision_module Started!");

	egoFieldM.setType(visualization_msgs::Marker::LINE_LIST);

	egoFieldM.setOrientation(1, 0, 0, 0);

	egoFieldM.setScale(0.05);

	egoFieldM.setColor(0, 0, 0);

	egoLinesM.setType(visualization_msgs::Marker::LINE_LIST);

	egoLinesM.setOrientation(1, 0, 0, 0);

	egoLinesM.setScale(0.15);

	egoLinesM.setColor(0, 1, 0);

	egoGoalPostLM.setType(visualization_msgs::Marker::CYLINDER);

	egoGoalPostLM.setOrientation(1, 0, 0, 0);

	egoGoalPostLM.setScale(0.1, 0.1, 1.1);

	egoGoalPostLM.setColor(0, 0, 1);

	egoGoalPostRM.setType(visualization_msgs::Marker::CYLINDER);

	egoGoalPostRM.setOrientation(1, 0, 0, 0);

	egoGoalPostRM.setScale(0.1, 0.1, 1.1);

	egoGoalPostRM.setColor(0, 0, 1);

	egoCircleM.setType(visualization_msgs::Marker::CYLINDER);

	egoCircleM.setOrientation(1, 0, 0, 0);

	egoCircleM.setScale(1.3, 1.3, 0.1);

	egoCircleM.setColor(1, 1, 1);

	egoBallM.setOrientation(1, 0, 0, 0);

	egoBallM.setScale(0.3);

	egoBallM.setColor(1, 0, 0);

	locM.setOrientation(1, 0, 0, 0);

	locM.setScale(0.5);

	locM.setColor(1, 1, 0);

	locPhiM.setType(visualization_msgs::Marker::CYLINDER);

	locPhiM.setOrientation(1, 1, 0, 0);

	locPhiM.setScale(0.1, 0.1, 0.6);

	locPhiM.setColor(1, 0, 0);

	if (!cam->InitCameraDevice(true))
	{
		ROS_ERROR("vision_module Failed to initialize Camera!");
		return false;
	}
	if (!ballDetector.Init())
	{
		ROS_ERROR("Can't Initialize ball detector");
		return false;
	}
	if (!goalDetector.Init())
	{
		ROS_ERROR("Can't Initialize goal detector");
		return false;
	}
	if (!lineDetector.Init())
	{
		ROS_ERROR("Can't Initialize line detector");
		return false;
	}
	if (!circleDetector.Init())
	{
		ROS_ERROR("Can't Initialize circle detector");
		return false;
	}
	if (!fieldDetector.Init())
	{
		ROS_ERROR("Can't Initialize field detector");
		return false;
	}
	if (!obstacleDetector.Init())
	{
		ROS_ERROR("Can't Initialize obstacle detector");
		return false;
	}
	if (!ProjectionObj.Init(cam->IsDummy()))
	{
		ROS_ERROR("Can't Initialize camera Projection model");
		return false;
	}
	if (!guiManager.Init())
	{
		ROS_ERROR("Can't Initialize guimanager");
		return false;
	}
	if (!loc.Init(rName))
	{
		ROS_ERROR("Can't Initialize localization");
		return false;
	}

	ROS_INFO("Init Finished Successfully");

	return true;
}

