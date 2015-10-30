//Vision.cpp
// Created on: Apr 20, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#include <vision_module/Vision.hpp>

using namespace cv;
using namespace std;
#define SVAE_PICTURES false
#define NOT_BALLS_ERODE 6 //cm
#define NOT_BALLS_DILATE 15 //cm
#define RED_HSV Scalar(5, 255, 255)
#define MIN_Y_TO_HEAD_CONTROLLER 1

double MIN_LINE_CIRCLE_LENGHT = 0.4;
double MAX_LINE_CIRCLE_LENGHT = 1.5;
double MIN_LINE_LENGHT_FOR_LOC = 1.5;

enum LineType
{
	HorUndef,
	HorCenter,
	HorGoal,
	HorGoalNear,
	VerUndef,
	VerLeft,
	VerRight,
	VerLeftNear,
	VerRightNear,
	VerLeftT,
	VerRightT
};

/**
 *
 */
class LineContainer
{

public:
	inline LineContainer(LineSegment _line, LineType _type) :
			line(_line), type(_type)
	{

	}
	LineSegment line;
	LineType type;
};

bool Vision::update()
{
	ros::Time now = ros::Time::now();
	visionCounter++;

	_cameraProjections.Calibrate();

	if (!_cameraProjections.Update() && !cam->IsReady())
	{
		return false;
	}

	confidence = cam->TakeCapture();

	if (confidence < 0.75 && !cam->IsReady())
	{
		return false;
	}

	rawHSV = Mat(cam->rawImage.size(), CV_8UC3);
	cvtColor(cam->rawImage, rawHSV, CV_BGR2HSV);

	if (guiRawImg_pub.thereAreListeners())
	{
		guiRawImg = cam->rawImage.clone();
	}

	if (_cameraProjections.CalculateProjection())
	{
		Process(now);
	}

	if (!ballTargetUpdated)
	{
		ballTarget.header.stamp = ros::Time::now();
		ballTarget.point.z -= 0.01;
		if (ballTarget.point.z < 0)
			ballTarget.point.z = 0;
	}
	ballTarget_pub.publish(ballTarget);
	ballTargetUpdated = false;
	geometry_msgs::PointStamped robotPos;
	robotPos.header.stamp = ros::Time::now();
	robotPos.point.x = Localization.x;
	robotPos.point.y = Localization.y;
	robotPos.point.z = Localization.z;
	robotPos_pub.publish(robotPos);

	if (_hsvPresenter.Update()
			&& (visionCounter % params.debug.publishTime->get() == 0))
	{
		for (size_t i = 0; i < params.camCalibrator.clicked.size(); i++)
		{
			circle(guiRawImg, params.camCalibrator.clicked[i], 3, blueColor(),
					2);
		}
		topViewCircle.update();
		topViewCircleLR.update();
		topViewGoalPostLeft.update();
		topViewGoalPostRight.update();
		topViewGoalPostLeftLR.update();
		topViewGoalPostRightLR.update();
		LocPhiMarker.update();
		topViewLines.update();
		topViewLinesLR.update();
		topViewMarker.publish();
		topViewMarkerLR.publish();
		localizationMarker.publish();
		_hsvPresenter.Publish();
		filedConvecxImg_pub.publish(fieldConvectHull, MatPublisher::gray);
		fieldImg_pub.publish(fieldBinary, MatPublisher::gray);
		lineImg_pub.publish(lineBinary, MatPublisher::gray);
		goalImg_pub.publish(goalBinary, MatPublisher::gray);
		ballImg_pub.publish(ballBinary, MatPublisher::gray);
		_cameraProjections.Publish(guiRawImg,
				guiRawImg_pub.thereAreListeners());
		guiRawImg_pub.publish(guiRawImg, MatPublisher::bgr);
		ballMarker.marker.points.clear();
		LocMarker.marker.points.clear();
		topViewLines.marker.points.clear();
		topViewLinesLR.marker.points.clear();
		topViewCircle.marker.points.clear();
		topViewCircle.marker.pose.position.x = -100;
		topViewCircle.marker.pose.position.y = -100;
		topViewCircle.marker.pose.position.z = -100;
		topViewCircleLR.marker.points.clear();
		topViewCircleLR.marker.pose.position.x = -100;
		topViewCircleLR.marker.pose.position.y = -100;
		topViewCircleLR.marker.pose.position.z = -100;
		topViewGoalPostLeft.marker.points.clear();
		topViewGoalPostLeft.marker.pose.position.z = -100;
		topViewGoalPostRight.marker.points.clear();
		topViewGoalPostRight.marker.pose.position.z = -100;
		topViewGoalPostLeftLR.marker.points.clear();
		topViewGoalPostLeftLR.marker.pose.position.z = -100;
		topViewGoalPostRightLR.marker.points.clear();
		topViewGoalPostRightLR.marker.pose.position.z = -100;
		LocPhiMarker.marker.points.clear();
		LocPhiMarker.marker.pose.position.z = -100;
		topViewMarker.clear();
		topViewMarkerLR.clear();
		localizationMarker.clear();
	}

	return true;
}

void Vision::Process(ros::Time now)
{

	if (!params.ball.track->get())
	{
		head_control::LookAtTarget t;
		t.enabled = true;
		t.pitchEffort = -1;
		t.yawEffort = -1;
		head_pub.publish(t);
	}



	FieldHullReal.clear();

	fieldBinary = Mat::zeros(rawHSV.size(), CV_8UC1);
	Mat fieldBinaryRaw = Mat::zeros(rawHSV.size(), CV_8UC1);
	inRange(rawHSV,
			Scalar(params.field.h0->get(), params.field.s0->get(),
					params.field.v0->get()),
			Scalar(params.field.h1->get(), params.field.s1->get(),
					params.field.v1->get()), fieldBinaryRaw);
	fieldBinary = fieldBinaryRaw.clone();

	if (params.debug.maskField->get() && guiRawImg_pub.thereAreListeners())
	{
		Mat darker = Mat::zeros(guiRawImg.size(), CV_8UC3);
		darker.copyTo(guiRawImg, 255 - fieldBinaryRaw);
	}

	vector<Point> fieldPoints;
	vector<Point> fieldContourUndistort;
	vector<vector<Point> > allFieldContours;


	if (_fieldDetector.GetPoints(fieldBinary, fieldPoints, allFieldContours))
	{
		if (params.debug.showAllField->get()
				&& guiRawImg_pub.thereAreListeners())
		{
			drawContours(guiRawImg, allFieldContours, -1, Scalar(70, 220, 70),
					2, 8);
		}

		if (_cameraProjections._distorionModel.UndistortP(fieldPoints,
				fieldContourUndistort))
		{
			vector<Point> hullUndistort, hullUndistortMidP, hullField;
			convexHull(fieldContourUndistort, hullUndistort, false);

			for (size_t i = 0; i < hullUndistort.size(); i++)
			{
				size_t cur = i;
				size_t next = (i >= hullUndistort.size() - 1) ? 0 : i + 1;
				LineSegment ls(hullUndistort[cur], hullUndistort[next]);
				vector<Point2d> resMP = ls.GetMidPoints(4);
				for (size_t j = 0; j < resMP.size(); j++)
				{
					hullUndistortMidP.push_back(Point(resMP[j].x, resMP[j].y));

				}
			}

			if (_cameraProjections._distorionModel.DistortP(hullUndistortMidP,
					hullField))
			{

				vector<vector<Point> > hulls = vector<vector<Point> >(1,
						hullField);
				fieldConvectHull = Mat::zeros(fieldBinary.size(), CV_8UC1);

				vector<cv::Point> tmpBodyMaskContour =
						_fieldDetector.getBodyMaskContourInRaw(
								-Radian2Degree(0)); //todo
				bool considerBodyMask = (tmpBodyMaskContour.size() >= 6);

				//uchar *fieldMaskHullData = fieldMaskHull.data;
				//for (size_t y = 0; y < RAW_HEIGHT; y++)
				//{
				//	for (size_t x = 0; x < RAW_WIDTH; x++)
				//	{
				//		cv::Point p(x, y);
				//		if (cv::pointPolygonTest(FieldHull, p, false) >= 0)
				//		{
				//			if (considerBodyMask )
				//			{
				//				if (cv::pointPolygonTest(tmpBodyMaskContour, p, false) <= 0)
				//				{
				//					*fieldMaskHullData = 255;
				//				}
				//			}
				//			else
				//			{
				//				*fieldMaskHullData = 255;
				//			}

				//		}
				//		fieldMaskHullData++;
				//	}
				//}

				drawContours(fieldConvectHull, hulls, -1, grayWhite(),
				CV_FILLED, 8);

				if (considerBodyMask)
				{

					cv::Mat bodyMaskMat = cv::Mat::zeros(rawHSV.size(),
					CV_8UC1);
					vector<vector<cv::Point> > hullstmpBodyMaskContour = vector<
							vector<cv::Point> >(1, tmpBodyMaskContour);
					drawContours(bodyMaskMat, hullstmpBodyMaskContour, -1,
							grayWhite(),
							CV_FILLED, 8);
					fieldConvectHull -= bodyMaskMat;
				}

				if (hullField.size() > 1)
				{
					FieldHullRealCenter.x = 0;
					FieldHullRealCenter.y = 0;
					for (size_t fI = 0; fI < hullField.size(); fI++)
					{
						cv::Point2f realHP;
						if (!_cameraProjections.GetOnRealCordinate(
								hullField[fI], realHP))
						{
							ROS_ERROR("Error in programming!");
						}
						FieldHullReal.push_back(realHP);
						FieldHullRealCenter.x += realHP.x;
						FieldHullRealCenter.y += realHP.y;
					}

					FieldHullRealCenter.x /= FieldHullReal.size();
					FieldHullRealCenter.y /= FieldHullReal.size();
				}

				Mat gray;
				cvtColor(cam->rawImage, gray, CV_BGR2GRAY);
				if (params.debug.showFieldHull->get()
						&& guiRawImg_pub.thereAreListeners())
				{
					drawContours(guiRawImg, hulls, -1, yellowColor(), 2, 8);

				}
				const int OBJ_COUNT = 3;
				Mat binaryImgs[OBJ_COUNT];
				binaryImgs[0] = ballBinary = Mat::zeros(rawHSV.size(), CV_8UC1); //ball
				binaryImgs[1] = goalBinary = Mat::zeros(rawHSV.size(), CV_8UC1); //goal
				binaryImgs[2] = obstacleBinary = Mat::zeros(rawHSV.size(),
				CV_8UC1); //obstacle
				hsvRangeC ranges[OBJ_COUNT];
				ranges[0] = params.ball;
				ranges[1] = params.goal;
				ranges[2] = params.obstacle;
				bool inTemplate[OBJ_COUNT];
				inTemplate[0] = true;
				inTemplate[1] = false;
				inTemplate[2] = true;

				Mat channels[3], lineTmp, lineBinaryFull;
				split(rawHSV, channels);
				lineBinary = Mat::zeros(lineBinary.size(), CV_8UC1);
				blur(channels[2], lineTmp, Size(3, 3));

				Canny(lineTmp, lineTmp, params.cannyThreadshold->get(),
						params.cannyThreadshold->get() * 3, 3);

				lineTmp.copyTo(lineBinary, fieldConvectHull);
				channels[2].copyTo(lineBinaryFull, fieldConvectHull);

				_fieldDetector.FindInField(rawHSV, fieldConvectHull, binaryImgs,
						ranges, inTemplate, OBJ_COUNT);

				if (params.debug.maskBall->get()
						&& guiRawImg_pub.thereAreListeners())
				{

					Mat darker = Mat::zeros(guiRawImg.size(), CV_8UC3);
					darker.copyTo(guiRawImg, 255 - ballBinary);
				}
				if (params.debug.maskGoal->get()
						&& guiRawImg_pub.thereAreListeners())
				{

					Mat darker = Mat::zeros(guiRawImg.size(), CV_8UC3);
					darker.copyTo(guiRawImg, 255 - goalBinary);
				}
				if (params.debug.maskLine->get()
						&& guiRawImg_pub.thereAreListeners())
				{

					Mat darker = Mat::zeros(guiRawImg.size(), CV_8UC3);
					darker.copyTo(guiRawImg, 255 - lineBinary);
				}


				if (false)
				{
					Mat BigWhiteNotBall = ballBinary.clone();
					erode(BigWhiteNotBall, BigWhiteNotBall, Mat(),
							Point(-1, -1),
							NOT_BALLS_ERODE / params.topView.scale->get());

					dilate(BigWhiteNotBall, BigWhiteNotBall, Mat(),
							Point(-1, -1),
							NOT_BALLS_DILATE / params.topView.scale->get());

					ballImg_debug_pub.publish(BigWhiteNotBall,
							MatPublisher::gray, now);
					ballBinary = ballBinary - BigWhiteNotBall;
				}

				erode(ballBinary, ballBinary, Mat(), Point(-1, -1),
						params.ball.erode->get());

				dilate(ballBinary, ballBinary, Mat(), Point(-1, -1),
						params.ball.dilate->get());

				erode(goalBinary, goalBinary, Mat(), Point(-1, -1),
						params.goal.erode->get());

				dilate(goalBinary, goalBinary, Mat(), Point(-1, -1),
						params.goal.dilate->get());

				{
					vector<vector<cv::Point> > obstacleInRaw;
					vector<cv::Point2f> obsInReal;
					vector<cv::Point> obstaclePoint =
							_obstacleDetector.GetObstacleContours(
									obstacleBinary, obsInReal, obstacleInRaw,
									_cameraProjections);
				}

				vector<LineSegment> clusteredLines;
				Point2d resultCircle;

				{
					vector<LineSegment> resLines;

					if (_lineDetector.GetLines(rawHSV, fieldBinaryRaw,
							guiRawImg, guiRawImg_pub.thereAreListeners(),
							lineBinary, resLines))
					{
						vector<LineSegment> resLinesReal;

						vector<LineSegment> clusteredLinesImg;
						if (_cameraProjections.GetOnRealCordinate(resLines,
								resLinesReal))
						{

							Rect rec;
							rec.x = -1 * params.topView.width->get();
							rec.y = -1 * params.topView.width->get();
							rec.width = 2 * params.topView.width->get();
							rec.height = 2 * params.topView.width->get();

							if (MergeLinesMax(resLinesReal,
									params.line.AngleToMerge->get(),
									params.line.DistanceToMerge->get(),
									clusteredLines, rec))
							{

								vector<Point2d> circlePoint;
								for (size_t lineI = 0;
										lineI < clusteredLines.size(); lineI++)
								{
									double lLength =
											clusteredLines[lineI].GetLength();
									if (lLength > 1.5 || lLength < 0.2)
									{
										continue;
									}

									LineSegment pls =
											clusteredLines[lineI].PerpendicularLineSegment();
									for (size_t lineJ = lineI + 1;
											lineJ < clusteredLines.size();
											lineJ++)
									{

										double lLength2 =
												clusteredLines[lineJ].GetLength();
										if (lLength2 > 1.5 || lLength2 < 0.2)
										{
											continue;
										}
										if (dist3D_Segment_to_Segment(
												clusteredLines[lineJ],
												clusteredLines[lineI]) > 0.3)
											continue;
										LineSegment pls2 =
												clusteredLines[lineJ].PerpendicularLineSegment();
										Point2d intersect;
									}
								}



								if (_cameraProjections.GetOnImageCordinate(
										clusteredLines, clusteredLinesImg))
								{
									for (size_t i = 0;
											i < clusteredLines.size(); i++)
									{
										geometry_msgs::Point loc;
										loc.x = loc.y = Localization.y;
										loc.z = 0.0;

										geometry_msgs::Point p1;
										p1.x = clusteredLines[i].P1.x;
										p1.y = clusteredLines[i].P1.y;
										p1.z = 0.0;
										geometry_msgs::Point p2;
										p2.x = clusteredLines[i].P2.x;
										p2.y = clusteredLines[i].P2.y;
										p2.z = 0.0;

										geometry_msgs::Point ploc1;
										ploc1.x = clusteredLines[i].P1.x
												+ Localization.x;
										ploc1.y = clusteredLines[i].P1.y
												+ Localization.y;
										ploc1.z = 0.0;

										geometry_msgs::Point ploc2;
										ploc2.x = clusteredLines[i].P2.x
												+ Localization.x;
										ploc2.y = clusteredLines[i].P2.y
												+ Localization.y;
										ploc2.z = 0.0;

										topViewLines.marker.points.push_back(
												p1);
										topViewLines.marker.points.push_back(
												p2);
										topViewLinesLR.marker.points.push_back(
												ploc1);
										topViewLinesLR.marker.points.push_back(
												ploc2);

										if (params.debug.showCombinedLine->get()
												&& guiRawImg_pub.thereAreListeners())
										{
											line(guiRawImg,
													clusteredLinesImg[i].P1,
													clusteredLinesImg[i].P2,
													greenColor(), 3, 8);
											circle(guiRawImg,
													clusteredLinesImg[i].P1, 2,
													blueColor(), 2, 8);
											circle(guiRawImg,
													clusteredLinesImg[i].P2, 2,
													blueColor(), 2, 8);
										}

									}
								}
							}
							else
							{
								cout << "PPFEROOR" << endl;
							}
							for (size_t i = 0; i < resLines.size(); i++)
							{
								if (params.debug.showAllLine->get()
										&& guiRawImg_pub.thereAreListeners())
								{
									line(guiRawImg, resLines[i].P1,
											resLines[i].P2, redColor(), 1, 8);
								}
							}

						}
						else
						{
							cout << "EERRORR1" << endl;
						}
					}
				}

				vector<Point2f> goalPositionOnReal;
				{
					vector<LineSegment> resLines, alllL;

					bool goalRes = _goalDetector.GetPosts(gray,
							goalBinary.clone(), _cameraProjections, hullField,
							resLines, alllL, goalPositionOnReal,
							guiRawImg_pub.thereAreListeners(), guiRawImg);

					if (params.debug.showAllGoal->get()
							&& guiRawImg_pub.thereAreListeners())
					{
						for (size_t i = 0; i < alllL.size(); i++)
						{
							line(guiRawImg, alllL[i].P1, alllL[i].P2,
									Scalar(255, 0, 0), 2, 8);
						}
					}
					if (goalRes)
					{

						if (params.debug.showGoalVer->get()
								&& guiRawImg_pub.thereAreListeners())
						{
							for (size_t i = 0; i < resLines.size(); i++)
							{
								line(guiRawImg, resLines[i].P1, resLines[i].P2,
										yellowColor(), 3, 8);
							}
						}

						if (goalPositionOnReal.size() > 1)
						{

							topViewGoalPostRight.marker.pose.position.x =
									goalPositionOnReal[0].x;
							topViewGoalPostRight.marker.pose.position.y =
									goalPositionOnReal[0].y;
							topViewGoalPostRight.marker.pose.position.z = 0.6;

							topViewGoalPostLeft.marker.pose.position.x =
									goalPositionOnReal[1].x;
							topViewGoalPostLeft.marker.pose.position.y =
									goalPositionOnReal[1].y;
							topViewGoalPostLeft.marker.pose.position.z = 0.6;

							topViewGoalPostRightLR.marker.pose.position.x =
									goalPositionOnReal[0].x + Localization.x;
							topViewGoalPostRightLR.marker.pose.position.y =
									goalPositionOnReal[0].y + Localization.y;
							topViewGoalPostRightLR.marker.pose.position.z = 0.6;

							topViewGoalPostLeftLR.marker.pose.position.x =
									goalPositionOnReal[1].x + Localization.x;
							topViewGoalPostLeftLR.marker.pose.position.y =
									goalPositionOnReal[1].y + Localization.y;
							topViewGoalPostLeftLR.marker.pose.position.z = 0.6;

						}
						else if (goalPositionOnReal.size() > 0)
						{

							topViewGoalPostRight.marker.pose.position.x =
									goalPositionOnReal[0].x;
							topViewGoalPostRight.marker.pose.position.y =
									goalPositionOnReal[0].y;
							topViewGoalPostRight.marker.pose.position.z = 0.6;

							topViewGoalPostRightLR.marker.pose.position.x =
									goalPositionOnReal[0].x + Localization.x;
							topViewGoalPostRightLR.marker.pose.position.y =
									goalPositionOnReal[0].y + Localization.y;
							topViewGoalPostRightLR.marker.pose.position.z = 0.6;
						}

					}

				}


				{

					vector<cv::Rect> ballResult = _ballDetector.GetBallRect(
							rawHSV, hullField, fieldBinaryRaw, gray, ballBinary,
							_cameraProjections, guiRawImg,
							guiRawImg_pub.thereAreListeners());

					for (size_t bI = 0; bI < ballResult.size(); bI++)
					{
						Point2d b = RectToMyCircle(ballResult[bI]).Center;
						vector<Point> in;
						vector<Point2f> out;
						in.push_back(b);
						if (_cameraProjections.GetOnRealCordinate(in, out))
						{
							ballTarget.header.stamp = ros::Time::now();

							ballTarget.point.x = out[0].x;
							ballTarget.point.y = out[0].y;
							ballTarget.point.z = 1;
							if (ballTarget.point.z > 1)
								ballTarget.point.z = 1;
							ballTargetUpdated = true;

							ballMarker.update(out[0].x, out[0].y, 0);
							if (params.debug.showBall->get()
									&& guiRawImg_pub.thereAreListeners())
							{
								circle(guiRawImg, b,
										RectToMyCircle(ballResult[0]).radius,
										pinkColor(), 3);
							}

							if (params.ball.track->get())
							{
								if (abs(out[0].y) > 0.24)
								{
									head_control::LookAtTarget t;
									t.enabled = true;
									t.is_angular_data = true;
									t.is_relative = true;
									t.pitchEffort = 1;
									t.yawEffort = 1;
									t.vec.z = atan2(-out[0].y, out[0].x) / 3.5; //sign(bR.y) * 0.1; //Head_Control_Pos.z;
									t.vec.y = 0;
									t.vec.x = 0;
									head_pub.publish(t);
								}
							}
						}
					}
					if (ballResult.size() > 0)
					{

					}
					else
					{
						Head_Control_Pos.z = 0;
						head_control::LookAtTarget t;
						t.enabled = true;
						t.is_angular_data = true;
						t.is_relative = true;
						t.vec.z = 0;
						t.vec.y = 0;
						t.vec.x = 0;
						head_pub.publish(t);
					}
				}
				
				{
				  bool circleDetected=false;
				  const double A2 = A / 2.;
				  const double B2 = B / 2.;
				  double compassOffsetLoc=0;
					vector<LineContainer> AllLines;
			
					if (true)
					{

						LineSegment HorLine(cv::Point(0, -10),
								cv::Point(0, 10));
						LineSegment VerLine(cv::Point(10, 0),
								cv::Point(-10, 0));

						for (size_t i = 0; i < clusteredLines.size(); i++)
						{
							LineSegment lineSeg = clusteredLines[i];

							if (lineSeg.GetLength() > MIN_LINE_LENGHT_FOR_LOC)
							{
								cv::Point2d mid = lineSeg.GetMiddle();

								if (lineSeg.GetAbsMinAngleDegree(VerLine) < 45)
								{
									LineType thisType = VerUndef;
									double angleDiffVer =
											lineSeg.GetExteriorAngleDegree(
													VerLine);
									if (angleDiffVer < -90)
										angleDiffVer += 180;
									if (angleDiffVer > 90)
										angleDiffVer += -180;

									lowPass(angleDiffVer, compassOffsetLoc,
											getUpdateCoef(0.05, lineSeg));

									if (lineSeg.DistanceFromLine(
											cv::Point(0, 0)) > 0.7)
									{
										double estimatedY = 0;
										if (mid.y > 0) 
										{

											thisType = VerLeft;
											estimatedY = B2 - mid.y;
										}
										else 
										{
											thisType = VerRight;
											estimatedY = -B2 + abs(mid.y);
										}
										lowPass(estimatedY, Localization.y,
												getUpdateCoef(LOWPASSNORMAL,
														lineSeg));
									}
									else if (lineSeg.DistanceFromLine(
											FieldHullRealCenter) > 0.6
											&& cv::contourArea(FieldHullReal)
													> 4)
									{
										LineSegment l2Test = lineSeg;
										if (lineSeg.P1.x > lineSeg.P2.x)
										{
											l2Test.P1 = lineSeg.P2;
											l2Test.P2 = lineSeg.P1;
										}

										double estimatedY = 0;
										if (l2Test.GetSide(FieldHullRealCenter)
												< 0) 
										{
											thisType = VerLeftNear;
											estimatedY = B2 - mid.y;
										}
										else 
										{
											thisType = VerRightNear;
											estimatedY = -B2 + abs(mid.y);
										}
										lowPass(estimatedY, Localization.y,
												getUpdateCoef(LOWPASSNORMAL,
														lineSeg));
									}
									AllLines.push_back(
											LineContainer(lineSeg, thisType));
								}
								else
								{
									LineType thisType = HorUndef;
									double angleDiffHor =
											lineSeg.GetExteriorAngleDegree(
													HorLine);
									if (angleDiffHor < -90)
										angleDiffHor += 180;
									if (angleDiffHor > 90)
										angleDiffHor += -180;
									lowPass(angleDiffHor, compassOffsetLoc,
											getUpdateCoef(0.05, lineSeg));
								
									if (circleDetected
											&& DistanceFromLineSegment(lineSeg,
													resultCircle) < 1)
									{
										thisType = HorCenter;
										double estimatedX = -mid.x;

										lowPass(estimatedX, Localization.x,
												getUpdateCoef(LOWPASSNORMAL,
														lineSeg));
										
									}

									if (goalPositionOnReal.size() >= 2 
											&& lineSeg.DistanceFromLine(
													goalPositionOnReal[0]) < 0.5
											&& lineSeg.DistanceFromLine(
													goalPositionOnReal[1])
													< 0.5)
									{

										double estimatedX = 0;
										if (mid.x > 0) 
										{
											estimatedX = A2 - mid.x;
										}
										else 
										{
											estimatedX = -A2 + abs(mid.x);
										}
										lowPass(estimatedX, Localization.x,
												getUpdateCoef(LOWPASSSTRONG,
														lineSeg));
									}
									else if (goalPositionOnReal.size() == 1 
											&& lineSeg.DistanceFromLine(
													goalPositionOnReal[0])
													< 0.5)
									{

										double estimatedX = 0;
										if (mid.x > 0) 
										{
											estimatedX = A2 - mid.x;
										}
										else
										{
											estimatedX = -A2 + abs(mid.x);
										}
										lowPass(estimatedX, Localization.x,
												getUpdateCoef(LOWPASSNORMAL,
														lineSeg));
									}

									AllLines.push_back(
											LineContainer(lineSeg, thisType));
								}
							}
						}

						for (size_t i = 0; i < AllLines.size(); i++) 
						{
							LineContainer hI = AllLines[i];
							if (hI.type != HorUndef && hI.type != HorCenter)
								continue;
							for (size_t j = i; j < AllLines.size(); j++)
							{
								LineContainer vJ = AllLines[j];
								if (vJ.type < VerUndef)
									continue;
								int pNear = 0;
								if (pNear > 0)
								{
									cv::Point2d intersectP;
									if (vJ.line.IntersectLineForm(hI.line,
											intersectP))
									{
										if (GetDistance(intersectP, vJ.line.P1)
												> 0.8
												&& GetDistance(intersectP,
														vJ.line.P2) > 0.8) 
										{
											double estimatedX =
													-hI.line.GetMiddle().x;

											lowPass(estimatedX, Localization.x,
													getUpdateCoef(LOWPASSNORMAL,
															hI.line));
											AllLines[i].type = HorCenter;

										}
									}
								}
							}
						}

					}

				}

			}
		}
	}
}


bool Vision::Init()
{
	topViewLines.setType(visualization_msgs::Marker::LINE_LIST);

	topViewLines.setOrientation(1, 0, 0, 0);

	topViewLines.setScale(0.15);

	topViewLines.setColor(0, 1, 0);

	topViewGoalPostLeft.setType(visualization_msgs::Marker::CYLINDER);

	topViewGoalPostLeft.setOrientation(1, 0, 0, 0);

	topViewGoalPostLeft.setScale(0.1, 0.1, 1.1);

	topViewGoalPostLeft.setColor(1, 1, 0);

	topViewGoalPostRight.setType(visualization_msgs::Marker::CYLINDER);

	topViewGoalPostRight.setOrientation(1, 0, 0, 0);

	topViewGoalPostRight.setScale(0.1, 0.1, 1.1);

	topViewGoalPostRight.setColor(1, 1, 0);

	topViewCircle.setType(visualization_msgs::Marker::CYLINDER);

	topViewCircle.setOrientation(1, 0, 0, 0);

	topViewCircle.setScale(1.3, 1.3, 0.1);

	topViewCircle.setColor(1, 1, 1);

	topViewLinesLR.setType(visualization_msgs::Marker::LINE_LIST);

	topViewLinesLR.setOrientation(1, 0, 0, 0);

	topViewLinesLR.setScale(0.15);

	topViewLinesLR.setColor(0, 1, 0);

	topViewGoalPostLeftLR.setType(visualization_msgs::Marker::CYLINDER);

	topViewGoalPostLeftLR.setOrientation(1, 0, 0, 0);

	topViewGoalPostLeftLR.setScale(0.1, 0.1, 1.1);

	topViewGoalPostLeftLR.setColor(0, 0, 1);

	topViewGoalPostRightLR.setType(visualization_msgs::Marker::CYLINDER);

	topViewGoalPostRightLR.setOrientation(1, 0, 0, 0);

	topViewGoalPostRightLR.setScale(0.1, 0.1, 1.1);

	topViewGoalPostRightLR.setColor(0, 0, 1);

	topViewCircleLR.setType(visualization_msgs::Marker::CYLINDER);

	topViewCircleLR.setOrientation(1, 0, 0, 0);

	topViewCircleLR.setScale(1.3, 1.3, 0.1);

	topViewCircleLR.setColor(1, 1, 1);

	ballMarker.setOrientation(1, 0, 0, 0);

	ballMarker.setScale(0.3);

	ballMarker.setColor(1, 0, 0);

	LocMarker.setOrientation(1, 0, 0, 0);

	LocMarker.setScale(0.5);

	LocMarker.setColor(1, 1, 0);

	LocPhiMarker.setType(visualization_msgs::Marker::CYLINDER);

	LocPhiMarker.setOrientation(1, 1, 0, 0);

	LocPhiMarker.setScale(0.1, 0.1, 0.6);

	LocPhiMarker.setColor(1, 0, 0);

	if (false == cam->InitCameraDevice(true))
	{
		ROS_ERROR("vision_module Failed to initialize Camera!");
		return false;
	}
	ROS_INFO("vision_module Started");

//namedWindow("CameraCapture", CV_WINDOW_AUTOSIZE | CV_GUI_EXPANDED);

	head_control::LookAtTarget t;
	t.enabled = true;
	t.is_angular_data = true;
	t.is_relative = false;
	t.vec.z = 0;
	t.vec.y = 0.5;
	t.vec.x = 0;
	head_pub.publish(t);
	if (!_ballDetector.Init())
	{
		ROS_ERROR("Can't Initialize ball detector");
	}
	if (!_goalDetector.Init())
	{
		ROS_ERROR("Can't Initialize goal detector");
	}
	if (!_lineDetector.Init())
	{
		ROS_ERROR("Can't Initialize line detector");
	}
	if (!_fieldDetector.Init())
	{
		ROS_ERROR("Can't Initialize field detector");
	}
	if (!_obstacleDetector.Init())
	{
		ROS_ERROR("Can't Initialize obstacle detector");
	}
	if (!_cameraProjections.Init(cam->IsDummy()))
	{
		ROS_ERROR("Can't Initialize camera Projection model");
	}
	return true;
}

