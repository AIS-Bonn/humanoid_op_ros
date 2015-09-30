//BallDetector.cpp
// Created on: May 12, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <vision_module/soccer_objects/BallDetector.hpp>

std::vector<cv::Rect> BallDetector::GetBallRect(const Mat &rawHSV,
		const vector<Point> &hullField, const Mat &fieldBinaryRaw, Mat &rawGray,
		const Mat &ballMask, CameraProjections &projection, Mat &guiImg,
		bool SHOWGUI)
{

	int distanceToCascade = params.ball.dist2cascade->get();
	double ratioHT = params.ball.BiggerRectHT->get();
	double ratioHB = params.ball.BiggerRectHB->get();
	double ratioW = params.ball.BiggerRectW->get();
	vector<cv::Rect> resGetNearestContour;
	vector<vector<cv::Point> > contours;
	lastBallDetection.clear();
	if (ballMask.empty())
	{
		return resGetNearestContour;
	}

	cv::Mat ballMaskBlur;
	cv::GaussianBlur(ballMask, ballMaskBlur, cv::Size(15, 15), 2, 2);
	cv::Mat ballContours = cv::Mat::zeros(rawHSV.size(), CV_8UC1);

	findContours(ballMask.clone(),
			contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	vector<MyCircle> DetectedNearBall;
	vector<MyCircle> DetectedFarBall;
	vector<MyCircle> CandidateBall;
	vector<MyCircle> PoorCandidateBall;
	std::sort(contours.begin(), contours.end(), sorter(&projection));

	for (size_t i = 0; i < contours.size(); i++)
	{
		cv::approxPolyDP(contours[i], contours[i],
				cv::arcLength(contours[i], true)
						* params.ball.approxPoly->get(), true);
		vector<cv::Point2d> centeres;
		vector<double> radiuses;
		if (contours[i].size() > 5 && cv::contourArea(contours[i]) > 50)
		{
			for (size_t lineI = 0; lineI < contours[i].size(); lineI++)
			{

				LineSegment ls;
				LineSegment ls2;
				LineSegment ls3;
				if (lineI == contours[i].size() - 1)
				{
					ls.P1 = contours[i][lineI];
					ls.P2 = contours[i][0];
					ls2.P1 = contours[i][0];
					ls2.P2 = contours[i][1];
					ls3.P1 = contours[i][1];
					ls3.P2 = contours[i][2];
				}
				else if (lineI == contours[i].size() - 2)
				{
					ls.P1 = contours[i][lineI];
					ls.P2 = contours[i][lineI + 1];
					ls2.P1 = contours[i][lineI + 1];
					ls2.P2 = contours[i][0];
					ls3.P1 = contours[i][0];
					ls3.P2 = contours[i][1];

				}
				else if (lineI == contours[i].size() - 3)
				{
					ls.P1 = contours[i][lineI];
					ls.P2 = contours[i][lineI + 1];
					ls2.P1 = contours[i][lineI + 1];
					ls2.P2 = contours[i][lineI + 2];
					ls3.P1 = contours[i][lineI + 2];
					ls3.P2 = contours[i][0];
				}
				else
				{
					ls.P1 = contours[i][lineI];
					ls.P2 = contours[i][lineI + 1];
					ls2.P1 = contours[i][lineI + 1];
					ls2.P2 = contours[i][lineI + 2];
					ls3.P1 = contours[i][lineI + 2];
					ls3.P2 = contours[i][lineI + 3];
				}

				LineSegment pls = ls.PerpendicularLineSegment();
				LineSegment pls2 = ls2.PerpendicularLineSegment();
				LineSegment pls3 = ls3.PerpendicularLineSegment();

				cv::Point2d intersect1;
				if (pls.IntersectLineForm(pls2, intersect1)
						&& pointPolygonTest(contours[i], intersect1, false) > 0)
				{
					cv::Point2d intersect2;
					if (pls.IntersectLineForm(pls3, intersect2)
							&& pointPolygonTest(contours[i], intersect2, false)
									> 0)
					{
						cv::Point2d intersect3;
						if (pls2.IntersectLineForm(pls3, intersect3)
								&& pointPolygonTest(contours[i], intersect3,
										false) > 0)
						{
							double distanceMax = std::max(
									GetDistance(intersect1, intersect2),
									std::max(
											GetDistance(intersect1, intersect3),
											GetDistance(intersect2,
													intersect3)));
							double lenAvg1 = (ls.DistanceFromLine(intersect1)
									+ ls.DistanceFromLine(intersect2)
									+ ls.DistanceFromLine(intersect3)) / 3.;
							double lenAvg2 = (ls2.DistanceFromLine(intersect1)
									+ ls2.DistanceFromLine(intersect2)
									+ ls2.DistanceFromLine(intersect3)) / 3.;
							double lenAvg3 = (ls3.DistanceFromLine(intersect1)
									+ ls3.DistanceFromLine(intersect2)
									+ ls3.DistanceFromLine(intersect3)) / 3.;
							double lenAvg = lenAvg1 + lenAvg2 + lenAvg3;
							lenAvg /= 3.;
							if (distanceMax < lenAvg * 0.4)
							{
								centeres.push_back(
										GetAverage(intersect1,
												GetAverage(intersect2,
														intersect3)));
								radiuses.push_back(lenAvg);

							}

						}
					}
				}
			}
		}

		if (centeres.size() > 1)
		{
			double maxRadius = -9999;
			int maxIndex = -1;
			for (size_t ic = 0; ic < centeres.size(); ic++)
			{
				if (radiuses[ic] > maxRadius)
				{
					maxRadius = radiuses[ic];
					maxIndex = ic;
				}
			}

			cv::Rect recCircle(centeres[maxIndex].x - radiuses[maxIndex],
					centeres[maxIndex].y - radiuses[maxIndex],
					radiuses[maxIndex] * 2, radiuses[maxIndex] * 2);
			if (checkDistance_Rec(recCircle, projection)
					&& checkHistogram(rawHSV, fieldBinaryRaw,
							centeres[maxIndex], radiuses[maxIndex],
							params.ball.histogramH->get(),
							params.ball.histogramS->get(),
							params.ball.histogramV->get()))
			{

				double distanceToZero = GetDistance(
						cv::Point2d((params.camera.width->get() / 2),
								(params.camera.height->get())),
						centeres[maxIndex]);

				if (distanceToZero < distanceToCascade)
				{
					MyCircle circleToAdd;
					circleToAdd.Center = centeres[maxIndex];
					circleToAdd.radius = radiuses[maxIndex];
					DetectedNearBall.push_back(circleToAdd);
				}
				else
				{
					MyCircle circleToAdd;
					circleToAdd.Center = centeres[maxIndex];
					circleToAdd.radius = radiuses[maxIndex];
					CandidateBall.push_back(circleToAdd);
				}
				if (SHOWGUI && params.debug.showBallD->get())
				{

					circle(guiImg, centeres[maxIndex], radiuses[maxIndex],
							yellowColor(), 3, 8, 0);
					cv::drawContours(guiImg, contours, i, greenColor(), 2);
				}
			}
			else
			{
				MyCircle circleToAdd;
				circleToAdd.Center = centeres[maxIndex];
				circleToAdd.radius = radiuses[maxIndex];
				CandidateBall.push_back(circleToAdd);
				if (SHOWGUI && params.debug.showBallD->get())
				{
					cv::drawContours(guiImg, contours, i, blueColor(), 2);
				}
			}

		}
		else if (centeres.size() > 0)
		{
			MyCircle circleToAdd;
			circleToAdd.Center = centeres[0];
			circleToAdd.radius = radiuses[0];
			CandidateBall.push_back(circleToAdd);
			if (SHOWGUI && params.debug.showBallD->get())
					{
			cv::drawContours(guiImg, contours, i, pinkMeloColor(), 2);
					}
		}
		else
		{
			if (contours[i].size() <= 5 && contours[i].size() > 3
					&& cv::contourArea(contours[i]) > 20)
			{
				MyCircle circleToAdd;
				cv::minEnclosingCircle(contours[i], circleToAdd.Center,
						circleToAdd.radius);
				CandidateBall.push_back(circleToAdd);
			}
			if (SHOWGUI && params.debug.showBallD->get())
			{
				cv::drawContours(guiImg, contours, i, redColor(), 2);
			}
		}
	}

	if (DetectedNearBall.size() > 0)
	{
		for (size_t resCounter = 0; resCounter < DetectedNearBall.size();
				resCounter++)
		{
			resGetNearestContour.push_back(
					MyCircleToRect(DetectedNearBall[resCounter]));
		}
	}
	else
	{
		for (size_t iCan = 0; iCan < CandidateBall.size(); iCan++) // iterate through each contour.
		{

			std::vector<cv::Rect> objects;
			cv::Rect rec = MyCircleToRect(CandidateBall[iCan]);

			int xL = max_n(0, (int )(rec.x - rec.width * ratioW));
			int xR = min_n(params.camera.width->get(),
					(int )(rec.x + rec.width + rec.width * ratioW));

			int yT = max_n(0, (int )(rec.y - rec.height * ratioHT));
			int yB = min_n(params.camera.height->get(),
					(int )(rec.y + rec.height + rec.height * ratioHB));

			rec.x = xL;
			rec.y = yT;
			rec.width = xR - xL;
			rec.height = yB - yT;

			cv::Mat imgGray = rawGray(rec);
			/*			cv::imshow("REC", imgGray);
			 cv::waitKey(100);*/

			int minSizeDim = max(3,
					(int) (max(rec.size().height, rec.size().width) / 14.));
			int minNeighborsDim = max(5,
					(int) (min(rec.size().height, rec.size().width) / 17.));
			object_cascade.detectMultiScale(imgGray, objects, 1.1,
					minNeighborsDim, 0, cv::Size(minSizeDim, minSizeDim),
					rec.size());

			for (size_t i = 0; i < objects.size(); i++)
			{
				objects[i].x += rec.x;
				objects[i].y += rec.y;
			}
			std::sort(objects.begin(), objects.end(), sorter(&projection));

			for (size_t iObj = 0; iObj < objects.size(); iObj++)
			{

				cv::Rect resRect(objects[iObj].x, objects[iObj].y,
						objects[iObj].width, objects[iObj].height);
				cv::Point2d centerObj = GetCenter(resRect);
				if (pointPolygonTest(hullField, centerObj, false) < 0)
				{
					continue;
				}

				if (!checkDistance_Rec(resRect, projection))
					continue;

				if (!checkHistogram(rawHSV, fieldBinaryRaw, centerObj,
						objects[iObj].width, params.ball.histogramH_C->get(),
						params.ball.histogramS_C->get(),
						params.ball.histogramV_C->get()))
					continue;

				if (cv::countNonZero(ballMask(resRect))
						< (resRect.width * resRect.height) * 0.2)
					continue;

				if (GetDistance(centerObj, CandidateBall[iCan].Center)
						> resRect.width * 0.5)
					continue;

				resGetNearestContour.push_back(resRect);
				break; //to just find one ball in each ROI
			}
		}
	}
	if (SHOWGUI && params.debug.showBallD->get())
	{

		cv::Mat darker = cv::Mat::zeros(guiImg.size(), CV_8UC3);
		darker.copyTo(guiImg, 255 - ballMask);
		circle(guiImg,
				cv::Point2d((params.camera.width->get() / 2),
						(params.camera.height->get())), distanceToCascade,
				whiteColor(), 1, 8, 0);
	}

	lastBallDetection = resGetNearestContour;
	return resGetNearestContour;
}

bool BallDetector::checkHistogram(const Mat &rawHSV, const Mat &fieldBinaryRaw,
		const vector<cv::Point> &con, double minHistogramDiffH,
		double minHistogramDiffS, double minHistogramDiffV)
{
	vector<cv::Point> conRoi;

	cv::Rect boundingRectangle = boundingRect(con);
	cv::Mat hsvRoi = rawHSV(boundingRectangle);
	cv::Mat notFieldMaskRoi = 255 - fieldBinaryRaw(boundingRectangle);
	cv::Mat paintedContour = cv::Mat::zeros(hsvRoi.size(), CV_8UC1);
	for (size_t i = 0; i < con.size(); i++)
	{
		conRoi.push_back(
				cv::Point(con[i].x - boundingRectangle.x,
						con[i].y - boundingRectangle.y));
	}
	vector<vector<cv::Point> > conRoiS = vector<vector<cv::Point> >(1, conRoi); //Nasty stuff just because of no implementation on draw one contour in opencv!
	drawContours(paintedContour, conRoiS, -1, grayWhite(),
	CV_FILLED, 8); // Draw the convexhull of the field

	cv::Mat justBallMask;
	paintedContour.copyTo(justBallMask, notFieldMaskRoi);

	cv::Mat forGuiShowOnlyBall;
	hsvRoi.copyTo(forGuiShowOnlyBall, justBallMask);
	//cv::imshow("hsvRoi", justBallMask);
	//cv::waitKey(100);
	//cv::imshow("forGuiShowOnlyBall", forGuiShowOnlyBall);
	//cv::waitKey(100);

	return checkHistogramInPic(hsvRoi, justBallMask, minHistogramDiffH,
			minHistogramDiffS, minHistogramDiffV);

}

bool BallDetector::checkHistogram(const Mat &rawHSV, const Mat &fieldBinaryRaw,
		cv::Point center, int radius, double minHistogramDiffH,
		double minHistogramDiffS, double minHistogramDiffV)
{
	cv::Rect recCircle(center.x - radius, center.y - radius, radius * 2,
			radius * 2);
	if (center.x - radius < 0 || center.y - radius < 0
			|| center.x + radius * 2 >= params.camera.width->get()
			|| center.y + radius * 2 >= params.camera.height->get())
	{
		//printf("Because the calculated radius is bigger than image \n");
		recCircle.x = max(0, center.x - radius);
		recCircle.y = max(0, center.y - radius);
		if (recCircle.x + radius * 2 > params.camera.width->get() - 1)
		{
			recCircle.width = params.camera.width->get() - 1 - recCircle.x;
		}
		else
		{
			recCircle.width = radius * 2;
		}
		if (recCircle.y + radius * 2 > params.camera.height->get() - 1)
		{
			recCircle.height = params.camera.height->get() - 1 - recCircle.y;
		}
		else
		{
			recCircle.height = radius * 2;
		}
		//	return false;
	}

	cv::Mat hsvRoi = rawHSV(recCircle);
	cv::Mat notFieldMaskRoi = 255 - fieldBinaryRaw(recCircle);
	cv::Mat paintedContour = cv::Mat::zeros(hsvRoi.size(), CV_8UC1);

	cv::circle(paintedContour,
			cv::Point(center.x - recCircle.x, center.y - recCircle.y), radius,
			grayWhite(), -1);

	cv::Mat justBallMask;
	paintedContour.copyTo(justBallMask, notFieldMaskRoi);

	//cv::Mat forGuiShowOnlyBall;
	//hsvRoi.copyTo(forGuiShowOnlyBall, justBallMask);
	//cv::imshow("hsvRoi", justBallMask);
	//cv::waitKey(100);
	//cv::imshow("forGuiShowOnlyBall", forGuiShowOnlyBall);
	//cv::waitKey(100);

	return checkHistogramInPic(hsvRoi, justBallMask, minHistogramDiffH,
			minHistogramDiffS, minHistogramDiffV);

}

bool BallDetector::checkHistogramInPic(cv::Mat &hsvRoi, cv::Mat &justBallMask,
		double minHistogramDiffH, double minHistogramDiffS,
		double minHistogramDiffV)
{
	cv::Mat hist_base;
	int channelss[] =
	{ 0, 1, 2 };
	float h_ranges[] =
	{ 0, 180 };
	float s_ranges[] =
	{ 0, 256 };
	float v_ranges[] =
	{ 0, 256 };
	/// Using  bins
	int h_bins = 32;
	int s_bins = 32;
	int v_bins = 32;
	const float* ranges[] =
	{ h_ranges, s_ranges, v_ranges };
	int histSize[] =
	{ h_bins, s_bins, v_bins };

	cv::calcHist(&hsvRoi, 1, channelss, justBallMask, hist_base, 2, histSize,
			ranges, true, false);
	cv::normalize(hist_base.clone(), hist_base, 0, 1, cv::NORM_MINMAX, -1,
			cv::Mat());

	if (!hist_base.empty())
	{

		for (std::vector<cv::Mat>::size_type i = 0; i < hist_list.size(); i++)
		{
			cv::Mat hist_base_channels[3], hist_list_channels[3];
			split(hist_base, hist_base_channels);
			split(hist_list[i], hist_list_channels);

			double tmpDiffH = compareHist(hist_base_channels[0],
					hist_list_channels[0], 3);
			double tmpDiffS = compareHist(hist_base_channels[0],
					hist_list_channels[0], 3);
			double tmpDiffV = compareHist(hist_base_channels[0],
					hist_list_channels[0], 3);

			if (tmpDiffH < minHistogramDiffH && tmpDiffS < minHistogramDiffS
					&& tmpDiffV < minHistogramDiffV)
			{
				return true;
			}
		}
		return false;
	}
	ROS_ERROR("Error in programming");
	return false;
}

bool BallDetector::checkDistance_Rec(const cv::Rect &rec,
		CameraProjections &projection)
{
	LineSegment lowerBound(
			Point2f(params.ball.NearestDistance->get(),
					params.ball.NearMinLen->get()),
			Point2f(params.ball.FarestDistance->get(),
					params.ball.FarMinLen->get()));
	LineSegment higherBound(
			Point2f(params.ball.NearestDistance->get(),
					params.ball.NearMaxLen->get()),
			Point2f(params.ball.FarestDistance->get(),
					params.ball.FarMaxLen->get()));
	LinearBoundaryChecker checker(lowerBound, higherBound);
	double minlen = min(rec.height, rec.width);
	double maxlen = max(rec.height, rec.width);

	cv::Point center(rec.x + rec.width / 2, rec.y + rec.height / 2);
	cv::Point2f realWorldCenter;
	if (!projection.GetOnRealCordinate(center, realWorldCenter))
	{
		ROS_ERROR("Error in programming");
		return false;
	}

	double distanceToRobot = GetDistance(realWorldCenter);

	if (!checker.Check(distanceToRobot, minlen)
			|| !checker.Check(distanceToRobot, maxlen))
	{
		return false;
	}

	return true;
}

bool BallDetector::checkContourRatio(const cv::vector<cv::Point> &con)
{
	cv::RotatedRect rec = cv::minAreaRect(con);

	double minlen = min(rec.size.height, rec.size.width);
	double maxlen = max(rec.size.height, rec.size.width);
	double ratio = maxlen / minlen;

	if (ratio > params.ball.maxRatio->get()
			|| ratio < params.ball.minRatio->get())
	{
		return false;
	}
	return true;
}

bool BallDetector::Init()
{
	return readFromFile<Mat>("ballColorHist.yml", hist_list)
			&& object_cascade.load(
					"/nimbro/share/launch/config/vision/cascadeBall.xml");
}

