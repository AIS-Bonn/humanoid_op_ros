//GuiManager.cpp
// Created on: May 7, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <vision_module/Tools/GuiManager.hpp>

#define SAFESETFALSE(x) 			\
{									\
	if(x())							\
	{								\
		x.set(false);				\
	}								\
}

#define SAFESETTRUE(x) 			    \
{									\
	if(!x())						\
	{								\
		x.set(true);				\
	}								\
}

#define CHECKPUTTXT(x) 			    	\
{										\
	if(params.gui->x())					\
	{									\
		putText(guiRawImg, #x, Point(55,55), CV_FONT_NORMAL, 1,blueMeloColor(), 1, 1);	\
	}									\
}

void GuiManager::Update(const Mat &rawHSV, const Mat &img)
{
	if (params.gui->calibDistSize_Ball() && distanceSizeSample.isValid()
			&& params.ball->addSizeDistance())
	{

		Point2f realPos = projection->convertToBallProjection(
				distanceSizeSample.realLocation);
		double len = distanceSizeSample.length1;
		double len2 = distanceSizeSample.length2;
		Point3d tmp = Point3d(min_n(len, len2), max_n(len, len2),
				GetDistance(realPos));
		params.ball->distSizeVec.push_back(tmp);
		bool res = add2File<Point3d>(params.ball->distSizeVecPath, tmp);

		distanceSizeSample.reset();
		std::sort(params.ball->distSizeVec.begin(),
				params.ball->distSizeVec.end(),
				[](const Point3d& a, const Point3d& b)
				{
					return a.z < b.z;
				});
		writeGuiConsoleFormat(greenColor(),
				"SizeDistance is %s for ball! count=%d \r\n",
				(res ? "successfully saved" : "failed to save"),
				(int) params.ball->distSizeVec.size());
	}

	if (params.gui->calibDistSize_Goal() && distanceSizeSample.isValid()
			&& params.goal->addSizeDistance())
	{

		Point2f realPos = distanceSizeSample.realLocation;
		double len = distanceSizeSample.length1;
		double len2 = distanceSizeSample.length2;
		Point3d tmp = Point3d(min_n(len, len2), max_n(len, len2),
				GetDistance(realPos));

		params.goal->distSizeVec.push_back(tmp);
		bool res = add2File<Point3d>(params.goal->distSizeVecPath, tmp);
		distanceSizeSample.reset();
		std::sort(params.goal->distSizeVec.begin(),
				params.goal->distSizeVec.end(),
				[](const Point3d& a, const Point3d& b)
				{
					return a.z < b.z;
				});
		writeGuiConsoleFormat(greenColor(),
				"SizeDistance is %s for goal! count=%d \r\n",
				(res ? "successfully saved" : "failed to save"),
				(int) params.goal->distSizeVec.size());
	}

	if (params.gui->saveHistogram() && histogramContour.size() >= 3
			&& params.ball->addHistogram())
	{

		cv::Mat paintedContour = cv::Mat::zeros(rawHSV.size(),
		CV_8UC1);
		vector<vector<cv::Point> > resBallCs = vector<vector<cv::Point> >(1,
				histogramContour);
		cv::drawContours(paintedContour, resBallCs, -1, cv::Scalar(255),
		CV_FILLED, 8);

		cv::Mat t[3], tp;
		cv::Mat channels[3];

		split(rawHSV, channels);
		bitwise_and(channels[0], paintedContour, t[0]);
		bitwise_and(channels[1], paintedContour, t[1]);
		bitwise_and(channels[2], paintedContour, t[2]);
		cv::merge(t, 3, tp);
		split(tp, channels);
		cv::Mat mask = paintedContour.clone();

		cv::merge(t, 3, tp);
		cv::Mat roi(tp, boundingRect(histogramContour));
		cv::Mat roimask(mask, boundingRect(histogramContour));

		cv::Mat hist_base[3];
		if (calcHist3Channels(roi, roimask, hist_base))
		{
			double minHistDiff[3];
			for (int cnlIdx = 0; cnlIdx < 3; cnlIdx++) //HSV channels
			{
				minHistDiff[cnlIdx] = 9999; // Just a big number
				if (!hist_base[cnlIdx].empty())
				{
					for (std::vector<cv::Mat>::size_type i = 0;
							i < params.ball->histList[cnlIdx].size(); i++)
					{
						double tmpDiff = compareHist(hist_base[cnlIdx],
								params.ball->histList[cnlIdx][i], 3);
						if (tmpDiff < minHistDiff[cnlIdx])
						{
							minHistDiff[cnlIdx] = tmpDiff;
						}
					}
				}
				else
				{
					writeGuiConsoleFormat(redColor(),
							"Error in programming \r\n");
				}

			}

			if (minHistDiff[0] < 0.01 || minHistDiff[1] < 0.01
					|| minHistDiff[2] < 0.01)
			{
				writeGuiConsoleFormat(redColor(), "Duplicated histogram \r\n");
			}
			else
			{
				for (int cnlIdx = 0; cnlIdx < 3; cnlIdx++) //HSV channels
				{
					writeGuiConsoleFormat(whiteColor(), "minHistDiff[%d] = %f",
							cnlIdx, minHistDiff[cnlIdx]);

					params.ball->histList[cnlIdx].push_back(hist_base[cnlIdx]);
					add2File<cv::Mat>(params.ball->histListcPath[cnlIdx],
							hist_base[cnlIdx]);
				}
			}

		}

		writeGuiConsoleFormat(greenColor(),
				"Histogram is saved for ball! Count =%d \r\n",
				(int) params.ball->histList[0].size()); //Due to the fact that all channels fills the same time, there is no difference between cnlIdxs

		histogramContour.clear();

	}

	if (params.gui->saveSample() && rectSaver.isFinished()
			&& (params.ball->savePositive() || params.ball->saveNegative()))
	{

		Rect rec = rectSaver.getRect();
		bool saveSuccess = true;
		string path = params.configPath + "samples/"
				+ (params.ball->savePositive() ? "posBall/" : "negBall/");

		if (boost::filesystem::create_directories(
				boost::filesystem::path(path)))
		{
			writeGuiConsoleFormat(greenColor(), "Folder created in: %s",
					path.c_str());
		}
		int rectCounter = 0;
		int maxRotation =
				params.ball->savePositive() ?
						params.ball->rectRotationMax() : 0;
		int stepRotation =
				params.ball->savePositive() ?
						params.ball->rectRotationStep() : 1;
		for (int i = -maxRotation; i <= maxRotation; i += stepRotation)
		{
			// rect is the RotatedRect (I got it from a contour...)
			RotatedRect rectR;
			rectR.center = Point(rec.x + rec.width / 2, rec.y + rec.height / 2);
			rectR.size = Size(rec.width, rec.height);
			rectR.angle = i;
			// matrices we'll use
			Mat M, rotated, cropped;
			// get angle and size from the bounding box
			float angle = rectR.angle;
			Size rect_size = rectR.size;
			// thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
			if (rectR.angle < -45.)
			{
				angle += 90.0;
				swap(rect_size.width, rect_size.height);
			}
			// get the rotation matrix
			M = getRotationMatrix2D(rectR.center, angle, 1.0);
			// perform the affine transformation
			warpAffine(img, rotated, M, img.size(), INTER_CUBIC);
			// crop the resulting image
			getRectSubPix(rotated, rect_size, rectR.center, cropped);

			stringstream tmpStr;
			tmpStr << ros::WallTime::now().sec << "_" << i << "_";

			if (params.ball->savePositive()
					&& params.ball->rectResizeWidth() > 0
					&& params.ball->rectResizeHeight() > 0)
			{
				resize(cropped, cropped,
						Size(params.ball->rectResizeWidth(),
								params.ball->rectResizeHeight())); //resize image
			}

			saveSuccess &= imwrite(path + tmpStr.str() + ".jpg", cropped);
			rectCounter++;
			flip(cropped, cropped, 1);
			saveSuccess &= imwrite(path + tmpStr.str() + "R.jpg", cropped);
			rectCounter++;
		}
		writeGuiConsoleFormat(saveSuccess ? greenColor() : redColor(),
				"%d %s ball rectangle %s", rectCounter,
				(params.ball->savePositive() ? "Positive" : "Negative"),
				saveSuccess ? "saved successfully" : "could not save.");
		rectSaver.reset();

	}

	if (params.ball->saveNegativeFull())
	{
		bool saveSuccess = true;
		string path = params.configPath + "samples/negBall/";

		if (boost::filesystem::create_directories(
				boost::filesystem::path(path)))
		{
			writeGuiConsoleFormat(greenColor(), "Folder created in: %s",
					path.c_str());
		}

		int rectCounter = 0;

		stringstream tmpStr;
		tmpStr << ros::WallTime::now().sec;

		saveSuccess &= imwrite(path + tmpStr.str() + ".jpg", img);
		rectCounter++;
		Mat imgFilip;
		flip(img, imgFilip, 1);
		saveSuccess &= imwrite(path + tmpStr.str() + "R.jpg", imgFilip);
		rectCounter++;

		writeGuiConsoleFormat(saveSuccess ? greenColor() : redColor(),
				"%d Full negative ball image %s", rectCounter,
				saveSuccess ? "saved successfully" : "could not save.");
		rectSaver.reset();

	}

	if (params.ball->clearDistSizeVec())
	{
		params.ball->distSizeVec.clear();
		write2File<Point3d>(params.ball->distSizeVecPath,
				params.ball->distSizeVec);
		writeGuiConsoleFormat(greenColor(),
				"SizeDistance is cleared for ball!");
	}
	if (params.ball->popSizeDistance())
	{
		readFromFile<Point3d>(params.ball->distSizeVecPath,
				params.ball->distSizeVec); //To read it again from file it might be sorted.

		if (params.ball->distSizeVec.size())
		{
			params.ball->distSizeVec.pop_back();
			write2File<Point3d>(params.ball->distSizeVecPath,
					params.ball->distSizeVec);
			std::sort(params.ball->distSizeVec.begin(),
					params.ball->distSizeVec.end(),
					[](const Point3d& a, const Point3d& b)
					{
						return a.z < b.z;
					});
		}
		writeGuiConsoleFormat(greenColor(),
				"SizeDistance is popped for ball! count=%d",
				(int) params.ball->distSizeVec.size());
	}

	if (params.goal->clearDistSizeVec())
	{
		params.goal->distSizeVec.clear();
		write2File<Point3d>(params.goal->distSizeVecPath,
				params.goal->distSizeVec);
		writeGuiConsoleFormat(greenColor(),
				"SizeDistance is cleared for goal!");
	}
	if (params.goal->popSizeDistance())
	{
		readFromFile<Point3d>(params.goal->distSizeVecPath,
				params.goal->distSizeVec); //To read it again from file it might be sorted.

		if (params.goal->distSizeVec.size() > 0)
		{
			params.goal->distSizeVec.pop_back();
			write2File<Point3d>(params.goal->distSizeVecPath,
					params.goal->distSizeVec);
			std::sort(params.goal->distSizeVec.begin(),
					params.goal->distSizeVec.end(),
					[](const Point3d& a, const Point3d& b)
					{
						return a.z < b.z;
					});
		}

		writeGuiConsoleFormat(greenColor(),
				"SizeDistance is popped for goal! count=%d",
				(int) params.goal->distSizeVec.size());
	}

	if (params.ball->clearHistogram())
	{
		for (int chIdx = 0; chIdx < 3; chIdx++)
		{
			params.ball->histList[chIdx].clear();
			write2File<Mat>(params.ball->histListcPath[chIdx],
					params.ball->histList[chIdx]);
		}

		writeGuiConsoleFormat(greenColor(), "Histogram is cleared for ball!");
	}

	if (params.ball->popHistogram())
	{
		for (int chIdx = 0; chIdx < 3; chIdx++)
		{
			readFromFile<Mat>(params.ball->histListcPath[chIdx],
					params.ball->histList[chIdx]); //To read it again from file it might be sorted.
			if (params.ball->histList[chIdx].size() > 0)
			{
				params.ball->histList[chIdx].pop_back();
				write2File<Mat>(params.ball->histListcPath[chIdx],
						params.ball->histList[chIdx]);
			}
			writeGuiConsoleFormat(greenColor(),
					"histList[%d] is popped for ball! count=%d ", chIdx,
					(int) params.ball->histList[chIdx].size());
		}
	}

	SAFESETFALSE(params.ball->popHistogram);
	SAFESETFALSE(params.ball->popHistogram);
	SAFESETFALSE(params.ball->clearHistogram);
	SAFESETFALSE(params.ball->addSizeDistance);
	SAFESETFALSE(params.ball->popSizeDistance);
	SAFESETFALSE(params.ball->clearDistSizeVec);
	SAFESETFALSE(params.ball->savePositive);
	SAFESETFALSE(params.ball->saveNegative);
	SAFESETFALSE(params.ball->addHistogram);
	SAFESETFALSE(params.ball->saveNegativeFull);
	SAFESETFALSE(params.goal->addSizeDistance);
	SAFESETFALSE(params.goal->popSizeDistance);
	SAFESETFALSE(params.goal->clearDistSizeVec);
}

void GuiManager::Publish(const Mat &gray, Mat &guiRawImg, bool shouldPublish)
{

	if (!shouldPublish)
		return;

	CHECKPUTTXT(calibDistSize_Goal);
	CHECKPUTTXT(calibDistSize_Ball);
	CHECKPUTTXT(showDistSize_Goal);
	CHECKPUTTXT(showDistSize_Ball);
	CHECKPUTTXT(saveSample);
	CHECKPUTTXT(saveHistogram);
	CHECKPUTTXT(debugCascade);
	CHECKPUTTXT(tfCalib);

	if (params.gui->debugCascade())
	{
		HAF_WARN_THROTTLE(1, "debugCascade is on!");
		std::vector<Rect> cascade_recs;
		object_cascade.detectMultiScale(gray, cascade_recs,
				params.ball->cascadeScale(), params.ball->cascadeNinNeighbors(),
				0 | CASCADE_SCALE_IMAGE, Size(10, 10), Size(320, 320));
		for (size_t i = 0; i < cascade_recs.size(); i++)
		{
			rectangle(guiRawImg, cascade_recs[i], pinkMeloColor(), 2);
		}
	}
	else if (params.gui->saveSample() && rectSaver.isValid())
	{
		rectangle(guiRawImg, rectSaver.getRect(), redColor(), 2);
	}
	else if (params.gui->saveHistogram())
	{
		for (size_t i = 0; i < histogramContour.size(); i++)
		{
			circle(guiRawImg, Point(histogramContour[i]), 2, redColor(), 1);
		}

		if (histogramContour.size() >= 3)
		{
			vector<vector<Point> > contours = vector<vector<Point> >(1,
					histogramContour); //Nasty stuff just because of no implementation on draw one contour in opencv!
			drawContours(guiRawImg, contours, 0, Scalar(0, 255, 0), 1);
		}

	}
	else if (params.gui->calibDistSize_Ball() && distanceSizeSample.isValid())
	{
		cv::circle(guiRawImg, distanceSizeSample.imgLocation,
				max_n(distanceSizeSample.length1, distanceSizeSample.length2),
				redColor(), 1);
		cv::circle(guiRawImg, distanceSizeSample.imgLocation,
				min_n(distanceSizeSample.length1, distanceSizeSample.length2),
				greenColor(), 1);

	}
	else if (params.gui->calibDistSize_Goal() && distanceSizeSample.isValid())
	{
		LineSegment line1(distanceSizeSample.imgLocation, 0,
				distanceSizeSample.length1);
		LineSegment line2(distanceSizeSample.imgLocation, 0,
				distanceSizeSample.length2);

		bool l1IsLonger = distanceSizeSample.length1
				> distanceSizeSample.length2;

		cv::line(guiRawImg, (l1IsLonger ? line1 : line2).P1,
				(l1IsLonger ? line1 : line2).P2, redColor(), 1);

		cv::line(guiRawImg, (l1IsLonger ? line2 : line1).P1,
				(l1IsLonger ? line2 : line1).P2, greenColor(), 2);
	}
	else if (params.gui->showDistSize_Ball())
	{
		Point2f realPos;
		if (params.ball->distSizeVec.size() >= 2
				&& projection->GetOnRealCordinate_single(mouseLastMovePos,
						realPos))
		{

			realPos = projection->convertToBallProjection(realPos);
			double distanceToRobot = GetDistance(realPos);
			if (distanceToRobot < MAXCONSIDERDISTANCE)
			{
				size_t start = 0;
				size_t end = params.ball->distSizeVec.size() - 1;
				bool rangeFound = false;
				for (size_t i = 0; i < params.ball->distSizeVec.size(); i++)
				{
					if (params.ball->distSizeVec[i].z <= distanceToRobot
							&& i + 1 < params.ball->distSizeVec.size()
							&& params.ball->distSizeVec[i + 1].z
									> distanceToRobot)
					{
						start = i;
						end = i + 1;
						rangeFound = true;
						break;
					}
				}

				if (!rangeFound)
				{
					if (distanceToRobot < params.ball->distSizeVec[0].z)
					{
						start = 0;
						end = 1;
						rangeFound = true;
					}
					else if (distanceToRobot
							> params.ball->distSizeVec[params.ball->distSizeVec.size()
									- 1].z)
					{
						start = params.ball->distSizeVec.size() - 2;
						end = params.ball->distSizeVec.size() - 1;
						rangeFound = true;
					}
					else
					{
						ROS_ERROR("Programming Error!");
					}
				}

				double NearMinLen = params.ball->distSizeVec[start].x;
				double NearMaxLen = params.ball->distSizeVec[start].y;
				double NearestDistance = params.ball->distSizeVec[start].z;

				double FarMinLen = params.ball->distSizeVec[end].x;
				double FarMaxLen = params.ball->distSizeVec[end].y;
				double FarestDistance = params.ball->distSizeVec[end].z;

				LineSegment lowerBound(Point2f(NearestDistance, NearMinLen),
						Point2f(FarestDistance, FarMinLen));
				LineSegment higherBound(Point2f(NearestDistance, NearMaxLen),
						Point2f(FarestDistance, FarMaxLen));
				LinearBoundaryChecker checker(lowerBound, higherBound);
				double minRadius = 0;
				double maxRadius = 0;
				if (checker.GetExtrapolation(distanceToRobot, minRadius,
						maxRadius))
				{
					if (maxRadius >= 1)
					{
						cv::circle(guiRawImg, mouseLastMovePos, maxRadius,
								redColor(), 1);
					}
					if (minRadius >= 1)
					{
						cv::circle(guiRawImg, mouseLastMovePos, minRadius,
								greenColor(), 1);
					}
				}
			}
		}
	}
	else if (params.gui->showDistSize_Goal())
	{
		Point2f realPos;
		if (params.goal->distSizeVec.size() >= 2
				&& projection->GetOnRealCordinate_single(mouseLastMovePos,
						realPos))
		{
			double distanceToRobot = GetDistance(realPos);
			if (distanceToRobot < MAXCONSIDERDISTANCE)
			{
				size_t start = 0;
				size_t end = params.goal->distSizeVec.size() - 1;
				bool rangeFound = false;
				for (size_t i = 0; i < params.goal->distSizeVec.size(); i++)
				{
					if (params.goal->distSizeVec[i].z <= distanceToRobot
							&& i + 1 < params.goal->distSizeVec.size()
							&& params.goal->distSizeVec[i + 1].z
									> distanceToRobot)
					{
						start = i;
						end = i + 1;
						rangeFound = true;
						break;
					}
				}

				if (!rangeFound)
				{
					if (distanceToRobot < params.goal->distSizeVec[0].z)
					{
						start = 0;
						end = 1;
						rangeFound = true;
					}
					else if (distanceToRobot
							> params.goal->distSizeVec[params.goal->distSizeVec.size()
									- 1].z)
					{
						start = params.goal->distSizeVec.size() - 2;
						end = params.goal->distSizeVec.size() - 1;
						rangeFound = true;
					}
					else
					{
						ROS_ERROR("Programming Error!");
					}
				}

				double NearMinLen = params.goal->distSizeVec[start].x;
				double NearMaxLen = params.goal->distSizeVec[start].y;
				double NearestDistance = params.goal->distSizeVec[start].z;

				double FarMinLen = params.goal->distSizeVec[end].x;
				double FarMaxLen = params.goal->distSizeVec[end].y;
				double FarestDistance = params.goal->distSizeVec[end].z;

				LineSegment lowerBound(Point2f(NearestDistance, NearMinLen),
						Point2f(FarestDistance, FarMinLen));
				LineSegment higherBound(Point2f(NearestDistance, NearMaxLen),
						Point2f(FarestDistance, FarMaxLen));
				LinearBoundaryChecker checker(lowerBound, higherBound);
				double minRadius = 0;
				double maxRadius = 0;
				if (checker.GetExtrapolation(distanceToRobot, minRadius,
						maxRadius))
				{
					if (maxRadius >= 1)
					{
						LineSegment lineMax(mouseLastMovePos, 0, maxRadius);
						cv::line(guiRawImg, lineMax.P1, lineMax.P2, redColor(),
								1);
					}
					if (minRadius >= 1)
					{
						LineSegment lineMin(mouseLastMovePos, 0, minRadius);
						cv::line(guiRawImg, lineMin.P1, lineMin.P2,
								greenColor(), 2);
					}
				}
			}
		}
	}
}

void GuiManager::rviz_click_callback(
		const geometry_msgs::PointStampedConstPtr& msg)
{
	if (msg->point.z > 0.3 || msg->point.z < -0.3)
	{
		writeGuiConsoleFormat(yellowColor(),
				"The clicked point in the rviz is not on the zero plane!");
	}

	writeGuiConsoleFormat(whiteColor(), "I heard: [%f , %f, %f] for %s ",
			msg->point.x, msg->point.y, msg->point.z,
			(params.gui->rvizSetLoc() ? "set location" : "tf calibration"));

	if (params.gui->rvizSetRobotTFOrigin())
	{
		params.tfP->robotX.set(msg->point.x);
		params.tfP->robotY.set(msg->point.y);
	}
	else if (params.gui->rvizSetLoc())
	{
		params.locClicked.x = msg->point.x;
		params.locClicked.y = msg->point.y;
		params.locIsClicked = true;
	}
	else if (params.gui->rvizSetCalibTF())
	{
		if (params.camCalibrator.clicked.size()
				> params.camCalibrator.rvizClicked.size())
		{
			params.camCalibrator.rvizClicked.push_back(
					Point2d(msg->point.x, msg->point.y));
			params.camCalibrator.cameraLocation.push_back(
					projection->cameraLocation);
			params.camCalibrator.opticalAngle.push_back(
					projection->opticalAngle);

			cout << "-->  cameraLocation = " << projection->cameraLocation;
			cout << "  opticalAngle = " << projection->opticalAngle << endl;
		}
		else
		{
			if (params.camCalibrator.rvizClicked.size() > 0)
			{
				params.camCalibrator.rvizClicked.pop_back();
				params.camCalibrator.cameraLocation.pop_back();
				params.camCalibrator.opticalAngle.pop_back();
			}
			params.camCalibrator.rvizClicked.push_back(
					Point2d(msg->point.x, msg->point.y));
			params.camCalibrator.cameraLocation.push_back(
					projection->cameraLocation);
			params.camCalibrator.opticalAngle.push_back(
					projection->opticalAngle);
		}
	}

}

void GuiManager::gui_events_callback(
		const rqt_vision_module::GuiEvent::ConstPtr & msg)
{
	if (msg->topic != "/vision/guiImg" && msg->topic != "/vision/webImg")
		return;
	Point pos;
	pos.x = msg->mouse_x * params.camera->width();
	pos.y = msg->mouse_y * params.camera->height();

	if (msg->mouse_event == (rqt_vision_module::Move))
	{
		mouseLastMovePos = pos;
	}
	if (params.gui->debugBallCandidates())
	{
		if (msg->mouse_event == (rqt_vision_module::LeftRelease))
		{
			params.ball->debugThisPos = pos;
			params.ball->debugThis = true;
		}
	}
	else if (params.gui->saveSample())
	{
		if (msg->mouse_event == (rqt_vision_module::MiddleRelease))
		{

			rectSaver.reset();

		}
		else if (msg->mouse_event == (rqt_vision_module::LeftClick))
		{
			rectSaver.reset();
			rectSaver.p1 = pos;
			rectSaver.hold = true;
		}
		else if (msg->mouse_event == (rqt_vision_module::LeftRelease))
		{
			rectSaver.p2 = pos;
			rectSaver.hold = false;
		}
		else if (msg->mouse_event == (rqt_vision_module::RightRelease))
		{
			if (!msg->ctrl_pressed && msg->shift_pressed && !msg->alt_pressed)
			{
				SAFESETTRUE(params.ball->savePositive);
			}
			else if (msg->ctrl_pressed && !msg->shift_pressed
					&& !msg->alt_pressed)
			{
				SAFESETTRUE(params.ball->saveNegative);
			}
			else if (msg->ctrl_pressed && !msg->shift_pressed
					&& msg->alt_pressed)
			{
				SAFESETTRUE(params.ball->saveNegativeFull);
			}
		}
		else if (msg->mouse_event == (rqt_vision_module::Move))
		{
			if (rectSaver.hold)
			{
				rectSaver.p2 = pos;
			}
		}
	}
	else if (params.gui->saveHistogram())
	{
		if (msg->mouse_event == (rqt_vision_module::MiddleRelease))
		{
			histogramContour.clear();
		}
		else if (msg->mouse_event == (rqt_vision_module::RightRelease))
		{
			if (msg->ctrl_pressed && msg->shift_pressed)
			{
				SAFESETTRUE(params.ball->addHistogram);
			}
			else
			{
				if (!histogramContour.empty())
				{
					histogramContour.pop_back();
				}
			}
		}
		else if (msg->mouse_event == (rqt_vision_module::LeftRelease))
		{
			histogramContour.push_back(pos);
		}
	}
	else if (params.gui->calibDistSize_Ball()
			|| params.gui->calibDistSize_Goal())
	{
		if (msg->mouse_event == (rqt_vision_module::MiddleRelease))
		{
			distanceSizeSample.reset();
		}
		else if (msg->mouse_event == (rqt_vision_module::LeftRelease))
		{
			distanceSizeSample.imgLocation = pos;
			Point2f realClicked;
			if (projection->GetOnRealCordinate_single(
					distanceSizeSample.imgLocation, realClicked))
			{
				distanceSizeSample.realLocation = realClicked;
			}
		}
		else if (msg->mouse_event
				== (rqt_vision_module::MiddleRotatedBackwards))
		{
			int step = 1;
			if (msg->alt_pressed)
			{
				step = 2;
			}
			if (msg->ctrl_pressed)
			{
				distanceSizeSample.length1 += step;
			}
			else
			{
				distanceSizeSample.length2 += step;
			}
		}
		else if (msg->mouse_event == (rqt_vision_module::MiddleRotatedForward))
		{
			int step = 1;
			if (msg->alt_pressed)
			{
				step = 2;
			}
			if (msg->ctrl_pressed)
			{
				distanceSizeSample.length1 -= step;
			}
			else
			{
				distanceSizeSample.length2 -= step;
			}
			if (distanceSizeSample.length1 < 0)
			{
				distanceSizeSample.length1 = 0;
			}
			if (distanceSizeSample.length2 < 0)
			{
				distanceSizeSample.length2 = 0;
			}
		}
		else if (msg->mouse_event == (rqt_vision_module::RightRelease))
		{
			if (msg->ctrl_pressed && msg->shift_pressed)
			{
				if (params.gui->calibDistSize_Ball())
				{
					SAFESETTRUE(params.ball->addSizeDistance);
				}
				else if (params.gui->calibDistSize_Goal())
				{
					SAFESETTRUE(params.goal->addSizeDistance);
				}
			}
		}
	}
	else if (params.gui->tfCalib())
	{
		if (msg->mouse_event == (rqt_vision_module::LeftRelease))
		{
			if (msg->ctrl_pressed && msg->shift_pressed)
			{
				SAFESETTRUE(params.tfP->calibrateKinematicSimplex);
			}
			else
			{
				writeGuiConsoleFormat(whiteColor(),
						"Projection click @ [%d,%d] ", pos.x, pos.y);
				if (params.camCalibrator.clicked.size()
						<= params.camCalibrator.rvizClicked.size())
				{
					params.camCalibrator.clicked.push_back(pos);
				}
				else
				{
					params.camCalibrator.clicked.pop_back();
					params.camCalibrator.clicked.push_back(pos);
				}
			}
		}
		else if (msg->mouse_event == (rqt_vision_module::RightRelease))
		{

			if (msg->ctrl_pressed && msg->shift_pressed)
			{
				SAFESETTRUE(params.tfP->calibrateKinematicHill);
			}
			else
			{
				if (params.camCalibrator.clicked.size() > 0)
				{
					params.camCalibrator.clicked.pop_back();
				}
				if (params.camCalibrator.clicked.size() > 0
						&& params.camCalibrator.clicked.size()
								< params.camCalibrator.rvizClicked.size())
				{
					params.camCalibrator.rvizClicked.pop_back();
					params.camCalibrator.cameraLocation.pop_back();
					params.camCalibrator.opticalAngle.pop_back();
				}
			}
		}
		else if (msg->mouse_event == (rqt_vision_module::MiddleRelease))
		{
			writeGuiConsoleFormat(greenColor(), "Clear All Points");
			params.camCalibrator.rvizClicked.clear();
			params.camCalibrator.cameraLocation.clear();
			params.camCalibrator.opticalAngle.clear();
			params.camCalibrator.clicked.clear();
		}
	}
	else if (params.gui->printUnderMouseInfo())
	{
		if (msg->ctrl_pressed)
		{
			Point2f inPoint, resPoint;
			projection->distorionModel.UndistortP(pos, resPoint);
			inPoint = resPoint;
			if (projection->GetOnRealCordinate_FromUndistorted_Single(inPoint,
					resPoint))
			{
				Point2d unroatedCameraYaw = projection->unroateCameraYaw(
						resPoint);
				writeGuiConsoleFormat(whiteColor(),
						" GetOnRealCordinate_FromUndistorted_Single  = [%.2f, %.2f] -> %.2f",
						unroatedCameraYaw.x, unroatedCameraYaw.y,
						GetDistance(unroatedCameraYaw));
			}
		}
		else if (msg->shift_pressed)
		{
			Point2f resPoint;

			if (projection->GetOnRealCordinate_single(pos, resPoint))
			{
				if (msg->alt_pressed)
				{
					resPoint = projection->convertToBallProjection(resPoint);
				}
				writeGuiConsoleFormat(whiteColor(),
						" %s  = [%.2f, %.2f] -> %.2f",
						msg->alt_pressed ?
								"convertToBallProjection" :
								"GetOnRealCordinate_single", resPoint.x,
						resPoint.y, GetDistance(resPoint));
			}
		}

	}
}
