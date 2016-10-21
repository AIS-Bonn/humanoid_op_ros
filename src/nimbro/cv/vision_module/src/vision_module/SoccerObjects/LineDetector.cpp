//LineDetector.cpp
// Created on: May 14, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <vision_module/SoccerObjects/LineDetector.hpp>

bool LineDetector::GetLines(Mat &rawHSV, Mat & fieldMask, Mat &guiImg,
		CameraProjections &projection, bool SHOWGUI, const Mat &lineBinary,
		Rect box, vector<LineSegment> &resLines)
{
	bool aprxDist = params.line->aprxDist();
	const int NUM_MID_P = 3;
	const int COUNT_MID_P = pow(2, NUM_MID_P) + 1;

	vector<Vec4i> linesFromHoughP;
	HoughLinesP(lineBinary, linesFromHoughP, params.line->rhoHough(),
	M_PI / params.line->thetaHough(), params.line->threasholdHough(),
			params.line->MinLineLength(),
			params.line->maxLineGapHough());
	if (linesFromHoughP.size() < 1)
		return false;
	resLines.reserve(linesFromHoughP.size());
	const int MIN_PIXEL = 2;
	double LINE_WIDTH_CHECK = params.line->widthCheck();
	int jumpMax = params.line->jumpMax();
	int jumpMin = params.line->jumpMin();

	vector<Point> queryPoint(linesFromHoughP.size() * COUNT_MID_P * 3);
	vector<Point2f> realQueryPoint(linesFromHoughP.size() * COUNT_MID_P * 3);

	vector<int> bothEndSize(linesFromHoughP.size() * COUNT_MID_P * 2);

	vector<LineSegment> PerpendicularLS(
			linesFromHoughP.size() * COUNT_MID_P * 3);

	vector<vector<cv::Point2d> > midds(linesFromHoughP.size(),
			vector<Point2d>(COUNT_MID_P));
	vector<LineSegment> allLines(linesFromHoughP.size());

	for (size_t i = 0; i < linesFromHoughP.size(); i++)
	{
		Vec4i lP = linesFromHoughP[i];
		allLines[i] = LineSegment(Point2d(lP[0], lP[1]), Point2d(lP[2], lP[3]));
		midds[i] = allLines[i].GetMidPoints(NUM_MID_P, false);
		for (size_t j = 0; j < COUNT_MID_P; j++)
		{
			int idx = i * COUNT_MID_P + j;
			PerpendicularLS[idx] = allLines[i].PerpendicularLineSegment(jumpMax,
					midds[i][j]);

			cv::LineIterator itUp(lineBinary, midds[i][j],
					PerpendicularLS[idx].P1, 8);
			cv::LineIterator itDown(lineBinary, midds[i][j],
					PerpendicularLS[idx].P2, 8);

			queryPoint[idx * 3] = midds[i][j];
			if (aprxDist)
			{
				for (int k = 0; k < itUp.count; k++, ++itUp)
				{
					Point p = itUp.pos();
					if (k <= MIN_PIXEL)
					{
						queryPoint[idx * 3 + 1] = p;
					}
					else
					{
						break;
					}
				}
				for (int k = 0; k < itDown.count; k++, ++itDown)
				{
					Point p = itDown.pos();
					if (k <= MIN_PIXEL)
					{
						queryPoint[idx * 3 + 2] = p;
					}
					else
					{
						break;
					}
				}
			}
			else
			{
				Point2f centerOnReal;
				if (!projection.GetOnRealCordinate_single(queryPoint[idx * 3],
						centerOnReal))
				{
					ROS_ERROR("Programming Error");
					return false;
				}
				for (int k = 0; k < itUp.count; k++, ++itUp)
				{
					Point p = itUp.pos();
					bothEndSize[idx * 2] = k;
					Point2f pR;
					if (!projection.GetOnRealCordinate_single(p, pR))
					{
						ROS_ERROR("Programming Error");
						return false;
					}
					if (GetDistance(centerOnReal, pR) > LINE_WIDTH_CHECK)
					{
						break;
					}
				}
				for (int k = 0; k < itDown.count; k++, ++itDown)
				{
					Point p = itDown.pos();
					bothEndSize[idx * 2 + 1] = k;

					Point2f pR;
					if (!projection.GetOnRealCordinate_single(p, pR))
					{
						ROS_ERROR("Programming Error");
						return false;
					}
					if (GetDistance(centerOnReal, pR) > LINE_WIDTH_CHECK)
					{
						break;
					}
				}
			}

		}
	}

	if (aprxDist)
	{

		if (!projection.GetOnRealCordinate(queryPoint, realQueryPoint))
		{
			ROS_ERROR("Programming Error");
			return false;
		}
	}

	uchar *dataImg = rawHSV.data;
	uchar *dataFieldImg = fieldMask.data;
	for (size_t i = 0; i < allLines.size(); i++)
	{
		double greenVoter = 0;
		int vote_for_double_up = 0;
		int vote_for_double_down = 0;
		int vote_for_color_up = 0;
		int vote_for_color_down = 0;
		int not_valid_point = 0;

		for (size_t j = 0; j < COUNT_MID_P; j++)
		{
			int idx = i * COUNT_MID_P + j;
			int cur_upArraySize = 0;
			int cur_downArraySize = 0;
			cv::LineIterator itUp(lineBinary, midds[i][j],
					PerpendicularLS[idx].P1, 8);
			cv::LineIterator itDown(lineBinary, midds[i][j],
					PerpendicularLS[idx].P2, 8);
			Point cenP = queryPoint[idx * 3];
			double upDistance = 0.0;
			double downDistance = 0.0;
			if (aprxDist)
			{
				Point upP = queryPoint[idx * 3 + 1];
				Point downP = queryPoint[idx * 3 + 2];

				upDistance = GetDistance(cenP, upP);
				downDistance = GetDistance(cenP, downP);

				Point2f cenPR = realQueryPoint[idx * 3];
				Point2f upPR = realQueryPoint[idx * 3 + 1];
				Point2f downPR = realQueryPoint[idx * 3 + 2];

				double upDistanceR = GetDistance(cenPR, upPR);
				double downDistanceR = GetDistance(cenPR, downPR);
				cur_upArraySize = min(itUp.count,
						(int) ((LINE_WIDTH_CHECK / upDistanceR) * upDistance));
				cur_downArraySize =
						min(itDown.count,
								(int) ((LINE_WIDTH_CHECK / downDistanceR)
										* downDistance));
			}
			else
			{
				cur_upArraySize = bothEndSize[idx * 2];
				cur_downArraySize = bothEndSize[idx * 2 + 1];
				upDistance = cur_upArraySize;
				downDistance = cur_downArraySize;
			}

			cur_upArraySize = max(jumpMin, cur_upArraySize);
			cur_downArraySize = max(jumpMin, cur_downArraySize);

			if (upDistance > 0.1 && downDistance > 0.1 && itUp.count >= jumpMin
					&& itDown.count >= jumpMin)
			{
				Point end_up, end_down;
				bool firstColor = params.line->colorVUse();
				bool firstGreen = params.line->greenVUse();
				bool firstDouble = params.line->doubleVUse();

				int startDouble = cur_upArraySize
						* params.line->doubleVStart();
				int endDouble = cur_upArraySize * params.line->doubleVEnd();
				int startGreen = cur_upArraySize
						* params.line->greenVStart();
				int endGreen = cur_upArraySize * params.line->greenVEnd();
				int startColor = cur_upArraySize
						* params.line->colorVStart();
				int endColor = cur_upArraySize * params.line->colorVEnd();

				for (int k = 0; k < cur_upArraySize; k++, ++itUp)
				{
					Point p = itUp.pos();
					end_up = p;
					if (k > 0)
					{
						int ypixel = p.y * params.camera->width();
						uchar* pixel = dataImg + (ypixel + p.x) * 3;
						uchar h = pixel[0];
						uchar s = pixel[1];
						uchar v = pixel[2];
						uchar* pixelF = dataFieldImg + ypixel + p.x;
						if (firstDouble && k >= startDouble && k <= endDouble)
						{
							if (*(*itUp) > 254)
							{
								vote_for_double_up++;
								firstDouble = false;
							}
						}
						if (firstColor && k >= startColor && k <= endColor)
						{
							if (h >= params.line->h0()
									&& h <= params.line->h1()
									&& s >= params.line->s0()
									&& s <= params.line->s1()
									&& v >= params.line->v0()
									&& v <= params.line->v1())
							{
								vote_for_color_up++;
								firstColor = false;
							}
						}
						if (firstGreen && k >= startGreen && k <= endGreen)
						{
							if (*pixelF > 254)
							{
								greenVoter += 0.5;
								firstGreen = false;
							}
						}
					}
					if (!params.line->showVote() && !firstColor
							&& !firstGreen && !firstDouble)
					{
						break;
					}
				}
				firstColor = params.line->colorVUse();
				firstGreen = params.line->greenVUse();
				firstDouble = params.line->doubleVUse();

				for (int k = 0; k < cur_downArraySize; k++, ++itDown)
				{
					Point p = itDown.pos();
					end_down = p;
					if (k > 0)
					{
						int ypixel = p.y * params.camera->width();
						uchar* pixel = dataImg + (ypixel + p.x) * 3;
						uchar h = pixel[0];
						uchar s = pixel[1];
						uchar v = pixel[2];
						uchar* pixelF = dataFieldImg + ypixel + p.x;
						if (firstDouble && k >= startDouble && k <= endDouble)
						{
							if (*(*itDown) > 254)
							{
								vote_for_double_down++;
								firstDouble = false;
							}
						}
						if (firstColor && k >= startColor && k <= endColor)
						{
							if (h >= params.line->h0()
									&& h <= params.line->h1()
									&& s >= params.line->s0()
									&& s <= params.line->s1()
									&& v >= params.line->v0()
									&& v <= params.line->v1())
							{
								vote_for_color_down++;
								firstColor = false;
							}
						}
						if (firstGreen && k >= startGreen && k <= endGreen)
						{
							if (*pixelF > 254)
							{
								greenVoter += 0.5;
								firstGreen = false;
							}
						}
					}
					if (!params.line->showVote() && !firstColor
							&& !firstGreen && !firstDouble)
					{
						break;
					}
				}

				if (SHOWGUI && params.line->showVote())
				{
					cv::line(guiImg, end_up, end_down, yellowColor(), 1);
					cv::circle(guiImg, end_up, 1, blueColor(), 1);
					cv::circle(guiImg, end_down, 1, redColor(), 1);
				}
			}
			else
			{
				not_valid_point++;
			}
		}

		bool checkIsValid = (COUNT_MID_P > (not_valid_point * 2));
		if (checkIsValid)
		{
			bool greenOk = true;
			bool upDoubleOk = true;
			bool downDoubleOk = true;
			bool upColorOk = true;
			bool downColorOk = true;
			double probability = 1;
			double totalVP = (COUNT_MID_P - not_valid_point);
			if (params.line->greenVUse())
			{
				int MIN_LINE_GREEN_VOTE = (params.line->greenVote() / 100.)
						* totalVP;
				greenOk = (greenVoter >= MIN_LINE_GREEN_VOTE);
				probability *= (greenVoter / totalVP);
			}
			if (params.line->doubleVUse())
			{
				int MIN_LINE_DOUBLE_VOTE =
						(params.line->doubleVote() / 100.) * totalVP;
				upDoubleOk = (vote_for_double_up >= MIN_LINE_DOUBLE_VOTE);
				downDoubleOk = (vote_for_double_down >= MIN_LINE_DOUBLE_VOTE);
				probability *= (max(vote_for_double_up, vote_for_double_down)
						/ totalVP);
			}
			if (params.line->colorVUse())
			{

				int MIN_LINE_COLOR_VOTE = (params.line->colorVote() / 100.)
						* totalVP;
				upColorOk = (vote_for_color_up >= MIN_LINE_COLOR_VOTE);
				downColorOk = (vote_for_color_down >= MIN_LINE_COLOR_VOTE);
				probability *= (max(vote_for_color_up, vote_for_color_down)
						/ totalVP);
			}

			bool colorOK = (downColorOk || upColorOk);
			bool doubleOK = (downDoubleOk || upDoubleOk);
			if (greenOk && colorOK && doubleOK)
			{
				allLines[i].setProbability(probability);
				resLines.push_back(allLines[i]);
			}
			else
			{
				if (params.line->showAllLine() && SHOWGUI)
				{
					line(guiImg, allLines[i].P1, allLines[i].P2, redColor(), 1,
							8);
				}
			}
		}
		else
		{
			if (params.line->showAllLine() && SHOWGUI)
			{
				line(guiImg, allLines[i].P1, allLines[i].P2, redMeloColor(), 1,
						8);
			}
		}

	}

	return resLines.size() > 0;
}
