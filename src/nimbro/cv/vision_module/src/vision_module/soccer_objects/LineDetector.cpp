//LineDetector.cpp
// Created on: May 14, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <vision_module/soccer_objects/LineDetector.hpp>

bool LineDetector::GetLines(Mat &rawHSV, Mat & fieldMask,
		Mat &guiImg, bool SHOWGUI, const Mat &lineBinary,
		vector<LineSegment> &resLines)
{
	int MIN_LINE_DOUBLE_VOTE = params.line.lineVoteDouble->get();
	int MIN_LINE_VOTE = params.line.lineVote->get();
	int MIN_LINE_COLOR_VOTE = params.line.lineVoteColor->get();
	vector<Vec4i> linesP;
	HoughLinesP(lineBinary, linesP, 1, M_PI / 45, 20,
			params.line.MinLineLength->get(), 20);

	for (size_t i = 0; i < linesP.size(); i++)
	{
		Vec4i lP = linesP[i];
		LineSegment tmpLine(Point2d(lP[0], lP[1]), Point2d(lP[2], lP[3]));

		vector<cv::Point2d> midds = tmpLine.GetMidPoints(3); //2^3+1 = 16
		int lineVoter = 0;
		int vote_for_double = 0;
		int vote_for_color = 0;
		uchar *dataImg = fieldMask.data;
		//	printf("size= %d\n", midds.size());
		for (size_t j = 0; j < midds.size(); j++)
		{

			int jumpMin = params.line.jumpMin->get();
			int jumpMax = params.line.jumpMin->get();
			double distanceToZero = GetDistance(
					cv::Point2d((params.camera.width->get() / 2),
							(params.camera.height->get())), midds[j]);
			LineInterpolation interP(
					LineSegment(cv::Point2d(0, jumpMax),
							cv::Point2d(params.camera.height->get(), jumpMin)));
			double jump;
			if (!interP.GetValue(distanceToZero, jump))
			{
				//printh(CRed, "Error In Programming!");
				continue;
			}
			LineSegment tocheck = tmpLine.PerpendicularLineSegment(jump,
					midds[j]);
			cv::LineIterator it(lineBinary, tocheck.P1, tocheck.P2, 8);

			vector<uchar> buf(it.count);

			int currentCounter = 0;
			for (int k = 0; k < it.count; k++, ++it)
			{
				uchar val=*(*it);
				if ( val > 10)
				{
					vote_for_double++;
					currentCounter++;
				}
				if (currentCounter >= 2)
					break;
			}

			cv::LineIterator itHSV(rawHSV, tocheck.P1, tocheck.P2, 8);

			vector<uchar> bufHSV(itHSV.count);

			for (int k = 0; k < itHSV.count; k++, ++itHSV)
			{
				cv::Vec3b hsvC = (cv::Vec3b) *itHSV;
				if (hsvC[0] >= params.line.h0->get()
						&& hsvC[0] <= params.line.h1->get()
						&& hsvC[1] >= params.line.s0->get()
						&& hsvC[1] <= params.line.s1->get()
						&& hsvC[2] >= params.line.v0->get()
						&& hsvC[2] <= params.line.v1->get())
				{
					vote_for_color++;
					break;
				}
			}

			int safeToShow = 0;
			if (tocheck.P1.x >= 0 && tocheck.P1.y >= 0
					&& tocheck.P1.x < params.camera.width->get()
					&& tocheck.P1.y < params.camera.height->get())
			{
				safeToShow++;
				uchar* pixelP = dataImg
						+ ((((int) tocheck.P1.y * params.camera.width->get())
								+ (int) tocheck.P1.x) * 1);
				if (*pixelP > 50)
					lineVoter++;
			}
			if (tocheck.P2.x >= 0 && tocheck.P2.y >= 0
					&& tocheck.P2.x < params.camera.width->get()
					&& tocheck.P2.y < params.camera.height->get())
			{
				safeToShow++;
				uchar* pixelP = dataImg
						+ ((((int) tocheck.P2.y * params.camera.width->get())
								+ (int) tocheck.P2.x) * 1);
				if (*pixelP > 50)
					lineVoter++;
			}

			if (safeToShow >= 2)
			{
				if (SHOWGUI && params.debug.showLineD->get())
				{
					cv::line(guiImg, tocheck.P1, tocheck.P2, yellowColor(), 1);
				}
			}
		}
		if (lineVoter > MIN_LINE_VOTE && vote_for_double > MIN_LINE_DOUBLE_VOTE
				&& vote_for_color > MIN_LINE_COLOR_VOTE)
			resLines.push_back(tmpLine);

	}
	return resLines.size() > 0;

}

//bool LineDetector::GetLines(const Mat &lineBinary,
//		vector<LineSegment> &resLines)
//{
//	int W = lineBinary.size().width;
//	int H = lineBinary.size().height;
//	double *image;
//	image = (double *) malloc(W * H * sizeof(double));
//
//	//cast into LSD image type
//	uchar *data = (uchar *) lineBinary.data;
//	for (int i = 0; i < W; i++)
//	{
//		for (int j = 0; j < H; j++)
//		{
//			image[i + j * W] = data[i + j * W];
//		}
//	}
//
//	//run LSD
//	double *out;
//	int n;
//	out = lsd(&n, image, W, H);
//
//	for (int j = 0; j < n; j++)
//	{
//		//define segment end-points
//		Point pt1(out[0 + j * 7], out[1 + j * 7]);
//		Point pt2(out[ 2 + j * 7 ],out[ 3 + j * 7 ]);
//		resLines.push_back(LineSegment(pt1,pt2));
//	}
//	return resLines.size() > 0;
//}

//bool LineDetector::GetLines(const Mat &lineBinary,
//		vector<LineSegment> &resLines)
//{
//	int R = 3;
//	std::vector<LSEG> lSegs, lSegsPPHT;
//	std::vector<double> errors;
//	LSWMS lswms(lineBinary.size(), R, 100, false);
//	lswms.run(lineBinary, lSegs, errors);
//
//	for (size_t j = 0; j < lSegs.size(); j++)
//	{
//
//		resLines.push_back(LineSegment(lSegs[j][0],lSegs[j][1]));
//	}
//	return resLines.size() > 0;
//}
