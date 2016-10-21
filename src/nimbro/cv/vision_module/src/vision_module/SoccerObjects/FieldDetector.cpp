//FieldDetector.cpp
// Created on: May 12, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <vision_module/SoccerObjects/FieldDetector.hpp>
using namespace boost::timer;

bool FieldDetector::GetPoints(Mat &binaryFrame, vector<Point> &resPoints,
		vector<vector<Point> > &allFieldContours)
{

	if (params.field->erode() > 0)
	{
		erode(binaryFrame, binaryFrame, Mat(), Point(-1, -1),
				params.field->erode());
	}
	if (params.field->dilate() > 0)
	{
		dilate(binaryFrame, binaryFrame, Mat(), Point(-1, -1),
				params.field->dilate());
	}
	if (params.field->erode2() > 0)
	{
		erode(binaryFrame, binaryFrame, Mat(), Point(-1, -1),
				params.field->erode2());
	}
	if (params.field->dilate2() > 0)
	{
		dilate(binaryFrame, binaryFrame, Mat(), Point(-1, -1),
				params.field->dilate2());
	}

	findContours(binaryFrame.clone(),
			allFieldContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	resPoints.reserve(params.field->maxContourCount());

	std::sort(allFieldContours.begin(), allFieldContours.end(),
			SortFuncDescending);
	bool ret = false;
	int totalResult = 0;
	for (size_t i = 0; i < allFieldContours.size(); i++)
	{
		if (totalResult >= params.field->maxContourCount())
		{
			return ret;
		}
		Rect rec = boundingRect(allFieldContours[i]);
		double area = contourArea(allFieldContours[i]);
		if (std::abs(params.camera->height() - Bottom(rec))
				<= params.field->maxDownDiffPixel()
				&& area >= params.field->minArea())
		{
			approxPolyDP(allFieldContours[i], allFieldContours[i],
					cv::arcLength(allFieldContours[i], true) * 0.003, true);
			for (size_t pIt = 0; pIt < allFieldContours[i].size(); pIt++)
			{
				resPoints.push_back(allFieldContours[i][pIt]);
			}

			ret = true;
		}
		totalResult++;
	}

	return ret;
}

void FieldDetector::FindInField(const Mat &srcHsvImg, const Mat &tmplateGrayImg,
		Mat *dstGrayImgs, hsvRangeC *ranges, bool *inTemplate, int size)
{
	const int srcSize = srcHsvImg.rows * srcHsvImg.cols;
	int* indexs = new int[4];
	for (int k = 0; k < size; k++)
	{
		indexs[k] = 0;
	}
	uchar* srcHsvImg_D = srcHsvImg.data;
	uchar* tmplateGrayImg_D = tmplateGrayImg.data;

	for (int i = 0; i < srcSize; i++)
	{
		ushort h = srcHsvImg_D[0], s = srcHsvImg_D[1], v = srcHsvImg_D[2];
		if (tmplateGrayImg_D[0] >= 254)
		{
			for (int k = 0; k < size; k++)
			{

				if (h >= ranges[k].h0->get() && h <= ranges[k].h1->get()
						&& s >= ranges[k].s0->get() && s <= ranges[k].s1->get()
						&& v >= ranges[k].v0->get() && v <= ranges[k].v1->get())
				{
					dstGrayImgs[k].data[indexs[k]] = 255;
				}
				else
				{
					dstGrayImgs[k].data[indexs[k]] = 0;
				}
			}
		}
		else
		{
			for (int k = 0; k < size; k++)
			{

				if (inTemplate[k])
					continue;
				if (h >= ranges[k].h0->get() && h <= ranges[k].h1->get()
						&& s >= ranges[k].s0->get() && s <= ranges[k].s1->get()
						&& v >= ranges[k].v0->get() && v <= ranges[k].v1->get())
				{
					dstGrayImgs[k].data[indexs[k]] = 255;
				}
				else
				{
					dstGrayImgs[k].data[indexs[k]] = 0;
				}
			}
		}

		tmplateGrayImg_D += 1;
		srcHsvImg_D += 3;
		for (int k = 0; k < size; k++)
		{
			indexs[k]++;
		}
	}

	delete[] indexs;
}

