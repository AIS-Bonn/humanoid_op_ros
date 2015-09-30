//FieldDetector.cpp
// Created on: May 12, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <vision_module/soccer_objects/FieldDetector.hpp>
using namespace boost::timer;

#define MAX_DESTANCE_FROM_BUUTTOM_OF_IMAGE 200 //pixel
bool FieldDetector::GetPoints(Mat &binaryFrame, vector<Point> &resPoints,
		vector<vector<Point> > &allFieldContours)
{

	if (params.field.erode->get() > 0)
	{
		erode(binaryFrame, binaryFrame, Mat(), Point(-1, -1),
				params.field.erode->get());
	}
	if (params.field.dilate->get() > 0)
	{
		dilate(binaryFrame, binaryFrame, Mat(), Point(-1, -1),
				params.field.dilate->get());
	}
	if (params.field.erode2->get() > 0)
	{
		erode(binaryFrame, binaryFrame, Mat(), Point(-1, -1),
				params.field.erode2->get());
	}
	if (params.field.dilate2->get() > 0)
	{
		dilate(binaryFrame, binaryFrame, Mat(), Point(-1, -1),
				params.field.dilate2->get());
	}

	findContours(binaryFrame.clone()/*To have binaryFrame after this function*/,
			allFieldContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	std::sort(allFieldContours.begin(), allFieldContours.end(),
			SortFuncDescending);
	bool ret = false;
	int totalResult = 0;
	for (size_t i = 0; i < allFieldContours.size(); i++) // iterate through each contour.
	{
		if (totalResult >= params.field.maxContourCount->get())
		{
			return ret;
		}
		Rect rec = boundingRect(allFieldContours[i]);
		double area = contourArea(allFieldContours[i]);
		if (std::abs(params.camera.height->get() - Bottom(rec))
				<= MAX_DESTANCE_FROM_BUUTTOM_OF_IMAGE
				&& area >= params.field.minArea->get())
		{
			vector<Point> tmpContour = allFieldContours[i];
			approxPolyDP(tmpContour, tmpContour,
					cv::arcLength(tmpContour, true) * 0.003, true);
			for (size_t pIt = 0; pIt < tmpContour.size(); pIt++)
			{
				resPoints.push_back(tmpContour[pIt]);
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

