//FieldDetector.hpp
// Created on: May 12, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/General.hpp>
#include <vision_module/soccer_objects/IContourDetector.hpp>
#include <vision_module/Tools/SortFuntions.hpp>
#include <algorithm>    // std::sort
using namespace cv;
class FieldDetector:public IDetector
{
public:
	vector < cv::Point > BodyMaskContourInverted;
	bool GetPoints( Mat &binaryFrame,
			vector < Point >  &resPoints,	vector< vector< Point > > &allFieldContours);
	void FindInField(const Mat &srcHsvImg,const  Mat &tmplateGrayImg, Mat *dstGrayImgs,
			hsvRangeC *ranges,bool *inTemplate, int size=1);
	inline bool Init(){
		cv::FileStorage fr("/nimbro/share/launch/config/vision/BodyMask.yml", cv::FileStorage::READ);
			cv::FileNode fileNode = fr["IGUS"];
			read(fileNode, BodyMaskContourInverted);
			fr.release();
			if (BodyMaskContourInverted.size() < 6)
			{
				ROS_ERROR( "Check BodyMask.yml!");
			}

		return true;};
	vector < cv::Point > getBodyMaskContourInRaw(float rot)
	{
		vector < cv::Point > res;
		if (BodyMaskContourInverted.size() <6)
			return res;
		for (size_t i = 0; i < BodyMaskContourInverted.size(); i++)
		{

			cv::Point rotated = RotateAroundPoint(BodyMaskContourInverted[i], rot);
			cv::Point p(rotated.x + params.camera.width->get() / 2., abs(rotated.y - 480));
			if (p.inside(cv::Rect(0, 0, params.camera.width->get(), params.camera.height->get())))
			{
				if (res.size() == 0)
				{
					res.push_back(cv::Point(p.x,params.camera.height->get()));
				}
				res.push_back(p);
			}
		}
		if (res.size() > 0)
		{
			int xlast = res[res.size() - 1].x;
			res.push_back(cv::Point(xlast, params.camera.height->get()));
		}
		return res;
	}
};

