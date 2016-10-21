//OptimizorParam.hpp
// Created on: June 3, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include "opencv2/opencv.hpp"
#include <stdlib.h>     /* abs */
#include <vision_module/Tools/General.hpp>

using namespace cv;

class OptimizorParam
{
public:
	OptimizorParam(float data, float step = 0.001f) :
			data(data), step(step)
	{

	}
	float data;
	float step;
};
