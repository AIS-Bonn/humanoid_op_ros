//HillOptimizer.hpp
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

template<class T>
class HillOptimizer
{
private:
	vector<OptimizorParam> parameters;
	unsigned int paramsCount;
	T& obj;
	float (T::*pFunc)(vector<OptimizorParam>& parameters);
public:
	HillOptimizer(T& obj, float (T::*pFunc)(vector<OptimizorParam>& parameters)) :
		paramsCount(0),obj(obj), pFunc(pFunc)
	{

	}

	bool TuneForBest(int maxIteration, double tol = 0.001)
	{
		double lastRes = Tune();
		cout << " First Tune =" << lastRes << endl;
		int successCounter = 0;
		for (int i = 0; i < maxIteration; i++)
		{
			double curRes = Tune();
			cout << " Tune counter = " << i << " TuneRes = " << curRes << endl;
			if ((lastRes - curRes) < tol)
			{
				successCounter++;
			}
			else
			{
				successCounter = 0;
			}
			if (successCounter >= 3)
			{
				return true;
			}
			lastRes = curRes;
		}
		return false;
	}

	float Tune()
	{
		float bestResult = 999999999;
		for (size_t i = 0; i < paramsCount; i++)
		{
			float resOld = (obj.*pFunc)(parameters);
			float resAbove = resOld;
			float resBelow = resOld;

			bool tuned = false;
			while (!tuned)
			{
				float oldParameter = parameters[i].data;

				float stepAbove = oldParameter + parameters[i].step;
				float stepBelow = oldParameter - parameters[i].step;

				parameters[i].data = stepAbove;
				resAbove = (obj.*pFunc)(parameters);

				parameters[i].data = stepBelow;
				resBelow = (obj.*pFunc)(parameters);

				if (resAbove < resBelow && resAbove < resOld)
				{
					parameters[i].data = stepAbove;
					resOld = resAbove;
				}
				else if (resAbove > resBelow && resBelow < resOld)
				{
					parameters[i].data = stepBelow;
					resOld = resBelow;
				}
				else
				{
					parameters[i].data = oldParameter;
					bestResult = min(bestResult, resOld);
					tuned = true;
				}
			}
		}
		return bestResult;
	}

	void setParameters(vector<OptimizorParam> _parameters)
	{
		parameters = _parameters;
		paramsCount = _parameters.size();
	}
	vector<OptimizorParam> getParameters() const
	{
		return parameters;
	}
};
