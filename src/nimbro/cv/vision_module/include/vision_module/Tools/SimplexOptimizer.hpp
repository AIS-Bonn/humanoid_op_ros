//SimplexOptimizer.hpp
// Created on: Feb 20, 2016
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
//     This code is based on (Copyright (C) 2010 Botao Jia): http://www.codeguru.com/cpp/article.php/c17505/Simplex-Optimization-Algorithm-and-Implemetation-in-C-Programming.htm

#include <vector>
#include <limits>
#include <algorithm>
#include <functional>
#include <iostream>
#include <vision_module/Tools/OptimizorParam.hpp>

/**
* @ingroup VisionModule
*
* @brief Nelder–Mead optimization
**/
template<class T>
class SimplexOptimizer
{
private:
	vector<OptimizorParam> parameters;
	unsigned int paramsCount;
	T& obj;
	float (T::*pFunc)(const vector<float>& parameters);
public:
	SimplexOptimizer(T& obj,
			float (T::*pFunc)(const vector<float>& parameters)) :
			paramsCount(0), obj(obj), pFunc(pFunc)
	{

	}

	bool TuneForBest(int maxIteration,
			double tol = 1E8 * std::numeric_limits<float>::epsilon(),
			std::vector<std::vector<float> > x = std::vector<
					std::vector<float> >())
	{
		std::vector<float> init;
		for(size_t pi=0;pi<paramsCount;pi++)
		{
			init.push_back(parameters[pi].data);
		}
		int N = paramsCount;                         //space dimension
		const double a = 1.0, b = 1.0, g = 0.5, h = 0.5;   //coefficients
														   //a: reflection  -> xr
														   //b: expansion   -> xe
														   //g: contraction -> xc
														   //h: full contraction to x1
		std::vector<float> xcentroid_old(N, 0);   //simplex center * (N+1)
		std::vector<float> xcentroid_new(N, 0);   //simplex center * (N+1)
		std::vector<float> vf(N + 1, 0);    //f evaluated at simplex vertexes
		int x1 = 0, xn = 0, xnp1 = 0; //x1:   f(x1) = min { f(x1), f(x2)...f(x_{n+1} }
									  //xnp1: f(xnp1) = max { f(x1), f(x2)...f(x_{n+1} }
									  //xn:   f(xn)<f(xnp1) && f(xn)> all other f(x_i)
		int cnt = 0; //iteration step number

		if (x.size() == 0) //if no initial simplex is specified
		{ //construct the trial simplex
			//based upon the initial guess parameters
			std::vector<float> del(init);
			std::transform(del.begin(), del.end(), del.begin(),
					std::bind2nd(std::divides<float>(), 20));	//'20' is picked
															//assuming initial trail close to true

			for (int i = 0; i < N; ++i)
			{
				std::vector<float> tmp(init);
				tmp[i] += del[i];
				x.push_back(tmp);
			}
			x.push_back(init);		               //x.size()=N+1, x[i].size()=N

			//xcentriod
			std::transform(init.begin(), init.end(), xcentroid_old.begin(),
					std::bind2nd(std::multiplies<float>(), N + 1));
		}		                             //constructing the simplex finished

		//optimization begins
		for (cnt = 0; cnt < maxIteration; ++cnt)
		{

			for (int i = 0; i < N + 1; ++i)
			{
				vf[i] = (obj.*pFunc)(x[i]);
			}

			x1 = 0;
			xn = 0;
			xnp1 = 0;		         //find index of max, second max, min of vf.

			for (size_t i = 0; i < vf.size(); ++i)
			{
				if (vf[i] < vf[x1])
				{
					x1 = i;
				}
				if (vf[i] > vf[xnp1])
				{
					xnp1 = i;
				}
			}

			xn = x1;

			for (size_t i = 0; i < vf.size(); ++i)
			{
				if (vf[i] < vf[xnp1] && vf[i] > vf[xn])
					xn = i;
			}
			//x1, xn, xnp1 are found

			std::vector<float> xg(N, 0);	//xg: centroid of the N best vertexes
			for (size_t i = 0; i < x.size(); ++i)
			{
				if (i != (size_t)xnp1)
					std::transform(xg.begin(), xg.end(), x[i].begin(),
							xg.begin(), std::plus<float>());
			}
			std::transform(xg.begin(), xg.end(), x[xnp1].begin(),
					xcentroid_new.begin(), std::plus<float>());
			std::transform(xg.begin(), xg.end(), xg.begin(),
					std::bind2nd(std::divides<float>(), N));
			//xg found, xcentroid_new updated

			//termination condition
			float diff = 0;        //calculate the difference of the simplex centers
							   //see if the difference is less than the termination criteria
			for (int i = 0; i < N; ++i)
				diff += fabs(xcentroid_old[i] - xcentroid_new[i]);

			if (diff / N < tol)
				break;              //terminate the optimizer
			else
				xcentroid_old.swap(xcentroid_new); //update simplex center

			//reflection:
			std::vector<float> xr(N, 0);
			for (int i = 0; i < N; ++i)
				xr[i] = xg[i] + a * (xg[i] - x[xnp1][i]);
			//reflection, xr found

			float fxr = (obj.*pFunc)(xr); //record function at xr

			if (vf[x1] <= fxr && fxr <= vf[xn])
				std::copy(xr.begin(), xr.end(), x[xnp1].begin());

			//expansion:
			else if (fxr < vf[x1])
			{
				std::vector<float> xe(N, 0);
				for (int i = 0; i < N; ++i)
					xe[i] = xr[i] + b * (xr[i] - xg[i]);
				if ((obj.*pFunc)(xe) < fxr)
					std::copy(xe.begin(), xe.end(), x[xnp1].begin());
				else
					std::copy(xr.begin(), xr.end(), x[xnp1].begin());
			} //expansion finished,  xe is not used outside the scope

			//contraction:
			else if (fxr > vf[xn])
			{
				std::vector<float> xc(N, 0);
				for (int i = 0; i < N; ++i)
					xc[i] = xg[i] + g * (x[xnp1][i] - xg[i]);
				if ((obj.*pFunc)(xc) < vf[xnp1])
					std::copy(xc.begin(), xc.end(), x[xnp1].begin());
				else
				{
					for (size_t i = 0; i < x.size(); ++i)
					{
						if (i != (size_t)x1)
						{
							for (int j = 0; j < N; ++j)
								x[i][j] = x[x1][j] + h * (x[i][j] - x[x1][j]);
						}
					}
				}
			} //contraction finished, xc is not used outside the scope

		} //optimization is finished

		for(size_t pi=0;pi<x[x1].size();pi++)
		{
			parameters[pi].data = x[x1][pi];
		}
		cout << " Tune counter = " << cnt << endl;
		if (cnt == maxIteration)
		{
			return false;
		}
		return true;
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
