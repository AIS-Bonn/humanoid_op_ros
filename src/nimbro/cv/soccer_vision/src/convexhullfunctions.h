// Soccer Vision Node
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>
/**
* @file soccer_vision/src/convexhullfunctions.h
* @brief This file contains the functions to compute the convex hull of a 2D region.
* @author Julio Pastrana <pastrana@ais.uni-bonn.de>
* 
* 
**/

#ifndef CONVEXHULLFUNCTIONS_H
#define CONVEXHULLFUNCTIONS_H

#include <iostream>
#include <string>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>


#include "globaldefinitions.h"
#include "regionStack.h"

using namespace std;

namespace soccervision
{

	struct HullPoint {
		int x, y;

		bool operator <(const HullPoint &p) const {
				return x < p.x || (x == p.x && y < p.y);
		}
			
		void initialize(){
		x = 0;
		y = 0;
		}
	};

	//...........................................................................
	// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
	// Returns a positive value, if OAB makes a counter-clockwise turn,
	// negative for clockwise turn, and zero if the HullPoints are collinear.
	long long cross(const HullPoint &O, const HullPoint &A, const HullPoint &B);

	//...........................................................................


	//...........................................................................
	// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
	// Returns a positive value, if OAB makes a counter-clockwise turn,
	// negative for clockwise turn, and zero if the HullPoints are collinear.
	long long cross(const PixelPosition &O, const PixelPosition &A, const PixelPosition &B);
	//...........................................................................

	//...........................................................................
	//...........................................................................
	// Implementation of monotone chain 2D convex hull algorithm.
	// Asymptotic complexity: O(n log n).

	// Returns a list of HullPoints on the convex hull in counter-clockwise order.
	// Note: the last HullPoint in the returned list is the same as the first one.
	vector<HullPoint> convex_hull(vector<HullPoint> P);
	//...........................................................................

	vector<HullPoint> convex_hull_on_array(HullPoint P[], int size);
	//...........................................................................


	//...........................................................................

	void convex_hull_on_fixed_array(HullPoint P[], int size, regionStack & R);
	//...........................................................................

	//...........................................................................

	void convex_hull_on_fixed_pixelregion_array(regionStack &RS, regionStack & R);
	//...........................................................................


	//...........................................................................
	// Implementation of line generation using Bresenham's algorithm
	void lineBresenham_on_stack(int x0, int y0, int x1, int y1,  regionStack & line);
	//...........................................................................




	//...........................................................................
	// Implementation of line generation using Bresenham's algorithm
	void lineBresenham(int x0, int y0, int x1, int y1, vector <HullPoint> & puntos);
	//...........................................................................



	//This function tells if a line is within certain white area
	//using Bresenham's algorithm to generate the line.
	//Note that the image that gets to this function is one of the subsampled ones.
	bool isLineOnImage(int x0, int y0, int x1, int y1, unsigned char image[]);
	//...........................................................................

}
#endif
