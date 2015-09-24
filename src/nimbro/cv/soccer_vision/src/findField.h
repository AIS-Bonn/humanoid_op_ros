// Soccer Vision Node
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>
/**
* @file soccer_vision/src/findField.h
* @brief Implements the Find field methods of the NimbRo-OP Soccer Vison package.
* 
* \image html soccer_vision/orgGreenField.png Image showing pixels that belong to the green class (Field)
* 
* Strategy to detect the soccer field
* 
* For every column of the image and using a bottom-up strategy, count the green and non-green pixels. Stop counting 
* when the non-green counter is greater than certain threshold. Naturally, the position where the last green pixel was detected
* has to be taken into account because that will be consider as the boundary of the field.
* 
* 
* \image html soccer_vision/fieldDetection.png Approach to find the pixels that belong to the soccer field
* 
* Having a non-green pixel counter (blank counter) help to include the objects that are suppose to be within the playing area and to exclude regions that 
* are not. In our case the maximum number of non-green pixels allowed in a column is 45, this value has demostrated to work very well 
* with images of size (200x150).
* @code
* const unsigned int  MAX_BLANKS = 45;
* @endcode 
* 
* 
* After the column analysis and collecting all the pixels that belong to the green field the next step in the procedure is to compute the
* convex hull of the field area. The algorithm used to find the hull is the "monotone chain 2D convex hull algorithm", which has an
* asymptotic complexity of O(n log n). 
* 
* 
* The function that finds the hull points returns a list of coordinates in counter-clockwise order. This facilitates the analysis that 
* has to be done in order to create a binary mask that will contain the region that belongs to the green field.  The step after 
* computing the hullpoints is to connect them with a line to create a closed area. This line is created using the  
* "Bresenham line algorithm". Finally, the area inside the convex polygon is determined using a "Flood fill" algorithm. Having the field 
* binary mask is very useful because every thing that is not inside it can be consider noise or not desirable information. 
* 
* 
* The following images show a step by step example of the approach:
* <table border="0">
* <tr>     
*     <th>\image html soccer_vision/convexhull5.png Original Input image  </th>
*     <th></th>
*     <th>\image html soccer_vision/convexhull9.png Region after column analysis</th>
* </tr>
* <tr>     
*     <th>\image html soccer_vision/convexhull7.png Convex hull of the region</th>
*     <th></th>
*     <th>\image html soccer_vision/convexhull3.png Connected hull points</th>
* </tr>
* <tr>     
*     <th>\image html soccer_vision/convexhull4.png Convex hull area filled with a flood fill algorithm</th>
*     <th></th>
*     <th></th>
* </tr>
* </table>
*
* The final result looks as follows:
* \image html soccer_vision/convexhull1.png Result of the field area recognition
* 
**/

#ifndef FINDFIELD_H
#define FINDFIELD_H

#include <iostream>
#include <string>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "frameGrabber.h"
#include "globaldefinitions.h"




//using namespace std;
//using namespace soccervision;


namespace soccervision
{
	class FindField
	{
	public:
		
		HullPoint allgreenpoints[SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT];
		regionStack points_on_this_line;
			
		FindField(){}
		~FindField(){}
		
		
		void initialize(){}
		
		void findField(FrameGrabber & CamFrm);
	
	};

}
#endif
	