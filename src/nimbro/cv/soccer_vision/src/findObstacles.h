// Soccer Vision Node
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>
/**
* @file soccer_vision/src/findObstacles.h
* @brief Implements the find obstacles methods of the NimbRo-OP Soccer Vison package.
* 
*  
**/

#ifndef FINDOBSTACLES_H
#define FINDOBSTACLES_H

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


#include "globaldefinitions.h"
#include "frameGrabber.h"
#include "convexhullfunctions.h"
#include "regionStack.h"

namespace soccervision{
	
	class FindObstacles
	{


	public:

		
		unsigned char img_possible_obstacles[SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT];
		unsigned char img_region_marker[SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT];
		unsigned char MIN_COLOR_INTENSITY, MIN_OTHER_COLOR_INTENSITY;
		
		int regionCounter;
		regionStack possibleObstacleStack;
		regionStack tempRegionStack; 
		regionData  possibleObstacleData;
		
		FindObstacles(){
			MIN_COLOR_INTENSITY = 4;
			MIN_OTHER_COLOR_INTENSITY = 2;
		}
		
		~FindObstacles(){}    
		
		
		void findObstacles(FrameGrabber & CamFrm);
		void growObstacleRegion(unsigned char CaMa[], unsigned char ReFiMa[], unsigned char Subimg[] , int x, int y);
	};
}//End namespace

#endif