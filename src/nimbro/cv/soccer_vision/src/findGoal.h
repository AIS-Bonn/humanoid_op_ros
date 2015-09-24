// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

/**
* @file soccer_vision/src/findGoal.h
* @brief Implements the NimbRo-OP Soccer Vison package.
* 
* 
* 
* 
* 
**/

#ifndef FINDGOAL_H
#define FINDGOAL_H

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
#include "regionStack.h"
//#include "convexhullfunctions.h"
// #include "checkLimitsInSubImage.h"
// #include "pixelCameraCorrection.h"

//using namespace std;
//using namespace soccervision;

namespace soccervision{
	class FindGoal
	{
	public:
		int regionCounter;
		regionStack possibleGoalStack;
		regionStack tempRegionStack;     
		
		HullPoint allgoalpoints[SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT];
		
		unsigned char MIN_COLOR_INTENSITY;
		int MIN_GOAL_REGION_SIZE;     
		int FIELD_HULL_PIX_BUFFER;
		double ROBOT_TRUNK_TO_FLOOR_DISTANCE;

		FindGoal(){	  
			MIN_COLOR_INTENSITY = 3;
			MIN_GOAL_REGION_SIZE = 45;
			FIELD_HULL_PIX_BUFFER = 4;
			ROBOT_TRUNK_TO_FLOOR_DISTANCE = 0.61f;
		}
		~FindGoal(){
		}    
		
		void findGoal(FrameGrabber & CamFrm);	
		void growGoalRegion(unsigned char CaMa[], unsigned char ReFiMa[], unsigned char Subimg[] , int x, int y);	

	};
}//END namespace
#endif