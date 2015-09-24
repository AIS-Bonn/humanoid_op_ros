// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

#ifndef FINDLANDMARKSANDLINES_H
#define FINDLANDMARKSANDLINES_H

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



namespace soccervision{
	
	void executeGuoHallIteration(cv::Mat& im, int iter);
	void thinningWithGuoHallAlgorithm(cv::Mat& im);
	// bool filterTheLandmarkUsingShoulderPlane(tf::Matrix3x3 M, tf::Vector3 v, float x_filter_range, float y_filter_range, int pixel_x, int pixel_y);
	// 
	// 
	// 
	// bool filterTheLandmarkUsingShoulderPlane(tf::Matrix3x3 M, tf::Vector3 v, float x_filter_range, float y_filter_range, int pixel_x, int pixel_y){
	// 	
	// 	tf::Vector3 tmpvec;
	// 	tmpvec = pixelCameraCorrection(pixel_x,pixel_y);
	// 	tmpvec = compute_bodymark_ego_position(M,v,tmpvec);
	// 	
	// 	if (tmpvec.x()< x_filter_range && fabs(tmpvec.y())<y_filter_range){
	// 		return false;
	// 	}
	// 	
	// 	return true;
	// }
	////////////////////////////////////////////////////////////////////////

	class FindLandmarksAndLines
	{
		private:
		
			unsigned char MIN_COLOR_INTENSITY;

	
		public:
		
			int regionCounter;
			unsigned char img_skeleton1_copy[SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT];
			unsigned char img_skeleton2_copy[SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT];
					

		
			regionStack stackOfWhitePixels;
			
			regionStack neighborsStack1;
			regionStack neighborsStack2;
			regionStack neighborsStack3;
			regionStack neighborsStack4;	
					
			regionStack probableLs;
			regionStack probableTsXs;
			
			regionStack detectedTs;
			regionStack detectedXs;
			regionStack detectedLs;
			
			
			regionStack Line1;
			regionStack Line2;
			regionStack Line3;
			regionStack Line4;
			
			
			FindLandmarksAndLines(){
				MIN_COLOR_INTENSITY = 4;
			}
			~FindLandmarksAndLines(){}		
			
			void findLandmarks_and_Lines(FrameGrabber & CamFrm);

			
	};
}


#endif

