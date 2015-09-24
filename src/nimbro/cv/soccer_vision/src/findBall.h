// Soccer Vision Node
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

/**
* @file soccer_vision/src/findBall.h
* @brief Implements the methods to find the orange ball.
* 
* Looking for the ball means looking for orange regions within the image that contains the orange color class information.  Of course, the regions found 
* in the image have to have certain characteristics, i.e., they have to be round to be consider a ball.
* 
* The analysis starts by defining the minimum orange value that will beconsider a pixel. In our case the variable MIN_COLOR_INTENSITY
* contains this threshold. All the pixels that do not satisfy this criteria will be set to zero (0) and will not participate in the region growing 
* step. One should remember that all the subimages have a pixel range [0,16] due to the image subsampling.
* 
* The criteria to find a region is to find all the pixels that  are connected using the "8-connected pixel neighborhood" element.
* \image html soccer_vision/8neighbors.png 8-connected pixel neighborhood
* 
* First, one orange pixel has to be found, then, all its neighbors are checked. All the neighbors that belong to the orange color class are added to the region and  
* checked for more orange neighbors. This is done until no more pixels of the same color class are found.
* 
* Finding an orange region does not ensure that it is a ball. This is why, further analysis is required. The central moments of the found regions will help us to 
* calculate how round a shape is. Using the covariance matrix of a give region one can calculate its eigenvectors that correspond to the major (a) and minor (b) axes of the
* ellipse that encloses the shape. Then, using the following formula it is possible to compute the eccentricity of the region:
* @code
* eccentricity = sqrt ( 1 - (b/a) );
* //where a and b are axes of the ellipse that the encloses the analyzed region
* @endcode  
* 
* The values of the eccentricity are always positive. Zero eccentricity means that the shape is a perfect circle. Values greather than 0 and smaller than 1 mean that the 
* shape is an ellipse. Naturally the closer the value gets to 1 the more  elongated the shape is.
* 
*  \image html soccer_vision/findBall_eccentricity.png Steps to compute the eccentricity of an orange region
**/

#ifndef FINDBALL_H
#define FINDBALL_H

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
#include "checkLimitsInSubImage.h"
#include "frameGrabber.h"
#include "convexhullfunctions.h"
#include "regionStack.h"
#include "pixelCameraCorrection.h"

using namespace soccervision;

namespace soccervision
{
	
	
	/**
	* @class FindBall
	* @brief Class that has the methods to find the orange ball in the images.	
	**/
	class FindBall
	{


	public:

		int regionCounter;
		regionStack possibleBallStack;
		regionStack tempRegionStack;     


		unsigned char MIN_COLOR_INTENSITY;
		unsigned char MIN_BALL_REGION_SIZE;
		unsigned char BALL_REGION_SIZE_INCR;
		double MAX_ECCENTRICITY_LOW;
		double MAX_ECCENTRICITY_HIGH;
		double MAX_ECC_SIZE_LOW;
		double MAX_ECC_SIZE_HIGH;
		double MAX_ECC_SLOPE;
		double ROBOT_TRUNK_TO_FLOOR_DISTANCE;
		double  BALL_CENTER_TO_FLOOR_DISTANCE;

		/*! \fn FindBall()
		*   \brief Constructor that initialize all the necesary structures and variables.
		*/
		FindBall(){
			MIN_COLOR_INTENSITY = 2;
			MIN_BALL_REGION_SIZE = 10;
			BALL_REGION_SIZE_INCR = 20;
			MAX_ECCENTRICITY_LOW = 0.82;
			MAX_ECCENTRICITY_HIGH = 1.00;
			MAX_ECC_SIZE_LOW = MIN_BALL_REGION_SIZE;
			MAX_ECC_SIZE_HIGH = 80;
			MAX_ECC_SLOPE = (MAX_ECCENTRICITY_HIGH-MAX_ECCENTRICITY_LOW)/(MAX_ECC_SIZE_HIGH-MAX_ECC_SIZE_LOW);
			ROBOT_TRUNK_TO_FLOOR_DISTANCE = 0.61f; // TODO: Still correct?
			BALL_CENTER_TO_FLOOR_DISTANCE = 0.095f; // TODO: Still correct?
		}
		
		/*! \fn virtual void ~FindBall()
		*   \brief Destructor
		*/		
		~FindBall(){}    
		
		/*! \fn void findBall(FrameGrabber & CamFrm);
		*   \brief Finds the orange ball in the current frame.
		*   \param CamFrm pointer to a frameGrabber object, which contains all the necessary data.
		*/				
		void findBall(FrameGrabber & CamFrm);
		
		/*! \fn growBallRegion(unsigned char CaMa[], unsigned char ReFiMa[], unsigned char Subimg[] , int x, int y)
		*   \brief Grows orange regions starting from a (x,y) pixel coordinate.
		*   \param CaMa Camera mask
		*   \param ReFiMa Region Marker
		*   \param Subimg Image that contains the orange class information
		*   \param x x-coordinate where an orange pixel was found
		*   \param y y-coordinate where an orange pixel was found
		*/						
		void growBallRegion(unsigned char CaMa[], unsigned char ReFiMa[], unsigned char Subimg[] , int x, int y);
		
		/*! \fn computeRegionEccentricity(const covMatrix A)
		*   \brief Computes the Eccentricity of a region
		*   \param A The covariance matrix of the orange region
		*/			
		double computeRegionEccentricity(const covMatrix A);
		
		
	};
	
}
#endif