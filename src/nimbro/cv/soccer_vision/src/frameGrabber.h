// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

/**
* @file soccer_vision/src/frameGrabber.h
* @brief Container of the NimbRo-OP Soccer Vison package.
* 
* The FrameGrabber class works as a container for all the images, vectors, regions that are going to be used during the image processing task. The idea behind 
* this container is that all the classes can access, store and share data in a common place.
* 
**/


#ifndef FRAMEGRABBER_H
#define FRAMEGRABBER_H




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

#include <tf/transform_datatypes.h>

#include "globaldefinitions.h"
#include "yuvClasses.h"
#include "convexhullfunctions.h"
#include "ballInformationBag.h"
#include "goalInformationBag.h"
#include  "fieldInformationBag.h"
#include "regionStack.h"

using namespace std;

namespace soccervision
{
	/*! FrameGrabber class */
	class FrameGrabber
	{
		public:

			//! Timestamp
			/*! This variable contains the time stamp of the current frame. */
			double time;
			
			//! Frame raw data
			/*! This buffer contains the pixel data as it was retieved from the camera. */
			unsigned char frame_raw_data[ORG_IMAGE_WIDTH*ORG_IMAGE_HEIGHT*3];
			
			//! Classified data
			/*! This buffer contains the classiefied data after each pixel was assigned to its corresponding class. */
			unsigned char classified_data[ORG_IMAGE_WIDTH*ORG_IMAGE_HEIGHT];
			
			//! Camera mask
			/*! This buffer contains the a flag (true or false) that defines if each pixel in the subimages will be taken into consideration or will be descarted during the execution.*/
			unsigned char cameraMask[SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT];
			
			//! Field detection buffer
			/*! This buffer stores the pixels of the green area with in the image. From this pixels a mask that defienes the current field area will be computed.*/			
			unsigned char FieldBoundary[SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT];

			//! Field mask
 			/*! This buffer stores the area of the image where the soccer field was detected*/			
			unsigned char FieldMask[SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT];
			
			//! Region finder
 			/*! This buffer helps during the detection of regions of different colors (blobs).*/
			unsigned char regionFinderMarker[SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT];
			
			//! Classification object
 			/*! This object contains all the color look-up tables that identify each color class.*/
			YUVColorLUT  currentLUT;

			//! Field Stack
 			/*! This stack contains the (x,y) image coordinates that belong to the current field. */
			regionStack soccerFieldHull;
			
			regionStack obstacleHull;			
			
			PossibleGoalSections goalHulls;
			PossibleGoalSections goalMiddlePoints;
			
			//! Goal detected posts
 			/*! This stack contains the (x,y) image coordinates that identify the lowest points  of a goal post. */			
			PossibleGoalSections goalLowerPoints;
			
			//! Field information
 			/*! If the green field was detected this object will store the data that corresponds to that region.*/			
			FieldInformationBag currentFieldData;
			
			//! Ball information
 			/*! If an orange ball was found this object contains all the data that corresponds to that region.*/						
			BallInformationBag orangeBallBag;

			tf::Matrix3x3 tf_egorot_cameraoptical_OriginBasis;
			tf::Vector3 tf_egorot_cameraoptical_OriginVector;
			
			tf::Matrix3x3 tf_trunk_cameraoptical_OriginBasis;
			tf::Vector3 tf_trunk_cameraoptical_OriginVector;
			float x_shoulder_filter_range;
			float y_shoulder_filter_range;

			
			unsigned int imagecounter;
			
			vector<regionData> DetectedObstacles;
			
			vector<HullPoint> L_landmarks;
			vector<HullPoint> T_landmarks;
			vector<HullPoint> X_landmarks;
			vector<HullPoint> lines;

			regionStack fieldXShapes;     
			regionStack fieldTShapes;
			regionStack fieldLShapes;
			
		/**
       * Constructor.
       * The constructor will initialize all the necesary variables 
       */
		FrameGrabber(){
			imagecounter = 0;
			x_shoulder_filter_range = 0.08;
			y_shoulder_filter_range = 0.13;
		}
		
		/**
       * Destructor.
       * The destructor will clean all the vectors that were used during the soccer vision execution.
       */		
		~FrameGrabber(){
			DetectedObstacles.clear();
			L_landmarks.clear();
			T_landmarks.clear();
			X_landmarks.clear();
			lines.clear();
		}
		

		
		//----------------------------------------
/*! \fn bool readCameraMask(const string inputName)
*    \brief Opens and reads the file that contains the camera mask.
*    \param inputName The name of the file.
*/		
		bool readCameraMask(const string inputName){

			string line;
			int y=0, x=0;

			std::string absName = ros::package::getPath("soccer_vision") + "/" + inputName;


				ROS_INFO("Reading camera mask from: %s", inputName.c_str() );

				ifstream objectfile (absName.c_str());
				if (objectfile.is_open())
				{
					while ( objectfile.good() )
					{
						getline (objectfile,line);

						x = 0;
						for (unsigned int i=0; i<line.size(); i++){		
						if (line[i] == '0')
							cameraMask[SUB_SAMPLING_WIDTH*y + x] = (unsigned char)0;
						else if (line[i] == '1')
							cameraMask[SUB_SAMPLING_WIDTH*y + x] = (unsigned char)1;
						else 
							cameraMask[SUB_SAMPLING_WIDTH*y + x] = (unsigned char)0;
						x++;
						}
						y++;

					}
					objectfile.close();
				}
				else{
					ROS_ERROR("Unable to find camera mask: %s", inputName.c_str() );
					return false;
				}


			return true;
			//End reading file
		}	
		//----------------------------------------

	};
}	
#endif
