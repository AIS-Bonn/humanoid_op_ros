// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

#include "findGoal.h"
#include "convexhullfunctions.h"
#include "pixelCameraCorrection.h"
#include "checkLimitsInSubImage.h"

using namespace std;
using namespace soccervision;

	//.................................................................................
	void FindGoal::findGoal(FrameGrabber & CamFrm){
		
		//deleting previously detected sections
		CamFrm.goalHulls.reset();
		CamFrm.goalLowerPoints.reset();
		CamFrm.goalMiddlePoints.reset();

		regionCounter = 0;
		possibleGoalStack.clear();
		vector<HullPoint> temp_goalHull;
		
		//the region marker buffer will help to look for regions in the subsampled image
		memset( CamFrm.regionFinderMarker, 0, sizeof(unsigned char)*SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT );	
		
		for (int xy = 0; xy < SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT; xy++ ) {
			if ( CamFrm.cameraMask[xy] > 0 && CamFrm.currentLUT.objects[CC_GOAL].subimg[xy] >= MIN_COLOR_INTENSITY){
				CamFrm.currentLUT.objects[CC_GOAL].subimg[xy] = 255;      
			}
			else{
				CamFrm.currentLUT.objects[CC_GOAL].subimg[xy] = 0; //mark the value as evaluated
			} 
		} 
		
		//Eroding the resulting goal area to eliminate small elements
		cv::Mat srcerode = cv::Mat(SUB_SAMPLING_HEIGHT,SUB_SAMPLING_WIDTH,CV_8U, CamFrm.currentLUT.objects[CC_GOAL].subimg); 
		
		cv::Mat element1 = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2, 2)); 
		cv::erode( srcerode, srcerode, element1 );

		cv::Mat element2 = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 5)); 
		cv::dilate( srcerode, srcerode, element2 );
		
		int lowery = CamFrm.currentFieldData.bounding_box_miny - FIELD_HULL_PIX_BUFFER;
		if(lowery < 0) lowery = 0;
		
		unsigned int i;
		int j, pix;
		
		for (int y = CamFrm.currentFieldData.bounding_box_maxy, stagger = 0; y >= lowery; y-=2, stagger = 1 - stagger ) {
			for (int x = CamFrm.currentFieldData.bounding_box_minx + stagger; x <= CamFrm.currentFieldData.bounding_box_maxx; x+=2 ) {

				int pixelpos = SUB_SAMPLING_WIDTH*y + x;
				int bufferpixelpos = SUB_SAMPLING_WIDTH*(y+FIELD_HULL_PIX_BUFFER) + x;
				while(bufferpixelpos >= SUB_SAMPLING_NUM_PIXELS)
					bufferpixelpos -= SUB_SAMPLING_WIDTH;

				//In the FieldMask array values > 0 are considered to be in the field area 
				//0 in the regionFinderMarker means that the pixels was not checked and can be part of an yellow blob
				//The yellow pixel has to be  greater-equal than (>=MIN_COLOR_INTENSITY) to be accepted 

				if ( CamFrm.cameraMask[pixelpos] > 0 && (CamFrm.FieldMask[pixelpos] > 0 || CamFrm.FieldMask[bufferpixelpos] > 0) && CamFrm.regionFinderMarker[pixelpos] == 0 && CamFrm.currentLUT.objects[CC_GOAL].subimg[pixelpos] >= MIN_COLOR_INTENSITY){

					//If a yellow pixel was found clear all the buffers
					//before expanding the yellow region
					possibleGoalStack.clear();
					temp_goalHull.clear();

					growGoalRegion(CamFrm.cameraMask,CamFrm.regionFinderMarker, CamFrm.currentLUT.objects[CC_GOAL].subimg, x ,y);
					
					if (possibleGoalStack.size()>=MIN_GOAL_REGION_SIZE){
						
						// TODO: Julio: Go through all find functions (ball, goal etc...) and replace the convex hull function with the one that takes the regionstack directly
						//copy the blob 
						for (int k=0; k<possibleGoalStack.size(); k++){
							PixelPosition yellowPoint = possibleGoalStack.getPixelAtPos(k);
							allgoalpoints[k].x = yellowPoint.x;
							allgoalpoints[k].y = yellowPoint.y;
						}
						
						//find the convex hull of the yellow region
						temp_goalHull = convex_hull_on_array(allgoalpoints,possibleGoalStack.size());
						
						// adjust hull points that are almost inside the field hull
						for (i=0; i<temp_goalHull.size();i++)
						{
							for(j = 0;j <= FIELD_HULL_PIX_BUFFER;j++) // TODO: Change white line rejection condition to "if any hull points are outside the field mask" (single boolean flag)
							{
								pix = SUB_SAMPLING_WIDTH*(temp_goalHull[i].y+j) + temp_goalHull[i].x;
								if(pix < SUB_SAMPLING_NUM_PIXELS)
								{
									if(CamFrm.FieldMask[pix] > 0)
									{
										temp_goalHull[i].y += j;
										break;
									}
								}
							}
						}
						
						//get the hullmin and hullmax point
						float hullmin_y,hullmin_x,hullmax_x, hullmax_y;

						hullmin_y = temp_goalHull[0].y;
						hullmax_y = temp_goalHull[0].y;
						hullmin_x = temp_goalHull[0].x;
						hullmax_x = temp_goalHull[0].x;

						//computing the min and max values of the convex hull
						//this way the bounding box can be assembled 
						for (i=0; i<temp_goalHull.size();i++)
						{
							if(temp_goalHull[i].x<hullmin_x) hullmin_x = temp_goalHull[i].x;
							if(temp_goalHull[i].x>hullmax_x) hullmax_x = temp_goalHull[i].x;
							if(temp_goalHull[i].y<hullmin_y) hullmin_y = temp_goalHull[i].y;
							if(temp_goalHull[i].y>hullmax_y) hullmax_y = temp_goalHull[i].y;
						}

						float hull_mean_x = (hullmin_x + hullmax_x)*0.5;
						float hull_mean_y = (hullmin_y + hullmax_y)*0.5;
						
						bool found_negative_side = false;
						bool found_positive_side = false;
						float max_pos_side_distance=0;
						float max_neg_side_distance=0;
						HullPoint positive_side_point;
						HullPoint negative_side_point;
						
						for (i=0; i<temp_goalHull.size()-1;i++)
						{
							int pixelpos = SUB_SAMPLING_WIDTH*temp_goalHull[i].y + temp_goalHull[i].x;
							int pixelpos_top_of_convexhull =  SUB_SAMPLING_WIDTH*(int)hullmin_y + (int)hull_mean_x;
							
							//Only accepting point within the Field area
							if (CamFrm.FieldMask[pixelpos] > 0 && CamFrm.FieldMask[pixelpos_top_of_convexhull] == 0 && temp_goalHull[i].y>hull_mean_y){ // TODO: Change white line rejection condition to "if any hull points are outside the field mask"
								
								float valx = (temp_goalHull[i].x-hull_mean_x);
								float valy = (temp_goalHull[i].y-hull_mean_y);
								
								//float distance = sqrt( valx*valx + valy*valy); // Euclidean distance
								float distance =  abs(valx) + abs(valy); // Manhattan distance
								
								if (valx>=0){
									if (max_pos_side_distance<= distance){
										max_pos_side_distance = distance;
										found_positive_side = true;
										positive_side_point = temp_goalHull[i];
									}
								}

								if (valx<0){
									if (max_neg_side_distance<= distance){
										max_neg_side_distance = distance;
										found_negative_side = true;
										negative_side_point = temp_goalHull[i];
									}				
								}

							}
						}//endfor
									
						if (found_negative_side || found_positive_side){
							HullPoint mean_position_point;					
							mean_position_point.x = (int) hull_mean_x;
							mean_position_point.y = (int) hull_mean_y;
							CamFrm.goalMiddlePoints.goalSections[regionCounter].push(mean_position_point);
						}
						
						//Checking the points. If they are too close. They will be merged
						if (found_negative_side && found_positive_side){
							
							double value_neg_x = negative_side_point.x - hull_mean_x;
							double value_neg_y = negative_side_point.y - hull_mean_y;

							double value_pos_x = positive_side_point.x - hull_mean_x;
							double value_pos_y = positive_side_point.y - hull_mean_y;
							
							double length_neg = sqrt(value_neg_x*value_neg_x + value_neg_y*value_neg_y);
							double length_pos = sqrt(value_pos_x*value_pos_x + value_pos_y*value_pos_y);
							double dotproduct = (value_pos_x*value_neg_x)+(value_pos_y*value_neg_y);
							
							double normalized_result = dotproduct/(length_neg*length_pos);
							
							//Values have to be merged if normalized_result is > 0.3			    			    
							if ( normalized_result > 0.2 ){
								HullPoint mergin_positions;
								mergin_positions = ((positive_side_point.y > negative_side_point.y) ? positive_side_point : negative_side_point );
								CamFrm.goalLowerPoints.goalSections[regionCounter].push(mergin_positions);
																
								found_negative_side = false;
								found_positive_side = false;
							}
						}
						
						if (found_negative_side){			  
							CamFrm.goalLowerPoints.goalSections[regionCounter].push(negative_side_point);
						}
						
						if (found_positive_side){
							CamFrm.goalLowerPoints.goalSections[regionCounter].push(positive_side_point);			  
						}
						
						//copy the vector into the possible goal regions
						for (unsigned int k=0; k<temp_goalHull.size(); k++){
							CamFrm.goalHulls.goalSections[regionCounter].push(temp_goalHull[k]);
						}
						
						CamFrm.goalLowerPoints.regionCounter ++;
						CamFrm.goalMiddlePoints.regionCounter ++;
						CamFrm.goalHulls.regionCounter++;
						regionCounter++;
					}
				}
				else{
					CamFrm.regionFinderMarker[pixelpos]=255; //mark the value as evaluated
				}
			}
		}	
			
		temp_goalHull.clear();
		
	}//END findBall function


	//.................................................................................
	void FindGoal::growGoalRegion(unsigned char CaMa[], unsigned char ReFiMa[], unsigned char Subimg[] , int x, int y){
	
		int pixelpos;
		possibleGoalStack.clear();
		tempRegionStack.clear();
		PixelPosition pixorigin(x,y);
		PixelPosition pix(x,y);

		possibleGoalStack.push(pixorigin);    
		tempRegionStack.push(pixorigin);   

		pixelpos = SUB_SAMPLING_WIDTH*y + x;
		ReFiMa[pixelpos]=255;//mark the value as evaluated 
		
		//Offsets to the 8 closest neighbors.
		int neighborsOffsets[8][2] = { {-1,-1},{0,-1},{1,-1},{-1,0},{1,0},{-1,1},{0,1},{1,1}};        
		
			while(tempRegionStack.empty() == false ){

				if (tempRegionStack.pop(pixorigin)){
					
					for (int i=0;i<8;i++){
						
						pix.x = pixorigin.x + neighborsOffsets[i][0];
						pix.y = pixorigin.y + neighborsOffsets[i][1];				  

						if (checkLimitsInSubImage(pix.x, pix.y)){

							pixelpos = SUB_SAMPLING_WIDTH*pix.y + (pix.x);
							if (CaMa[pixelpos] > 0 && ReFiMa[pixelpos] == 0 && Subimg[pixelpos]>=MIN_COLOR_INTENSITY){

								possibleGoalStack.push(pix);
								tempRegionStack.push(pix);
							}
							ReFiMa[pixelpos]=255; //mark the value as evaluated
						}
					}
				}
			}//end WHILE
		
	//Note: After calling this function the data of the yellow region is going to
	//be stored in the regionStack object "possibleGoalStack"		
	}
