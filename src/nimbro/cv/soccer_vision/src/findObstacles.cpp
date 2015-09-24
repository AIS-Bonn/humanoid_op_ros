// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

#include "findObstacles.h"
#include "checkLimitsInSubImage.h"
#include "pixelCameraCorrection.h"
#include "filterUsingShoulderPlane.h"

using namespace std;
using namespace soccervision;

	//.................................................................................
	void FindObstacles::findObstacles(FrameGrabber & CamFrm){
		
		memset(img_possible_obstacles, 0, sizeof(unsigned char)*SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT );	
		memset(img_region_marker, 0, sizeof(unsigned char)*SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT );
		regionCounter = 0;
		CamFrm.DetectedObstacles.clear();
		possibleObstacleStack.clear();
		tempRegionStack.clear();

		for (int i=0; i<(SUB_SAMPLING_HEIGHT*SUB_SAMPLING_WIDTH);i++){
			if (CamFrm.FieldMask[i] ==255 && CamFrm.currentLUT.objects[CC_BLACK].subimg[i] >= MIN_COLOR_INTENSITY
				&& CamFrm.currentLUT.objects[CC_WHITE].subimg[i] < MIN_OTHER_COLOR_INTENSITY
				&& CamFrm.currentLUT.objects[CC_BALL].subimg[i] < MIN_OTHER_COLOR_INTENSITY
				&& CamFrm.currentLUT.objects[CC_GOAL].subimg[i] < MIN_OTHER_COLOR_INTENSITY){
				img_possible_obstacles[i] =255;
			}
		}
			//Eroding and dilating the resulting field area to eliminate small elements
			cv::Mat element1 = cv::getStructuringElement( 0, cv::Size( 0+5, 0+5 ),cv::Point( 0, 0 ) );
			cv::Mat element2 = cv::getStructuringElement( 0, cv::Size( 0+5, 0+5 ),cv::Point( 0, 0 ) );
			cv::Mat srcerode = cv::Mat(SUB_SAMPLING_HEIGHT,SUB_SAMPLING_WIDTH,CV_8U,img_possible_obstacles); 	    
			cv::erode( srcerode, srcerode, element1 );	    
			cv::dilate( srcerode, srcerode, element2 );	    
		
			// TODO: Delete me too! AND MAKE OWN POINT VECTORS OR WHATEVER FOR DISPLAY (DON'T HIJACK GOAL ONES)
	// 		CamFrm.goalLowerPoints.reset();
	// 		CamFrm.goalMiddlePoints.reset();

			//Search for dark blobs (bottom up)
			for (int y = CamFrm.currentFieldData.bounding_box_maxy, stagger = 0; y >= CamFrm.currentFieldData.bounding_box_miny; y-=2, stagger = 1 - stagger ) {
				for (int x = CamFrm.currentFieldData.bounding_box_minx + stagger; x <= CamFrm.currentFieldData.bounding_box_maxx; x+=2 ) {
		
					int pixelpos = SUB_SAMPLING_WIDTH*y + x;

					if ( CamFrm.cameraMask[pixelpos] > 0 && img_region_marker[pixelpos] == 0 && img_possible_obstacles[pixelpos] > 0){
			
						regionCounter++;
						growObstacleRegion(CamFrm.cameraMask, img_region_marker, img_possible_obstacles, x , y);

						if (possibleObstacleStack.size()>=150){
													
							
							CamFrm.obstacleHull.clear();
							convex_hull_on_fixed_pixelregion_array(possibleObstacleStack, CamFrm.obstacleHull);
						
							if(CamFrm.obstacleHull.size() > 0)
							{
								possibleObstacleStack.fulldata(possibleObstacleData); 
									
								//REFACTOR AFTER TEST
								
								const int minyincr = 3;
								int yincr = (int) (0.2*(possibleObstacleData.max_y-possibleObstacleData.min_y));
								if(yincr < minyincr) yincr = minyincr;
								int ythresh = possibleObstacleData.max_y - yincr;
								
								int x, y;
								int xsum = 0, ysum = 0, count = 0;
								int xmax = -1, xmin = SUB_SAMPLING_WIDTH;
								for (int k=0; k<CamFrm.obstacleHull.size(); k++){
									x = CamFrm.obstacleHull.arr[k].x;
									y = CamFrm.obstacleHull.arr[k].y;						
									if((y <= possibleObstacleData.max_y) && (y >= ythresh))
									{
										xsum += x;
										ysum += y;
										if(x > xmax) xmax = x;
										if(x < xmin) xmin = x;
										count++;
									}
								}
								
								float OBST_X_POS = (xmax + xmin + ((float) xsum)/count)/3.0;
								float OBST_Y_POS = ((float) ysum)/count;
								// TODO: Once in world vectors, use the xmin and xmax points (i.e. (xmin, OBST_Y_POS) and (xmax, OBST_Y_POS)) to calculate the obstacle diameter
								//       Also provide the world vectors for all three points! (min, max, mean)
								
								HullPoint hp;
								hp.x = (int) (OBST_X_POS + 0.5);
								hp.y = (int) (OBST_Y_POS + 0.5);
								CamFrm.goalMiddlePoints.regionCounter++;
								CamFrm.goalMiddlePoints.goalSections[CamFrm.goalMiddlePoints.regionCounter].push(hp);
								if(xmin <= xmax)
								{
									CamFrm.goalLowerPoints.regionCounter++;
									hp.x = xmin;
									CamFrm.goalLowerPoints.goalSections[CamFrm.goalLowerPoints.regionCounter].push(hp);
									hp.x = xmax;
									CamFrm.goalLowerPoints.goalSections[CamFrm.goalLowerPoints.regionCounter].push(hp);
								}
								//END REFACTOR AFTER TEST
								
								CamFrm.DetectedObstacles.push_back(possibleObstacleData);
							}
						}
					}
				}
			}
			
	// 	cv::imshow("after erosion and dilation",srcerode );
	// 	cv::Mat bw5 = cv::Mat(SUB_SAMPLING_HEIGHT,SUB_SAMPLING_WIDTH,CV_8U, img_possible_obstacles);	
	// 	cv::imshow("img_possible_obstacles", bw5 );
	}

	//.................................................................................
	void FindObstacles::growObstacleRegion(unsigned char CaMa[], unsigned char ReFiMa[], unsigned char Subimg[] , int x, int y){
	
		int pixelpos;
		possibleObstacleStack.clear();
		tempRegionStack.clear();
		PixelPosition pixorigin(x,y);
		PixelPosition pix(x,y);

		possibleObstacleStack.push(pixorigin);    
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
							if (CaMa[pixelpos] > 0 && ReFiMa[pixelpos] == 0 && Subimg[pixelpos] >= MIN_COLOR_INTENSITY){

								possibleObstacleStack.push(pix);
								tempRegionStack.push(pix);
							}
							ReFiMa[pixelpos]=255; //mark the value as evaluated
						}
					}

				}
			}//end WHILE

		
	//Note: After calling this function the data of the orange region is going to
	//be stored in the regionStack object "possibleBallStack"		
	}


