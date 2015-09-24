// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

#include "findBall.h"
#include "convexhullfunctions.h"

using namespace std;
using namespace soccervision;


	//.................................................................................
	void FindBall::findBall(FrameGrabber & CamFrm){
		
		for(int i= 0; i<(SUB_SAMPLING_HEIGHT*SUB_SAMPLING_WIDTH);i++){
			CamFrm.currentLUT.objects[CC_BALL].subimg[i] = (CamFrm.currentLUT.objects[CC_BALL].subimg[i] >= MIN_COLOR_INTENSITY ? 255 : 0);
		}

		//Eroding and dilating the resulting field area to eliminate small elements
		cv::Mat srcerode = cv::Mat(SUB_SAMPLING_HEIGHT,SUB_SAMPLING_WIDTH,CV_8U,CamFrm.currentLUT.objects[CC_BALL].subimg); 
		cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size( 2, 2));
		cv::erode( srcerode, srcerode, element );
		cv::dilate( srcerode, srcerode, element );
		
		//Setting the container of the ball information 
		//to to zeros and the flag of ballFoundOnTheField=false;
		CamFrm.orangeBallBag.reset();
		
		//the region marker buffer will help to look for regions in the subsampled image
		memset( CamFrm.regionFinderMarker, 0, sizeof(unsigned char)*SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT );	
		regionCounter = 0;	
		
		int pixelpos;
		regionData orangeRegionData;
		covMatrix  orangeRegionCovariance;
		
		//Search for orange pixels (bottom up)	
		for (int y = CamFrm.currentFieldData.bounding_box_maxy, stagger = 0; y >= CamFrm.currentFieldData.bounding_box_miny; y-=2, stagger = 1 - stagger ) {
			for (int x = CamFrm.currentFieldData.bounding_box_minx + stagger; x <= CamFrm.currentFieldData.bounding_box_maxx; x+=2 ) {

				pixelpos = SUB_SAMPLING_WIDTH*y + x;

				//In the FieldMask array values > 0 are considered to be in the field area 
				//0 in the regionFinderMarker means that the pixels was not checked and can be part of an orange blob
				//The orange area has to be  greater-equal than (>=MIN_COLOR_INTENSITY) to be accepted 

				if ( CamFrm.cameraMask[pixelpos] > 0 && CamFrm.FieldMask[pixelpos] > 0 && CamFrm.regionFinderMarker[pixelpos] == 0 && CamFrm.currentLUT.objects[CC_BALL].subimg[pixelpos] >= MIN_COLOR_INTENSITY){
					
					regionCounter ++;
					growBallRegion(CamFrm.cameraMask,CamFrm.regionFinderMarker, CamFrm.currentLUT.objects[CC_BALL].subimg, x ,y);
			
					if ( possibleBallStack.size() >= MIN_BALL_REGION_SIZE){
						
						possibleBallStack.fulldata(orangeRegionData);
						orangeRegionCovariance = possibleBallStack.covariance(orangeRegionData);
						int ballcenterx = (int)orangeRegionData.meanx;
						int ballcentery = (int)orangeRegionData.meany;
						int centerpos = SUB_SAMPLING_WIDTH*ballcentery + ballcenterx;
						
						bool IsballInTheField = (CamFrm.FieldMask[centerpos]>0);
						
						double maxe = MAX_ECCENTRICITY_LOW + (possibleBallStack.size() - MAX_ECC_SIZE_LOW)*MAX_ECC_SLOPE;
						double e = (IsballInTheField ? computeRegionEccentricity(orangeRegionCovariance) : maxe+1);

						if (e <= maxe && IsballInTheField){

							tf::Vector3 vecFromCameraToBall = pixelCameraCorrection(orangeRegionData.meanx, orangeRegionData.meany);

							//Rotating the previously calculated vector using the 
							//real position of the camera on the robot
							// TODO: Julio: EXTRACT THIS TO A compute_ball_ego_position() FUNCTION INSIDE pixelCameraCorrection.h!!
							tf::Vector3 vecRobotEyeToBallrot =  CamFrm.tf_egorot_cameraoptical_OriginBasis*vecFromCameraToBall;
							float lambda =  (BALL_CENTER_TO_FLOOR_DISTANCE - CamFrm.tf_egorot_cameraoptical_OriginVector.z() - ROBOT_TRUNK_TO_FLOOR_DISTANCE)/vecRobotEyeToBallrot.z();


							//computing the vector from Trunk (IMU position) to the ball 
							tf::Vector3 ego_ball_vector = CamFrm.tf_egorot_cameraoptical_OriginVector + lambda*vecRobotEyeToBallrot;
							double distanceToNewDetectedBall = sqrt((ego_ball_vector.x()*ego_ball_vector.x()) + (ego_ball_vector.y()*ego_ball_vector.y()));

							if (CamFrm.orangeBallBag.ballFoundOnTheField == false){
								CamFrm.orangeBallBag.ballFoundOnTheField = true;//a ball was found
								CamFrm.orangeBallBag.camera_world_vector = vecFromCameraToBall; //vector in camera world
								CamFrm.orangeBallBag.ego_ball_vector =  ego_ball_vector;//Vector from the IMU to the ball
								CamFrm.orangeBallBag.pos_x = orangeRegionData.meanx*SUB_SAMPLING_PARAMETER;//scaling back to the original image size
								CamFrm.orangeBallBag.pos_y = orangeRegionData.meany*SUB_SAMPLING_PARAMETER;//scaling back to the original image size
								CamFrm.orangeBallBag.pixelsize = possibleBallStack.size();
								CamFrm.orangeBallBag.ego_ball_distance = distanceToNewDetectedBall;
								
								double  radius2x =  ((double)orangeRegionData.min_x - (double)orangeRegionData.meanx);
								double  radius2y =  ((double)orangeRegionData.min_y - (double)orangeRegionData.meany);

								CamFrm.orangeBallBag.radiusA = 0;//(int)sqrt(radius1x*radius1x + radius1y*radius1y)*SUB_SAMPLING_PARAMETER;
								CamFrm.orangeBallBag.radiusB = (int)sqrt(radius2x*radius2x + radius2y*radius2y)*SUB_SAMPLING_PARAMETER;
							}
							else if (CamFrm.orangeBallBag.ballFoundOnTheField == true) {
						
								bool closer_ball = (distanceToNewDetectedBall < CamFrm.orangeBallBag.ego_ball_distance);
								bool much_bigger_ball = (possibleBallStack.size() >= (((int) CamFrm.orangeBallBag.pixelsize)+BALL_REGION_SIZE_INCR));
								bool much_smaller_ball = (possibleBallStack.size() < (((int) CamFrm.orangeBallBag.pixelsize)-BALL_REGION_SIZE_INCR));
								
								if ( much_bigger_ball || (!much_bigger_ball && closer_ball && !much_smaller_ball)) {
									CamFrm.orangeBallBag.ballFoundOnTheField = true;//a ball was found
									CamFrm.orangeBallBag.camera_world_vector = vecFromCameraToBall; //vector in camera world
									CamFrm.orangeBallBag.ego_ball_vector =  ego_ball_vector;//Vector from the IMU to the ball
									CamFrm.orangeBallBag.pos_x = orangeRegionData.meanx*SUB_SAMPLING_PARAMETER;//scaling back to the original image size
									CamFrm.orangeBallBag.pos_y = orangeRegionData.meany*SUB_SAMPLING_PARAMETER;//scaling back to the original image size
									CamFrm.orangeBallBag.pixelsize = possibleBallStack.size();
									CamFrm.orangeBallBag.ego_ball_distance = distanceToNewDetectedBall;

									double  radius2x =  ((double)orangeRegionData.min_x - (double)orangeRegionData.meanx);
									double  radius2y =  ((double)orangeRegionData.min_y - (double)orangeRegionData.meany);

									CamFrm.orangeBallBag.radiusA = 0;//(int)sqrt(radius1x*radius1x + radius1y*radius1y)*SUB_SAMPLING_PARAMETER;
									CamFrm.orangeBallBag.radiusB = (int)sqrt(radius2x*radius2x + radius2y*radius2y)*SUB_SAMPLING_PARAMETER;
								}
							}
						}
					}
				}
				else{
					CamFrm.regionFinderMarker[pixelpos]=255; //mark the value as evaluated
				}
			}
		}	
	}//END findBall function

	//.................................................................................
	void FindBall::growBallRegion(unsigned char CaMa[], unsigned char ReFiMa[], unsigned char Subimg[] , int x, int y){
	
		int pixelpos;
		possibleBallStack.clear();
		tempRegionStack.clear();
		PixelPosition pixorigin(x,y);
		PixelPosition pix(x,y);

		possibleBallStack.push(pixorigin);    
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

								possibleBallStack.push(pix);
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

	//.................................................................................
	double FindBall::computeRegionEccentricity(const covMatrix A){

		if (A.valid==true) {
			
				//Using the covariance matrix to calculate the eigenvalues
				//having them allow the calculation of the eccentricity value e = sqrt(1 - ev1/ev2)
				double term1 = (A.xx+A.yy)*0.5;
				double term2beforeSqrt = (4*A.xy*A.xy) 
							+ ((A.xx-A.yy)*(A.xx-A.yy));

			if (term2beforeSqrt <0){
				//returnning a very large Eccentricity value			  
				//means that the region is not round (ball like)
				return 1000; 
			}
			else{

				double term2 = sqrt(term2beforeSqrt)*0.5;
				double eigen1 = term1 + term2;
				double eigen2 = term1 - term2;

				double a = max(eigen1,eigen2);
				double b = min(eigen1,eigen2);

				double e = sqrt(1 - (b/a));				  
				return e;
			}

		}
		else{
			//returnning a very large Eccentricity value			  
			//means that the region is not round (ball like)
			return 1000; 	
		}  
	}
