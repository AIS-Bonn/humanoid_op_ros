// Soccer Vision Node
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

#include "findField.h"
#include "convexhullfunctions.h"
#include "yuvClasses.h"


using namespace std;
using namespace soccervision;

void FindField::findField(FrameGrabber & CamFrm){
			

			const unsigned int  MAX_BLANKS = 45;
			const unsigned char MIN_COLOR_INTENSITY = 4;
			const unsigned int MIN_FIELD_SIZE = 100;
			
			//Note:
			// (0)   pixel values that identifiy positions not on the field
			//(255)  pixel values that identifiy the field
			//Initially there is no pixel in the field area.
			
			memset( CamFrm.FieldBoundary,0, sizeof(unsigned char)*SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT );
			memset( CamFrm.FieldMask,0, sizeof(unsigned char)*SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT );
			CamFrm.currentFieldData.reset();

			for (int x = 0; x < SUB_SAMPLING_WIDTH; x++ ) { //<- Left to right.

				unsigned int blank = 0; //<- Blank pixel counter.
			
				for (int y=(SUB_SAMPLING_HEIGHT-1); y>=0; y-- ) { //<- Bottom-up. 

					int pixelpos = SUB_SAMPLING_WIDTH*y + x;	  
					if (CamFrm.cameraMask[pixelpos] != 0){
					
						//Merging SubImages Field, Ball, Black and White
						unsigned char fieldvalue = CamFrm.currentLUT.objects[CC_FIELD].subimg[pixelpos];

						if (fieldvalue>=MIN_COLOR_INTENSITY ){ 
							//255 identify pixels in the field
							CamFrm.FieldBoundary[pixelpos] = 255;
						}
						else{
							blank++; //Pixel that does not belong to the field
							if ( blank >= MAX_BLANKS ) { //<- N continuous blanks found in a column.			    
								y=0; //<- Found rough ending, jump out for the next turn.
							}
						}
					}//END if mask

				}
			}
			
			
// 			cv::imshow("thecolumns", cv::Mat(SUB_SAMPLING_HEIGHT,SUB_SAMPLING_WIDTH,CV_8U,CamFrm.FieldBoundary) );
// 			cv::waitKey(5);
			
			//Eroding and dilating the resulting field area to eliminate small elements
			cv::Mat srcerode = cv::Mat(SUB_SAMPLING_HEIGHT,SUB_SAMPLING_WIDTH,CV_8U,CamFrm.FieldBoundary); 
			cv::Mat element = cv::getStructuringElement( 0, cv::Size( 0+5, 0+5 ),cv::Point( 0, 0 ) );
			cv::erode( srcerode, srcerode, element );	    
			cv::dilate( srcerode, srcerode, element );	    

			
			//Collecting the points
			unsigned int greenPointsCounter = 0;
			
			for (int x = 0; x < SUB_SAMPLING_WIDTH; x++ ) { //<- Left to right.

			unsigned int blank = 0; //<- Blank pixel counter.
			
			for (int y=(SUB_SAMPLING_HEIGHT-1); y>=0; y-- ) { //<- Bottom-up. 

				int pixelpos = SUB_SAMPLING_WIDTH*y + x;		  
				if (CamFrm.cameraMask[pixelpos] != 0){
				
				//If the value is 255 the pixel belongs to the field
				if ( CamFrm.FieldBoundary[pixelpos] == 255){

					allgreenpoints[greenPointsCounter].x = x;
					allgreenpoints[greenPointsCounter].y = y;
					greenPointsCounter++;
				}		        
					else{
				blank++; //Pixel that does not belong to the field
				if ( blank >= MAX_BLANKS ) { //<- N continuous blanks found in a column.			    
					y=0; //<- Found rough ending, jump out for the next turn.
				}	
				}
				
				}//END if mask
			}
			}	

			//Computing the convex hull of the green area	        	  
			if (greenPointsCounter > MIN_FIELD_SIZE){
				
				
			convex_hull_on_fixed_array(allgreenpoints,greenPointsCounter, CamFrm.soccerFieldHull);
			
			//Computing lines between the convex hull points
			//also the bounding rectangle is calculated in this function	    
			
			int minx = 0;
			int miny = 0;
			int maxx = 0;
			int maxy = 0;   
			
			points_on_this_line.clear();
			
			int j;
			for (j=0; j<CamFrm.soccerFieldHull.size()-1; j++){
			
				PixelPosition pix1 = CamFrm.soccerFieldHull.getPixelAtPos(j);
				PixelPosition pix2 = CamFrm.soccerFieldHull.getPixelAtPos(j+1);
				lineBresenham_on_stack(pix1.x, pix1.y, pix2.x, pix2.y, points_on_this_line);

			
				if (j==0){
					minx = maxx = pix1.x;
					miny = maxy = pix1.y;
				}

				if (maxx < pix1.x){maxx = pix1.x;}
				if (minx > pix1.x){minx = pix1.x;}

				if (maxy < pix1.y){maxy = pix1.y;}
				if (miny > pix1.y){miny = pix1.y;}
			}
			
			
			//saving the data of the current field
			if(j > 0)
			{
				CamFrm.currentFieldData.bounding_box_minx = minx;
				CamFrm.currentFieldData.bounding_box_miny = miny;
				CamFrm.currentFieldData.bounding_box_maxx = maxx;
				CamFrm.currentFieldData.bounding_box_maxy = maxy;
			}
			
			//Marking all border points on the FieldMask
			for (j=0; j<points_on_this_line.size(); j++){
				
				int xp = points_on_this_line.getPixelAtPos(j).x;
				int yp = points_on_this_line.getPixelAtPos(j).y;

				int pixpos = SUB_SAMPLING_WIDTH*yp + xp;
				CamFrm.FieldMask[pixpos] = 255;
			}	    



			HullPoint linestart;
			HullPoint lineend;
			bool startpoint = false;	
			linestart.initialize();
			lineend.initialize();
			
			//This will generate the mask that contaings all the field area.
			//This loop will scann all the lines within the convexhull's bounding rectangle	    
			for (int y=miny; y<=maxy; y++){
				for (int x=minx; x<=maxx; x++){

					int pixpos = SUB_SAMPLING_WIDTH*y + x;

					if (CamFrm.FieldMask[pixpos] == 255){

						if (startpoint==false){
							linestart.x = x;
							linestart.y = y;
							startpoint = true;
						}

						lineend.x = x;
						lineend.y = y;

					}
				}

				startpoint = false;
				for (int k=linestart.x; k<=lineend.x; k++){
					int pixpos2 = SUB_SAMPLING_WIDTH*linestart.y + k;
					CamFrm.FieldMask[pixpos2] = 255;
				}
			}
			
			points_on_this_line.clear();
			}//END if (greenPointsCounter > 100)
						
			

		}//END find Field Function
