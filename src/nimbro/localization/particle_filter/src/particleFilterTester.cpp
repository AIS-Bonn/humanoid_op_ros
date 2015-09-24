#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"


#include <cstdio>
#include <iostream>
#include <ctime>
#include <math.h>
#include <vector>
#include <time.h>

#include <soccer_vision/Detections.h>
#include <field_model/field_model.h>

#include <ros/ros.h>

using namespace cv;
using namespace std;

//--------------------------------------------------------------------
inline bool checkFieldLimits(int x, int y)
{
  if (x<0 || y<0)
	  return false;

  if (x>=1040  || y>=740)
	  return false;

  return true;
}
//--------------------------------------------------------------------



//--------------------------------------------------------------------
double checkAngleLimits(double angle)
{
  if (angle>360) return 0; 
  if (angle<0) return 360;
  else return angle;
}
//--------------------------------------------------------------------


//--------------------------------------------------------------------
double cross_product( Point a, Point b ){
   return a.x*b.y - a.y*b.x;
}
//--------------------------------------------------------------------


//--------------------------------------------------------------------

double distance_to_line( Point begin, Point end, Point x ){
	
   //translate the begin to the origin
   end -= begin;
   x -= begin;

   //¿do you see the triangle?
   double area = cross_product(x, end);
   return area / norm(end);
}
//--------------------------------------------------------------------

//--------------------------------------------------------------------

double distance_to_point( Point begin, Point end){
	
   double x = (begin.x-end.x);
   double y = (begin.y-end.y);
   return sqrt(  (x*x) + (y*y) );   
}
//--------------------------------------------------------------------




//--------------------------------------------------------------------
void theRobot( Mat img, Point robotpos ,double angle )
{
  int thickness = -1;
  int lineType = 8;

  circle(img,robotpos , 8, cv::Scalar( 255, 0, 0 ), thickness , lineType);	

  ellipse( img,
           robotpos,
           Size( 0, 16 ),
           angle,
           0,
           180,
           Scalar( 255, 0, 0 ),
           thickness,
           lineType );


}
//--------------------------------------------------------------------


//--------------------------------------------------------------------
void theParticle( Mat img, Point robotpos ,double angle, double probability)
{
  int thickness = -1;
  int lineType = 8;

  //probability value [0,1]
  int colorOfProbability = (int)(255 - (probability*255.0));


  circle(img,robotpos , 8, cv::Scalar( colorOfProbability, colorOfProbability,  255), thickness , lineType);	

  ellipse( img,
           robotpos,
           Size( 0, 16 ),
           angle,
           0,
           180,
           Scalar(colorOfProbability, colorOfProbability,  255 ),
           thickness,
           lineType );
}
//--------------------------------------------------------------------


//--------------------------------------------------------------------
void drawFieldOfView(Mat img, Mat Rt){

	  Point fielOfViewPoints[1][3];
	  fielOfViewPoints[0][0] = Point( Rt.at<float>(0,0), Rt.at<float>(0,1));
	  fielOfViewPoints[0][1] = Point( Rt.at<float>(1,0), Rt.at<float>(1,1));
	  fielOfViewPoints[0][2] = Point( Rt.at<float>(2,0), Rt.at<float>(2,1));

	  const Point* ppt[1] = { fielOfViewPoints[0] };
	  int npt[] = { 3 };

 
          polylines(img, ppt, npt, 1, 1, Scalar( 0, 255, 255 ), 1, 8);

}
//--------------------------------------------------------------------


//--------------------------------------------------------------------

struct landmarkData {
  double x;
  double y;
  double z;
  int  type;
} ;

//--------------------------------------------------------------------


//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
int main(int argc, char** argv)
{

	

	ros::init(argc, argv, "ParticleTester");
	ros::NodeHandle particleTesterNode;

	vector < Point > theLandmarks;
	vector < bool >  isLandmarkInsight;
	vector < unsigned int >   landmarkType;
	vector < Point > contours;
	vector < landmarkData > theLandmarksAfterRandomSelection;

	float data[6] = {0,0,-500,500,500,500};
	float rot_data[4] = {0,0,0,0};


	

	srand (time(NULL)); // initialize random seed: //
	



    Mat teenSizeField( 740, 1040, CV_8UC3);    
    teenSizeField = teenSizeField*0 + 255;   
 

    //Half field
    rectangle(teenSizeField,
            Point(70,70),  
            Point(520,670),
            Scalar(0, 255, 0, 0), 
            1, 8, 0);

    //Half field
    rectangle(teenSizeField,
            Point(520,70),
            Point(970,670),
            Scalar(0, 255, 0, 0), 
            1, 8, 0);


    //Goalie area A
    rectangle(teenSizeField,
            Point(70,145),
            Point(170,595),
            Scalar(0, 255, 0, 0), 
            1, 8, 0);

    //Goal area A
    rectangle(teenSizeField,
            Point(10,240),
            Point(70,500),
            Scalar(0, 255, 0, 0), 
            1, 8, 0);


    //Penalty mark A
    circle(teenSizeField,Point(280,370) ,5, cv::Scalar( 0, 255, 0 ), 1,1);	


    //Goalie area B
    rectangle(teenSizeField,
            Point(870, 140),
            Point(970,595),
            Scalar(0, 255, 0, 0), 
            1, 8, 0);


    //Goal area B
    rectangle(teenSizeField,
            Point(970,240),
            Point(1030,500),
            Scalar(0, 255, 0, 0), 
            1, 8, 0);


    //Penalty mark A
    circle(teenSizeField,Point(760,370) ,5, cv::Scalar( 0, 255, 0 ), 1,1);	



    //Field Middle circle
    circle(teenSizeField,Point(520,370) ,75, cv::Scalar( 0, 255, 0 ), 1,1);	



	//X
	theLandmarks.push_back(Point(520,295));  landmarkType.push_back(field_model::WorldObject::Type_LineXingX); //the code of the X
	theLandmarks.push_back(Point(520,445));  landmarkType.push_back(field_model::WorldObject::Type_LineXingX); //the code of the X
	
	
	//L
	theLandmarks.push_back(Point(70,70));    landmarkType.push_back(field_model::WorldObject::Type_LineXingL); //the code of the L
	theLandmarks.push_back(Point(70,670));   landmarkType.push_back(field_model::WorldObject::Type_LineXingL); //the code of the L
	theLandmarks.push_back(Point(170,145));  landmarkType.push_back(field_model::WorldObject::Type_LineXingL); //the code of the L
    theLandmarks.push_back(Point(170,595));  landmarkType.push_back(field_model::WorldObject::Type_LineXingL); //the code of the L
    theLandmarks.push_back(Point(870, 140)); landmarkType.push_back(field_model::WorldObject::Type_LineXingL); //the code of the L
    theLandmarks.push_back(Point(870,595));  landmarkType.push_back(field_model::WorldObject::Type_LineXingL); //the code of the L
	theLandmarks.push_back(Point(970,70));   landmarkType.push_back(field_model::WorldObject::Type_LineXingL); //the code of the L 
    theLandmarks.push_back(Point(970,670));  landmarkType.push_back(field_model::WorldObject::Type_LineXingL); //the code of the L
	
	//T
	theLandmarks.push_back(Point(520,670));  landmarkType.push_back(field_model::WorldObject::Type_LineXingT); //the code of the T
	theLandmarks.push_back(Point(520,70));   landmarkType.push_back(field_model::WorldObject::Type_LineXingT); //the code of the T
	theLandmarks.push_back(Point(70,145));	  landmarkType.push_back(field_model::WorldObject::Type_LineXingT); //the code of the T
	theLandmarks.push_back(Point(70,595));	  landmarkType.push_back(field_model::WorldObject::Type_LineXingT); //the code of the T
    theLandmarks.push_back(Point(970, 140)); landmarkType.push_back(field_model::WorldObject::Type_LineXingT); //the code of the T
    theLandmarks.push_back(Point(970,595));  landmarkType.push_back(field_model::WorldObject::Type_LineXingT); //the code of the T

	
	//GOAL POSTs
	theLandmarks.push_back(Point(70,240));   landmarkType.push_back(field_model::WorldObject::Type_GoalPost); //the code of the Post 
	theLandmarks.push_back(Point(70,500));   landmarkType.push_back(field_model::WorldObject::Type_GoalPost); //the code of the Post 
	theLandmarks.push_back(Point(970,240));  landmarkType.push_back(field_model::WorldObject::Type_GoalPost); //the code of the Post 
	theLandmarks.push_back(Point(970,500));  landmarkType.push_back(field_model::WorldObject::Type_GoalPost); //the code of the Post 

	
	//Penalty landmarks
	theLandmarks.push_back(Point(760,370));  landmarkType.push_back(field_model::WorldObject::Type_XMarker); //the code of the Post 
	theLandmarks.push_back(Point(280,370));  landmarkType.push_back(field_model::WorldObject::Type_XMarker); //the code of the Post 

	

	
	for(unsigned int i=0; i<theLandmarks.size();i++){		
		isLandmarkInsight.push_back(false);	
	}	
	
    Mat orgFieldDrawing = teenSizeField.clone();


	
	
   int ROBOT_x_pos = 500;
   int ROBOT_y_pos = 60;
   double ROBOT_theta = 0;
   
   
   
    ros::Publisher observations_pub =  particleTesterNode.advertise<soccer_vision::Detections>("/vision/detections", 1);
    soccer_vision::Detections msg;
	soccer_vision::ObjectDetection objs;

	msg.header.frame_id = "/ego_floor";

   
	
	//while(true){
	while ( particleTesterNode.ok()) {

		
		    ros::spinOnce();
			teenSizeField = orgFieldDrawing.clone();

				
			for(unsigned int i=0; i<isLandmarkInsight.size();i++){		
				isLandmarkInsight[i] = false;
			}
				
				
			char key=cvWaitKey(35);
			if(key==27){break;}//Exit the program


			switch (key)

			{

			case 'u': // Up arrow
				//cerr << "up\n";
				if (checkFieldLimits(ROBOT_x_pos, ROBOT_y_pos+1)){
				ROBOT_y_pos += 1;
				}		
			break;

			case 'j': // down arrow
				//cerr << "down\n";
				if (checkFieldLimits(ROBOT_x_pos, ROBOT_y_pos-1)){
				ROBOT_y_pos -= 1;
				}
			break;

			case 'h': // left arrow
				//cerr << "left\n";
				if (checkFieldLimits(ROBOT_x_pos-1, ROBOT_y_pos)){
				ROBOT_x_pos -= 1;
				}
			break;

			case 'k': // right arrow
				//cerr << "right\n";
				if (checkFieldLimits(ROBOT_x_pos+1, ROBOT_y_pos)){
				ROBOT_x_pos += 1;
				}
			break;

			case 'e': // rotate to the right
				
				ROBOT_theta =  checkAngleLimits(ROBOT_theta+1);		
						//cerr << "w\t" << ROBOT_theta << endl;
			break;

			case 'w': // rotate to the left
				
				ROBOT_theta =  checkAngleLimits(ROBOT_theta-1);		
						//cerr << "e\t" << ROBOT_theta << endl;
			break;

			}



			double theAngle = (ROBOT_theta/360.f)*6.28318530718;


			//generating the rotation Matrix
			rot_data[0] = cos(theAngle);
			rot_data[1] = -sin(theAngle);
			rot_data[2] = sin(theAngle);
			rot_data[3] = cos(theAngle);

			Mat rot_mat = Mat(2, 2, CV_32FC1, &rot_data);

			Mat A, At, R, Rt, rotA;    
			A = Mat(3, 2, CV_32FC1, &data);

			transpose(A,At);

			R = rot_mat*At;    

			R.row(0) = R.row(0) + ROBOT_x_pos ;
			R.row(1) = R.row(1) + ROBOT_y_pos ;

			transpose(R,Rt);

			theLandmarksAfterRandomSelection.clear();
			contours.clear();
			
			contours.push_back(Point( Rt.at<float>(0,0), Rt.at<float>(0,1)));
			contours.push_back(Point( Rt.at<float>(1,0), Rt.at<float>(1,1)));
			contours.push_back(Point( Rt.at<float>(2,0), Rt.at<float>(2,1)));
			
			Point robotsPosition = Point( Rt.at<float>(0,0), Rt.at<float>(0,1));
			Point pointInTheMiddle  = ( Point( Rt.at<float>(1,0), Rt.at<float>(1,1)) + Point( Rt.at<float>(2,0), Rt.at<float>(2,1)) )*0.5;
			
			line( teenSizeField,robotsPosition, pointInTheMiddle, Scalar(0,255,255) , 1, 1 );

			for(unsigned  int i=0; i<theLandmarks.size();i++){					
				int insideornot = pointPolygonTest(contours, theLandmarks[i], false);				
				
				//[0,50] = true
				//[51,100] = false
				int a_random_number  = rand() % 100 + 1;
				bool use_the_landmark_insight = true;				
				if (a_random_number>50){use_the_landmark_insight = false;}
				
				if (insideornot>0 && use_the_landmark_insight){isLandmarkInsight[i] = true;}
			}	

			
			
			msg.lines.clear();
			msg.objects.clear();
			msg.header.stamp = ros::Time::now();
			
			for(unsigned  int i=0; i<isLandmarkInsight.size();i++){
			
				
				if (isLandmarkInsight[i] == true){
					circle(teenSizeField,theLandmarks[i], 5, Scalar( 0, 0, 255 ), -1, 8);
					double distanceToLine = distance_to_line(robotsPosition, pointInTheMiddle, theLandmarks[i]);
					double distanceToLandmark = distance_to_point(robotsPosition, theLandmarks[i]);
					double distanceToPointOnLine = sqrt( (distanceToLandmark*distanceToLandmark) - (distanceToLine*distanceToLine) );
					
					
					landmarkData lm;
					lm.x = distanceToPointOnLine;
					lm.y = distanceToLine;
					lm.z = 0;
					lm.type = landmarkType[i];					
					theLandmarksAfterRandomSelection.push_back(lm);

					objs.confidence = 1.0f;
					objs.type = landmarkType[i];
					objs.pose.theta = 0.0f;
					objs.pose.x = distanceToPointOnLine/100.0f;
					objs.pose.y = distanceToLine/100.0f;
					msg.objects.push_back(objs);
				}
				
			}
			
			observations_pub.publish(msg);
			
			cerr << "Selected landmarks: "<< theLandmarksAfterRandomSelection.size() << endl;
			
			drawFieldOfView(teenSizeField,Rt);
			theRobot(teenSizeField, Point(ROBOT_x_pos,ROBOT_y_pos), ROBOT_theta);
			imshow( "Soccer Field", teenSizeField );
			
   }//End while loop
   
   

    return 0;
}//END Program
