// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

#ifndef BALLINFORMATIONBAG_H
#define BALLINFORMATIONBAG_H

#include <iostream>
#include <string>
#include <iostream>
#include <fstream>
#include <string>


#include <tf/transform_datatypes.h>

using namespace std;



class BallInformationBag
{
  public:
    
	bool ballFoundOnTheField;
	double ego_ball_distance;
	int  radiusA;
	int  radiusB;
	int pos_x, pos_y;
	unsigned pixelsize;
	tf::Vector3 ego_ball_vector;
	tf::Vector3 camera_world_vector;
    
	BallInformationBag(){	  
		reset();
	};

	~BallInformationBag()
	{}

	void reset(){	  
	  ballFoundOnTheField = false;
	  ego_ball_distance = 1000; // Setting the value to a greater than expected world distance
	  radiusA = 0;
	  radiusB = 0;
	  pixelsize = 0;
	  pos_x = pos_y = 0;
	  ego_ball_vector.setValue(0,0,0);
	  camera_world_vector.setValue(0,0,0);
	};
  
  
	void operator=(const BallInformationBag BALL){
		  ballFoundOnTheField = BALL.ballFoundOnTheField;
		  ego_ball_distance = BALL.ego_ball_distance;
		  radiusA = BALL.radiusA;
		  radiusB = BALL.radiusB;
		  pos_x = BALL.pos_x;
		  pos_y = BALL.pos_y;
		  pixelsize = BALL.pixelsize;
		  ego_ball_vector = BALL.ego_ball_vector;
		  camera_world_vector = BALL.camera_world_vector;
	}	
};


#endif
