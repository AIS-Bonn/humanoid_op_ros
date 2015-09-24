// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

#ifndef GOALINFORMATIONBAG_H
#define GOALINFORMATIONBAG_H


#include <iostream>
#include <string>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <ros/console.h>
#include "convexhullfunctions.h"

using namespace std;
using namespace soccervision;


#define MAX_NUMBER_OF_HULL_POINTS 250
#define TOTAL_NUMBER_OF_YELLOW_SECTIONS 20 // TODO: Change back to 10 if you feel this is too large, but why bother?

class GoalHullContainer
{
private:
      int top;  	
      HullPoint arr[MAX_NUMBER_OF_HULL_POINTS];
            
  
public:

	GoalHullContainer(){top=-1;}
	~GoalHullContainer(){}

        void clear(){
			top=-1; //empty the stack
        }

	bool push(HullPoint & data){
		if (top<MAX_NUMBER_OF_HULL_POINTS && top>=-1){
		    top++;
		    arr[top]=data;
		    return true;
		}
		else{
		    ROS_WARN_THROTTLE(0.5, "Goal hull container stack is full");
		    return false;
		}
	}

	//----------------------------------------
	bool pop(HullPoint & data){

		if (top==-1){
			 ROS_WARN_THROTTLE(0.5, "Cannot pop an empty goal hull container stack");
		 return false;
		}
		else{
			data = arr[top];
			top--;
			return true;
		}
	}
	//----------------------------------------

	bool empty(){
		 return (top==-1);
	}
	//----------------------------------------

	int size(){return top+1;}

	//----------------------------------------
	bool mean(double & meanx, double & meany){

		meanx=0;
		meany=0;

		if (top==-1)
			return false;

		for(int k=0;k<=top;k++){
				meanx += arr[k].x;
				meany += arr[k].y;
		}
			meanx = meanx/(top+1);
			meany = meany/(top+1);

		return true;
	}
	//----------------------------------------

	bool minpoint(int & minx,int & miny){

		minx=0;
		miny=0;


		if (top==-1)
			return false;

		minx = arr[0].x;
		miny = arr[0].y;

		for(int k=0;k<=top;k++){

	 		minx = min(arr[k].x,minx);
	 		miny = min(arr[k].y,miny);

		}
		return true;
	}

	//----------------------------------------
	bool maxpoint(int & maxx,int & maxy){

		maxx=0;
		maxy=0;


		if (top==-1)
			return false;

		maxx = arr[0].x;
		maxy = arr[0].y;

		for(int k=0;k<=top;k++){

	 		maxx = max(arr[k].x,maxx);
	 		maxy = max(arr[k].y,maxy);

		}
		return true;
	}
	
	//----------------------------------------
	//No size check is made
	HullPoint getPixelAtPos(int i){
		return arr[i];
	}
	
};


class PossibleGoalSections{
  

 
      
      public: // TODO: MOVE THIS TO AFTER THE NEXT LINE
      int regionCounter;	
      GoalHullContainer goalSections[TOTAL_NUMBER_OF_YELLOW_SECTIONS];
  
      PossibleGoalSections(){
	regionCounter = -1;
      }
      ~PossibleGoalSections(){}      
  
  
      void reset(){
	
	  regionCounter = -1;
	  
 	  for (int i=0; i<TOTAL_NUMBER_OF_YELLOW_SECTIONS;i++){
 	    goalSections[i].clear();
 	  }		
      }
      
      void incrementSectionCounter(){ // TODO: MAKE SURE YOU CALL THIS FROM EVERYWHERE INSTEAD OF MANUALLY INCREMENTING
	 
	  if (regionCounter<(TOTAL_NUMBER_OF_YELLOW_SECTIONS-1))
	  regionCounter++;	  	  
      }

      int getSectionCounter(){	 	  
	  return regionCounter+1;	  	  
      }  
  
  
};


#endif
