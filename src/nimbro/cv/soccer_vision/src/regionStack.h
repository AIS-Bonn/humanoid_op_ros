// Soccer Vision node
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

#ifndef REGIONSTACK_H
#define REGIONSTACK_H


#include <iostream>
#include <string>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

#include <ros/console.h>

#include "globaldefinitions.h"



using namespace std;
//--------------------------------------------------------------------



class PixelPosition
{
   public:
	int x;
	int y;
	PixelPosition(){}
	PixelPosition(int vx, int vy){ x=vx; y=vy;}
	~PixelPosition(){}

	void set(int vx, int vy){ x=vx; y=vy;}

	void operator=(const PixelPosition d){
		  x = d.x;
		  y = d.y;
	}
	
	bool operator <(const PixelPosition &p) const {
		return x < p.x || (x == p.x && y < p.y);
	}	
};

//--------------------------------------------------------------------


//--------------------------------------------------------------------
class regionData
{
   public:

		bool valid;
		int min_x, min_y;
		int max_x, max_y;
		double meanx, meany;

	regionData(){
		valid=false;
		min_x = min_y = 0;
		max_x = max_y = 0;
		meanx = meany = 0.0f;	  	  
	}
	~regionData(){}
	
	void initialize(){
		valid=false;
		min_x = min_y = 0;
		max_x = max_y = 0;
		meanx = meany = 0.0f;
	}
	
	void operator=(const regionData d){
		valid = d.valid;
		min_x = d.min_x;
		min_y = d.min_y;
		max_x = d.max_x;
		max_y = d.max_y;
		meanx = d.meanx;
		meany = d.meany;
	}
};
//--------------------------------------------------------------------


//--------------------------------------------------------------------
class covMatrix
{
   public:
	bool valid;
	double xx;
	double xy;
	double yx;
	double yy;
	covMatrix(){
		valid = false;
		xx = xy = yx = yy = 0.0f;
	}
	~covMatrix(){}

	void initialize(){
		valid = false;
		xx = xy = yx = yy = 0.0f;
	}
	void set(bool vvalid, double vxx, double vxy, double vyx, double vyy){
		valid = vvalid;
		xx = vxx;
		xy = vxy;
		yx = vyx;
		yy = vyy;
	}

	void operator=(const covMatrix d){
		valid = d.valid;
		xx = d.xx;
		xy = d.xy;
		yx = d.yx;
		yy = d.yy;
	}


};
//--------------------------------------------------------------------


//--------------------------------------------------------------------
class regionStack{

public:
	
    PixelPosition arr[MAX_STACK_SIZE];
    int top;


    regionStack(){top=-1;}
    ~regionStack(){}

		void clear(){
			top=-1; //empty the stack
		}

		bool push(PixelPosition i){
		     if (top<MAX_STACK_SIZE){
		    	 top++;
		         arr[top]=i;
		         return true;
		     }
		     else{
		    	 ROS_WARN_THROTTLE(0.5, "A region stack is full");
		    	 return false;
		     }
		}

	//----------------------------------------
	bool pop(PixelPosition & data){

		if (top==-1){
			 ROS_WARN_THROTTLE(0.5, "Cannot pop an empty region stack");
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
	bool mean(double & meanx,double & meany){

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

	void fulldata(regionData &A){
	  
		A.initialize();  

		
		if (top==-1)
		    return;

		A.max_x = arr[0].x;
		A.max_y = arr[0].y;
		A.min_x = arr[0].x;
		A.min_y = arr[0].y;
		A.meanx = 0.0f;
		A.meany = 0.0f;	

		for(int k=0;k<=top;k++){

	 		A.max_x = max(arr[k].x,A.max_x);
	 		A.max_y = max(arr[k].y,A.max_y);
	 		A.min_x = min(arr[k].x,A.min_x);
	 		A.min_y = min(arr[k].y,A.min_y);	
			
			A.meanx += arr[k].x;
			A.meany += arr[k].y;

		}		
		
		A.meanx = A.meanx/(top+1);
		A.meany = A.meany/(top+1);
		A.valid = true;
	
		return;
	}
	
	//----------------------------------------
	covMatrix covariance(const regionData A){

		// assumption stack is not empty
		// it also has to have a minimum size (e.g. 2)
		covMatrix C;
		C.initialize();

		if (A.valid == false) {return C;}

		double meanx  = A.meanx;
		double meany  = A.meany;
		

		double valx=0;
		double valy=0;
		double covxx=0;
		double covxy=0;
		double covyy=0;

		for(int k=0;k<=top;k++){
				valx = arr[k].x - meanx;
				valy = arr[k].y - meany;

				covxx += valx*valx;
				covxy += valx*valy;
				covyy += valy*valy;
		}

		C.set(true, covxx, covxy, covxy, covyy);
		return C;
	}	
	
	
	//----------------------------------------
	//No size check is made
	PixelPosition getPixelAtPos(int i){
		return arr[i];
	}

	
};
//--------------------------------------------------------------------

#endif
