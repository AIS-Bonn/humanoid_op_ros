// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

#ifndef FILTERUSINGSHOULDERPLANE_H
#define FILTERUSINGSHOULDERPLANE_H


#include <iostream>
#include <string>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <boost/concept_check.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

//TF includes
#include <tf/transform_datatypes.h>
#include "pixelCameraCorrection.h"
#include "camera_parameters.h"
#include "globaldefinitions.h"

namespace soccervision{

//RETURNS false if the landmark is consider NOT VALID
//RETURNS true if the landmark is consider VALID (not on the robot)
inline bool filterTheLandmarkUsingShoulderPlane(tf::Matrix3x3 M, tf::Vector3 v, float x_filter_range, float y_filter_range, int pixel_x, int pixel_y){
	
	tf::Vector3 tmpvec;
	tmpvec = pixelCameraCorrection(pixel_x,pixel_y);
	tmpvec = compute_bodymark_ego_position(M,v,tmpvec);
	
	if (tmpvec.x()< x_filter_range && fabs(tmpvec.y())<y_filter_range){
		return false;
	}
	
	return true;
}

}//End namespace

#endif