// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

#ifndef FINDBODYMASK_H
#define FINDBODYMASK_H

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

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "globaldefinitions.h"
#include "checkLimitsInSubImage.h"
#include "frameGrabber.h"
#include "convexhullfunctions.h"
#include "regionStack.h"
#include "pixelCameraCorrection.h"
#include "camera_parameters.h"

class FindBodyMask
{
	
	public:
		
		struct RobotCorner
		{
			RobotCorner(const tf::Vector3& _pos, const tf::Vector3& _dir1, const tf::Vector3& _dir2)
			 : pos(_pos), direction_pred(_dir1), direction_succ(_dir2)
			{
			}
			
			RobotCorner()
			{}

			tf::Vector3 pos;
			tf::Vector3 direction_pred;
			tf::Vector3 direction_succ;
		};
		
		std::vector<RobotCorner> m_points;
		std::vector<RobotCorner> m_projectedPoints;
		
		FindBodyMask(){
			resetPoints();
		}
		
		~FindBodyMask(){}    

		void resetPoints(){
			
			m_points.clear();
			
			tf::Vector3 imu_to_center(-0.0189999999999999, 0.0, 0.0);
			
			const double X_DIM = 0.09;
			const double Y_DIM = 0.14;
			const double Z_DIM = 0.10;
			
			m_points.push_back(RobotCorner(imu_to_center + tf::Vector3(-X_DIM,+Y_DIM,+Z_DIM), tf::Vector3(0, -0.001, 0), tf::Vector3(0.001, 0, 0)));
			m_points.push_back(RobotCorner(imu_to_center + tf::Vector3(+X_DIM,+Y_DIM,+Z_DIM), tf::Vector3(-0.001, 0, 0), tf::Vector3(0, -0.001, 0)));
			m_points.push_back(RobotCorner(imu_to_center + tf::Vector3(+X_DIM,-Y_DIM,+Z_DIM), tf::Vector3(0, 0.001, 0), tf::Vector3(-0.001, 0, 0)));
			m_points.push_back(RobotCorner(imu_to_center + tf::Vector3(-X_DIM,-Y_DIM,+Z_DIM), tf::Vector3(0.001, 0, 0), tf::Vector3(0.0, 0.001, 0)));
		}
		
		void executeProjectionToCameraImage(const tf::Vector3 offset, const tf::Matrix3x3 rotationmatrix){
			
			m_projectedPoints.resize(4);
			for(size_t i = 0; i < m_points.size(); ++i)
			{
				const RobotCorner& corner = m_points[i];
				RobotCorner out;
				
				tf::Vector3 cam_point = rotationmatrix.transpose() * (corner.pos - offset);
				out.pos = cameraPixelCorrectionOriginalSize(cam_point);
				
				tf::Vector3 dir_pred = cam_point + rotationmatrix.transpose() * corner.direction_pred;
				out.direction_pred = cameraPixelCorrectionOriginalSize(dir_pred) - out.pos;

				tf::Vector3 dir_succ = cam_point + rotationmatrix.transpose() * corner.direction_succ;
				out.direction_succ = cameraPixelCorrectionOriginalSize(dir_succ) - out.pos;

				m_projectedPoints[i] = out;
			}
		}
		
		
};

#endif