// Soccer Vision node
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

#ifndef SOCCER_MARKERS_H
#define SOCCER_MARKERS_H

#include <string>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <soccer_vision/Detections.h>
#include <field_model/field_model.h>


class soccermarker{
	
	public:
	visualization_msgs::Marker marker;
	std::string theFrameId;
	ros::Duration markerLifeTime;
	soccermarker(){
		theFrameId = "/ego_floor";
		markerLifeTime = ros::Duration(0.2);
	}
	~soccermarker(){}
	
	visualization_msgs::Marker generate_ball_marker(ros::Time time,int id, float pos_x, float pos_y, float pos_z){

		marker.header.frame_id = theFrameId.c_str();
		marker.header.stamp = time;
		marker.ns = "ball";
		marker.id = id;
		marker.type = visualization_msgs::Marker::SPHERE; // Setting initial shape type to be a Sphere
		marker.action = visualization_msgs::Marker::ADD;
		
		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = pos_x;
		marker.pose.position.y = pos_y;
		marker.pose.position.z = pos_z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		//the size of the marker
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;
		
		//the color (in this case orange)
		marker.color.r = 0.88235f;
		marker.color.g = 0.19608f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = markerLifeTime;
		
		return marker;
	}

	visualization_msgs::Marker generate_goalpost_marker(ros::Time time,int id, float pos_x, float pos_y, float pos_z){

		marker.header.frame_id = theFrameId.c_str();
		marker.header.stamp = time;
		marker.ns = "post";
		marker.id = id;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		
		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = pos_x;
		marker.pose.position.y = pos_y;
		marker.pose.position.z = pos_z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		//the size of the marker
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;
		
		//the color (in this case yellow)
		marker.color.r = 1.f;
		marker.color.g = 1.f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = markerLifeTime;
		
		return marker;
	}
		

	visualization_msgs::Marker generate_Xmarker(ros::Time time,int id, float pos_x, float pos_y, float pos_z){

		marker.header.frame_id = theFrameId.c_str();
		marker.header.stamp = time;
		marker.ns = "xmark";
		marker.id = id;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		
		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = pos_x;
		marker.pose.position.y = pos_y;
		marker.pose.position.z = pos_z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		//the size of the marker
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;
		
		//the color (in this case red)
		marker.color.r = 1.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = markerLifeTime;
		
		return marker;
	}		

	visualization_msgs::Marker generate_Tmarker(ros::Time time,int id, float pos_x, float pos_y, float pos_z){

		marker.header.frame_id = theFrameId.c_str();
		marker.header.stamp = time;
		marker.ns = "tmark";
		marker.id = id;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		
		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = pos_x;
		marker.pose.position.y = pos_y;
		marker.pose.position.z = pos_z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		//the size of the marker
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;
		
		//the color (in this case red)
		marker.color.r = 0.0f;
		marker.color.g = 0.0f;
		marker.color.b = 1.0f;
		marker.color.a = 1.0;

		marker.lifetime = markerLifeTime;
		
		return marker;
	}

	visualization_msgs::Marker generate_Lmarker(ros::Time time,int id, float pos_x, float pos_y, float pos_z){

		marker.header.frame_id = theFrameId.c_str();
		marker.header.stamp = time;
		marker.ns = "lmark";
		marker.id = id;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		
		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = pos_x;
		marker.pose.position.y = pos_y;
		marker.pose.position.z = pos_z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		//the size of the marker
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;
		
		//the color (in this case red)
		marker.color.r = 0.5f;
		marker.color.g = 0.5f;
		marker.color.b = 0.5f;
		marker.color.a = 1.0;

		marker.lifetime = markerLifeTime;
		
		return marker;
	}		

	visualization_msgs::Marker generate_shoulder_marker_ok(ros::Time time,int id, float pos_x, float pos_y, float pos_z){

		marker.header.frame_id = "/trunk_link";
		marker.header.stamp = time;
		marker.ns = "lmark";
		marker.id = id;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		
		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = pos_x;
		marker.pose.position.y = pos_y;
		marker.pose.position.z = pos_z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		//the size of the marker
		marker.scale.x = 0.02;
		marker.scale.y = 0.02;
		marker.scale.z = 0.02;
		
		//the color (in this case red)
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 1.0f;
		marker.color.a = 1.0;

		marker.lifetime = markerLifeTime;
		
		return marker;
	}
	
	visualization_msgs::Marker generate_shoulder_marker_not_ok(ros::Time time,int id, float pos_x, float pos_y, float pos_z){

		marker.header.frame_id = "/trunk_link";
		marker.header.stamp = time;
		marker.ns = "lmark";
		marker.id = id;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		
		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = pos_x;
		marker.pose.position.y = pos_y;
		marker.pose.position.z = pos_z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		//the size of the marker
		marker.scale.x = 0.02;
		marker.scale.y = 0.02;
		marker.scale.z = 0.02;
		
		//the color (in this case red)
		marker.color.r = 1.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = markerLifeTime;
		
		return marker;
	}	
	
	
};


#endif