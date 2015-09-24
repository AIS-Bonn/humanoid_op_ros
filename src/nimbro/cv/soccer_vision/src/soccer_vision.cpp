// Soccer Vision node
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>
/**
* @file soccer_vision/src/soccer_vision.cpp
* @brief Soccer Vison Code. 
* @author Julio Pastrana <pastrana@ais.uni-bonn.de>
* 
* 
* In this loop all the fucntions that will take care of detecting objects on the field will be called.
**/


#include "soccer_vision.h"
#include "filterUsingShoulderPlane.h"

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/make_shared.hpp>
#include <boost/foreach.hpp>

using namespace soccervision;

PLUGINLIB_EXPORT_CLASS(SoccerVision, nodelet::Nodelet)

SoccerVision::SoccerVision()
{
}

SoccerVision::~SoccerVision()
{
}


void SoccerVision::onInit()
{
	ros::NodeHandle& nh = getNodeHandle();


	m_sub_input.subscribe(nh,"image",10);
	m_tf_filter = new tf::MessageFilter<sensor_msgs::Image> (m_sub_input,m_tf, "ego_rot", 10);

	m_pub_output = nh.advertise<sensor_msgs::Image>("classes", 10);
	
	//Definition of the detected objects publisher
	observations_pub =  nh.advertise<soccer_vision::Detections>("/vision/detections", 1);
	msg.header.frame_id = "/ego_floor";
	
	 
	

	ball_detected_camera_vector_pub = nh.advertise<geometry_msgs::PointStamped>("ball/focal_plane_ball_vector", 5);
	ball_detected_camera_vector.header.frame_id = "/camera_optical"; 
	
	ball_detected_ego_vector_pub = nh.advertise<geometry_msgs::PointStamped>("ball/ego_rot_ball_vector", 5);
	ball_detected_ego_vector.header.frame_id = "/ego_rot";

	
	detected_objects_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization/object_markers",1);
	
	detected_obstacles_markers_pub = nh.advertise<visualization_msgs::Marker>("visualization/obstacle_markers", 1);
	detected_obstacles_line_list.header.frame_id = "/ego_floor";
	detected_obstacles_line_list.type = visualization_msgs::Marker::LINE_LIST;
	
	NODELET_INFO("Starting Nimbro-OP vision node");	
	CameraFrame.currentLUT.initialize();
	FieldFinder.initialize();
	

	NODELET_INFO("Reading Camera mask and Look-up tables...");	
	CameraFrame.currentLUT.ReadYUVBlackLookUpTable("Black.lut"); //ID 0
	CameraFrame.currentLUT.ReadYUVWhiteLookUpTable("White.lut"); //ID 1
	CameraFrame.currentLUT.ReadYUVLookUpTable("Objects.lut");    //ID 2 3 4 ...	
	//CameraFrame.readCameraMask("maskForCamera");
	memset(CameraFrame.cameraMask, static_cast<unsigned char>(1), sizeof(CameraFrame.cameraMask));

	ros::NodeHandle("~").param("use_gui", m_use_gui, false);
	
	m_param = new Parameters();
	
	//START Image Publishers for the SoccerGUI
	it = new image_transport::ImageTransport(nh);	
	image_pub_ = it->advertise("image/rgb", 1);
	image_pub0_ = it->advertise("image/segmented", 1);
	image_pub1_ = it->advertise("image/smallimg1", 1);
	image_pub2_ = it->advertise("image/smallimg2", 1);
	image_pub3_ = it->advertise("image/smallimg3", 1);
	image_pub4_ = it->advertise("image/smallimg4", 1);
	image_pub5_ = it->advertise("image/smallimg5", 1);
	image_pub6_ = it->advertise("image/smallimg6", 1);
	image_pub7_ = it->advertise("image/smallimg7", 1);
	//END Image Publishers for the SoccerGUI

	m_tf_filter->registerCallback(boost::bind(&SoccerVision::processImage, this, _1));
}


void SoccerVision::processImage(const sensor_msgs::Image::ConstPtr& img)
{
  
	//All the detected objects will be time-stamped with this value
	ros::Time this_frame_time_stamp = ros::Time::now();
	
	//clear the array containing the visualization markers
	detected_objects_counter = 0;
	detected_obstacles_counter = 0;
	detected_objects_array.markers.clear();
	//landmarks_on_shoulder_array.markers.clear();

	//Publishing the landmarks and lines information 
	//Clean the vectors that contain the detected objects
	//for further population
	msg.lines.clear();
	msg.objects.clear();
	msg.obstacles.clear();
	msg.header.stamp = this_frame_time_stamp;
	
	//Preparing obtacle markers
	detected_obstacles_line_list.header.stamp = this_frame_time_stamp;
	detected_obstacles_line_list.color.r = 0.0;
	detected_obstacles_line_list.color.g = 0.0;
	detected_obstacles_line_list.color.b = 0.0;
	detected_obstacles_line_list.color.a = 1.0;
	detected_obstacles_line_list.action = visualization_msgs::Marker::ADD;
	detected_obstacles_line_list.id = 0;
	detected_obstacles_line_list.pose.orientation.w = 1;
	detected_obstacles_line_list.scale.x = 0.1;
	
	detected_obstacles_line_list.points.clear();
	
	tf::StampedTransform transform_egorot_cameraoptical;
	tf::StampedTransform transform_trunk_cameraoptical;

			try{

				tf::Matrix3x3 mat;
				mat.setEulerYPR(0.0, -0.055, 0.0);
				
				m_tf.lookupTransform("/ego_rot","/camera_optical",img->header.stamp, transform_egorot_cameraoptical);
				CameraFrame.tf_egorot_cameraoptical_OriginVector =  transform_egorot_cameraoptical.getOrigin();
				CameraFrame.tf_egorot_cameraoptical_OriginBasis =  mat * transform_egorot_cameraoptical.getBasis();
				
				m_tf.lookupTransform("/trunk_link","/camera_optical",img->header.stamp, transform_trunk_cameraoptical);				
				CameraFrame.tf_trunk_cameraoptical_OriginBasis  = mat * transform_trunk_cameraoptical.getBasis();
				CameraFrame.tf_trunk_cameraoptical_OriginVector = transform_trunk_cameraoptical.getOrigin();

				CameraFrame.x_shoulder_filter_range = m_param->landmark_filter_dist_x();
				CameraFrame.y_shoulder_filter_range = m_param->landmark_filter_dist_y();
				
				
				//TODO: fine-tuning required
				//CameraFrame.robotBodyToImage.executeProjectionToCameraImage(transform_trunk_cameraoptical.getOrigin(), transform_trunk_cameraoptical.getBasis());
			}
			catch (tf::TransformException &ex){
				ROS_ERROR("%s",ex.what());
			}	

			int pointercounter = 0;
			int pointercounterClass = 0;
			unsigned char Y1, U , Y2, V;


			
			// ************************* //
			// * 1. Color Segmentation * //
			// ************************* //
			
			memset(CameraFrame.classified_data, CC_NO_CLASS, sizeof(unsigned char)*ORG_IMAGE_WIDTH*ORG_IMAGE_HEIGHT);

			//Classifying each pixel into object classes using Look-up tables
			//Black, White, Field (green), Ball (orange), Goal (yellow)
			//check: classifyYUVFrameIntoColors	/**
			for (unsigned int i = 0; i<(img->width*img->height*2); i+=4){

				Y1 = (unsigned char)img->data[i];
				U  =  (unsigned char)img->data[i+1];
				Y2 = (unsigned char)img->data[i+2];
				V  =  (unsigned char)img->data[i+3];
					
				//Storing the YUV data 
				CameraFrame.frame_raw_data[pointercounter++] = Y1;
				CameraFrame.frame_raw_data[pointercounter++] = U;
				CameraFrame.frame_raw_data[pointercounter++] = V;

				//Classifying the YUV value				
				CameraFrame.classified_data[pointercounterClass++] = (unsigned char)CameraFrame.currentLUT.classifyYUVFrameIntoColors(Y1,U,V);
				

				//Storing the YUV data 
				CameraFrame.frame_raw_data[pointercounter++] = Y2;
				CameraFrame.frame_raw_data[pointercounter++] = U;
				CameraFrame.frame_raw_data[pointercounter++] = V;
				
				//Classifying  the YUV value	
				CameraFrame.classified_data[pointercounterClass++] = (unsigned char)CameraFrame.currentLUT.classifyYUVFrameIntoColors(Y2,U,V);		     
			}//END for

			
			

			// ************************* //
			// * 2. Subsampling images * //
			// ************************* //

			//Every subsampled image will be stored in the object
			//currentLUT.objects[COLOR_CLASS].subimg

			yuvImageSubsampling(CameraFrame.classified_data, CameraFrame.currentLUT.objects, CameraFrame.cameraMask);


			// ***************************** //
			// * 3. Find Field             * //
			// ***************************** //


			FieldFinder.findField(CameraFrame);



			// ***************************** //
			// * 4. Find orange ball       * //
			// ***************************** //
			BallFinder.findBall(CameraFrame);

				
				//If found Publish the detected BALL
				if(CameraFrame.orangeBallBag.ballFoundOnTheField){
					
					ball_detected_camera_vector.header.stamp = this_frame_time_stamp;
					ball_detected_camera_vector.point.x = CameraFrame.orangeBallBag.camera_world_vector.x();
					ball_detected_camera_vector.point.y = CameraFrame.orangeBallBag.camera_world_vector.y();
					ball_detected_camera_vector.point.z = CameraFrame.orangeBallBag.camera_world_vector.z();
					
					ball_detected_ego_vector.header.stamp = this_frame_time_stamp;
					ball_detected_ego_vector.point.x = CameraFrame.orangeBallBag.ego_ball_vector.x();
					ball_detected_ego_vector.point.y = CameraFrame.orangeBallBag.ego_ball_vector.y();
					ball_detected_ego_vector.point.z = CameraFrame.orangeBallBag.ego_ball_vector.z();
					
					ball_detected_camera_vector_pub.publish(ball_detected_camera_vector);
					ball_detected_ego_vector_pub.publish(ball_detected_ego_vector);
					
					detected_objects_array.markers.push_back(objmarker.generate_ball_marker(this_frame_time_stamp,
																++detected_objects_counter,
																CameraFrame.orangeBallBag.ego_ball_vector.x(),
																CameraFrame.orangeBallBag.ego_ball_vector.y(),
																0.09
 																));
				}

			// ***************************** //
			// * 5. Find yellow goal       * //
			// ***************************** //
			
			GoalFinder.findGoal(CameraFrame);
			

			// ***************************** //
			// * 6. Find Obstacles         * //
			// ***************************** //	

			
			ObstacleFinder.findObstacles(CameraFrame);
			
				//Publishing obstacles
				for (unsigned int co=0; co<CameraFrame.DetectedObstacles.size(); co++){

					
					bool valid_obstacle = false;
					valid_obstacle = filterTheLandmarkUsingShoulderPlane(CameraFrame.tf_trunk_cameraoptical_OriginBasis,	
											     CameraFrame.tf_trunk_cameraoptical_OriginVector,
												0.07,
												0.12,
												(int)CameraFrame.DetectedObstacles[co].meanx*4,
												(int)CameraFrame.DetectedObstacles[co].meany*4);
					if(valid_obstacle==true){
					//Publish Valid Obstacle
					tf::Vector3 vecFromCameraToLandmark;
					tf::Vector3 vecEgoCoordinates;
					geometry_msgs::Point p1, p2;
					
					detected_obstacles_counter ++;
					
					vecFromCameraToLandmark = pixelCameraCorrection(CameraFrame.DetectedObstacles[co].max_x,CameraFrame.DetectedObstacles[co].max_y);
					vecEgoCoordinates = compute_ego_position(CameraFrame.tf_egorot_cameraoptical_OriginBasis, CameraFrame.tf_egorot_cameraoptical_OriginVector, vecFromCameraToLandmark);			
					detected_obstacle.left_lower_corner.x = vecEgoCoordinates.x();
					detected_obstacle.left_lower_corner.y = vecEgoCoordinates.y();
					detected_obstacle.left_lower_corner.z = vecEgoCoordinates.z();
					
					p1.x =  vecEgoCoordinates.x();
					p1.y =  vecEgoCoordinates.y();
					p1.z =  0.0;
										
					vecFromCameraToLandmark = pixelCameraCorrection(CameraFrame.DetectedObstacles[co].min_x,CameraFrame.DetectedObstacles[co].max_y);
					vecEgoCoordinates = compute_ego_position(CameraFrame.tf_egorot_cameraoptical_OriginBasis, CameraFrame.tf_egorot_cameraoptical_OriginVector, vecFromCameraToLandmark);			
					detected_obstacle.right_lower_corner.x = vecEgoCoordinates.x();	
					detected_obstacle.right_lower_corner.y = vecEgoCoordinates.y();
					detected_obstacle.right_lower_corner.z = vecEgoCoordinates.z();
					
					msg.obstacles.push_back(detected_obstacle);
					
					p2.x =  vecEgoCoordinates.x();
					p2.y =  vecEgoCoordinates.y();
					p2.z =  0.0;
					
					detected_obstacles_line_list.points.push_back(p1);
					detected_obstacles_line_list.points.push_back(p2);
					
					}
					
				}
	
			
			// ************************************* //
			// * 7. Find white landmarks and lines * //
			// ************************************* //

// 			ROS_INFO("Before findLandmarks_and_Lines!");
// 			MarksAndLinesFinder.findLandmarks_and_Lines(CameraFrame);
// 			ROS_INFO("After findLandmarks_and_Lines!");
			
			nimbroStyleLineFinder.findLines(CameraFrame);

				
				//Publishing Landmarks
				for (unsigned int co=0; co<CameraFrame.X_landmarks.size(); co++){
					
						tf::Vector3 vecFromCameraToLandmark = pixelCameraCorrection(CameraFrame.X_landmarks[co].x,CameraFrame.X_landmarks[co].y);
						tf::Vector3 vecEgoCoordinates = compute_ego_position(CameraFrame.tf_egorot_cameraoptical_OriginBasis, CameraFrame.tf_egorot_cameraoptical_OriginVector, vecFromCameraToLandmark);			
							
						objs.confidence = 1.0f;
						objs.type = field_model::WorldObject::Type_LineXingX;
						objs.pose.theta = 0.0f;
						objs.pose.x = vecEgoCoordinates.x();
						objs.pose.y = vecEgoCoordinates.y();
						msg.objects.push_back(objs);
						
						detected_objects_array.markers.push_back(objmarker.generate_Xmarker(this_frame_time_stamp,
											++detected_objects_counter,
											vecEgoCoordinates.x(),
											vecEgoCoordinates.y(),
											0.0
											));
				}

				
				for (unsigned int co=0; co<CameraFrame.T_landmarks.size(); co++){
					
						tf::Vector3 vecFromCameraToLandmark = pixelCameraCorrection(CameraFrame.T_landmarks[co].x,CameraFrame.T_landmarks[co].y);
						tf::Vector3 vecEgoCoordinates = compute_ego_position(CameraFrame.tf_egorot_cameraoptical_OriginBasis, CameraFrame.tf_egorot_cameraoptical_OriginVector, vecFromCameraToLandmark);			
					
						objs.confidence = 1.0f;
						objs.type = field_model::WorldObject::Type_LineXingT;
						objs.pose.theta = 0.0f;
						objs.pose.x = vecEgoCoordinates.x();
						objs.pose.y = vecEgoCoordinates.y();
						msg.objects.push_back(objs);
						
						detected_objects_array.markers.push_back(objmarker.generate_Tmarker(this_frame_time_stamp,
											++detected_objects_counter,
											vecEgoCoordinates.x(),
											vecEgoCoordinates.y(),
											0.0
											));
						
				}
				
				for (unsigned int co=0; co<CameraFrame.L_landmarks.size(); co++){
					
						tf::Vector3 vecFromCameraToLandmark = pixelCameraCorrection(CameraFrame.L_landmarks[co].x,CameraFrame.L_landmarks[co].y);
						tf::Vector3 vecEgoCoordinates = compute_ego_position(CameraFrame.tf_egorot_cameraoptical_OriginBasis, CameraFrame.tf_egorot_cameraoptical_OriginVector, vecFromCameraToLandmark);			

						objs.confidence = 1.0f;
						objs.type = field_model::WorldObject::Type_LineXingL;
						objs.pose.theta = 0.0f;
						objs.pose.x = vecEgoCoordinates.x();
						objs.pose.y = vecEgoCoordinates.y();
						msg.objects.push_back(objs);
						
						
						detected_objects_array.markers.push_back(objmarker.generate_Lmarker(this_frame_time_stamp,
											++detected_objects_counter,
											vecEgoCoordinates.x(),
											vecEgoCoordinates.y(),
											0.0
											));
				}	
				
				//Publishing yellow regions lower points
				for (int co=0; co<CameraFrame.goalLowerPoints.getSectionCounter(); co++){
						for (int hp=0; hp<CameraFrame.goalLowerPoints.goalSections[co].size();hp++){			  
							
						tf::Vector3 vecFromCameraToLandmark = pixelCameraCorrection(CameraFrame.goalLowerPoints.goalSections[co].getPixelAtPos(hp).x,
																					CameraFrame.goalLowerPoints.goalSections[co].getPixelAtPos(hp).y);
						
						tf::Vector3 vecEgoCoordinates = compute_ego_position(CameraFrame.tf_egorot_cameraoptical_OriginBasis, CameraFrame.tf_egorot_cameraoptical_OriginVector, vecFromCameraToLandmark);			
				
						objs.confidence = 1.0f;
						objs.type = field_model::WorldObject::Type_GoalPost;
						objs.pose.theta = 0.0f;
						objs.pose.x = vecEgoCoordinates.x();
						objs.pose.y = vecEgoCoordinates.y();
						msg.objects.push_back(objs);
						
						detected_objects_array.markers.push_back(objmarker.generate_goalpost_marker(this_frame_time_stamp,
											++detected_objects_counter,
											vecEgoCoordinates.x(),
											vecEgoCoordinates.y(),
											0.0
											));
				}
				}			

				//Publishing detected landmarks, goal posts
				//Only if the size of the vectors (Lines and Objects is greather than zero)
				if (msg.lines.size() > 0 || msg.objects.size()>0 || msg.obstacles.size()>0){
					observations_pub.publish(msg);
				}
				
				if (detected_objects_counter>0){
					detected_objects_markers_pub.publish(detected_objects_array);
				}
				
				if (detected_obstacles_counter>0){
					detected_obstacles_markers_pub.publish(detected_obstacles_line_list);
				}
			
			
			//EXECUTE only if GUI is required
			m_use_gui = true; // TODO: This is only a temporary measure to force the use of the GUI
			if(m_use_gui)
			{
				
				ros::Time this_frame_time_stamp = ros::Time::now();

				//LOCAL VISIALIZATION ON the RGB image
				unsigned char rgbFrame[800*600*3];
				yuv2rgb(CameraFrame.frame_raw_data, rgbFrame,800,600);
				cv::Mat fullrgbframe = cv::Mat(600,800,CV_8UC3,rgbFrame);
				
				//Drawing obstacles
				for (unsigned int co=0; co<CameraFrame.DetectedObstacles.size(); co++){

					
					bool valid_obstacle = false;
					valid_obstacle = filterTheLandmarkUsingShoulderPlane(CameraFrame.tf_trunk_cameraoptical_OriginBasis,
																		CameraFrame.tf_trunk_cameraoptical_OriginVector,
																		0.07,
																		0.12,
																		(int)CameraFrame.DetectedObstacles[co].meanx*4,
																		(int)CameraFrame.DetectedObstacles[co].meany*4);
					if(valid_obstacle==false){
					cv::rectangle(  fullrgbframe,
									cv::Point( CameraFrame.DetectedObstacles[co].min_x*4, CameraFrame.DetectedObstacles[co].min_y*4 ),
									cv::Point( CameraFrame.DetectedObstacles[co].max_x*4, CameraFrame.DetectedObstacles[co].max_y*4 ),
									cv::Scalar( 255, 255, 255 ),1,8 );
					}
					else{
					cv::rectangle(  fullrgbframe,
									cv::Point( CameraFrame.DetectedObstacles[co].min_x*4, CameraFrame.DetectedObstacles[co].min_y*4 ),
									cv::Point( CameraFrame.DetectedObstacles[co].max_x*4, CameraFrame.DetectedObstacles[co].max_y*4 ),
									cv::Scalar( 0, 255, 0 ),1,8 );
					}
				}
				
				//Drawing Landmarks
				for (unsigned int co=0; co<CameraFrame.T_landmarks.size(); co++){
						cv::circle( fullrgbframe,cv::Point(CameraFrame.T_landmarks[co].x*SUB_SAMPLING_PARAMETER,CameraFrame.T_landmarks[co].y*SUB_SAMPLING_PARAMETER) ,3, cv::Scalar( 130, 0, 130 ), 5,1);					
				}

				for (unsigned int co=0; co<CameraFrame.L_landmarks.size(); co++){
						cv::circle( fullrgbframe,cv::Point(CameraFrame.L_landmarks[co].x*SUB_SAMPLING_PARAMETER,CameraFrame.L_landmarks[co].y*SUB_SAMPLING_PARAMETER) ,3, cv::Scalar( 0, 77, 255 ), 5,1);
				}

				for (unsigned int co=0; co<CameraFrame.X_landmarks.size(); co++){
						cv::circle( fullrgbframe,cv::Point(CameraFrame.X_landmarks[co].x*SUB_SAMPLING_PARAMETER,CameraFrame.X_landmarks[co].y*SUB_SAMPLING_PARAMETER) ,3, cv::Scalar( 255, 77, 0 ), 5,1);
				}
				

				for (unsigned int co=0; co<CameraFrame.lines.size(); co++){
				cv::circle( fullrgbframe,cv::Point(CameraFrame.lines[co].x*SUB_SAMPLING_PARAMETER,CameraFrame.lines[co].y*SUB_SAMPLING_PARAMETER) ,1, cv::Scalar( 0,69,139 ), 2,10 );
				}

				//CM
				//Drawing the NimbroStyle field hull
				for(int ni = 0; ni < SUB_SAMPLING_WIDTH - 1; ni++){
					cv::line( fullrgbframe, cv::Point(ni * SUB_SAMPLING_PARAMETER,SUB_SAMPLING_HEIGHT*SUB_SAMPLING_PARAMETER - nimbroStyleLineFinder.ln_Detector->m_FieldBoundary[ni] * SUB_SAMPLING_PARAMETER),
											cv::Point((ni + 1) * SUB_SAMPLING_PARAMETER, SUB_SAMPLING_HEIGHT*SUB_SAMPLING_PARAMETER - nimbroStyleLineFinder.ln_Detector->m_FieldBoundary[ni + 1] * SUB_SAMPLING_PARAMETER),
											cv::Scalar(140, 50, 140), 1, 0);
				}
				
				//CM
				//Drawing NimbroClassicLines
				for ( int li = 0; li < nimbroStyleLineFinder.ln_field_lines_num; li++ ) {

					ObjectRecognition::FieldLine2 & fl = nimbroStyleLineFinder.ln_field_lines[ li ];
					ObjectRecognition::Line::LinearGraphComponent& comp = *(ObjectRecognition::Line::LinearGraphComponent*)(fl.mNodes);

					int zeroX = 0;
					int zeroY = SUB_SAMPLING_HEIGHT*SUB_SAMPLING_PARAMETER;

					int lastpx = 0;
					int lastpy = 0;
					int r = rand()*255;
					int g = rand()*255;
					int b = rand()*255;
					for(unsigned int ni=0;ni<comp.mNodes.size();ni++){
						int px = int ( zeroX + (*nimbroStyleLineFinder.ln_Detector->m_NodeBuffer)[ comp.mNodes[ni] ].x_pos * SUB_SAMPLING_PARAMETER );
						int py = int ( zeroY - (*nimbroStyleLineFinder.ln_Detector->m_NodeBuffer)[ comp.mNodes[ni] ].y_pos * SUB_SAMPLING_PARAMETER );
						if(ni > 0){
							cv::line( fullrgbframe, cv::Point(lastpx,lastpy),
											cv::Point(px, py),
											cv::Scalar(b, g, r), 4, 0);
							
							//TODO
							//draw the lines in world-Coordinates
							//if the lines appear straight undistortion works fine if not check camera matrix
						}
						lastpx = px;
						lastpy = py;
					}
				}
				
// 				//Drawing the fieldHull
// 				for (int co=0; co< (int)(CameraFrame.soccerFieldHull.size()-1); co++)
// 				{
// 					PixelPosition pix1 = CameraFrame.soccerFieldHull.getPixelAtPos(co);
// 					PixelPosition pix2 = CameraFrame.soccerFieldHull.getPixelAtPos(co+1);					
// 					cv::circle( fullrgbframe,cv::Point(pix1.x*SUB_SAMPLING_PARAMETER,pix1.y*SUB_SAMPLING_PARAMETER) ,2, cv::Scalar( 0, 0, 255 ), 3,8 );
// 					cv::line( fullrgbframe,cv::Point(pix1.x*SUB_SAMPLING_PARAMETER,pix1.y*SUB_SAMPLING_PARAMETER),
// 							cv::Point(pix2.x*SUB_SAMPLING_PARAMETER,pix2.y*SUB_SAMPLING_PARAMETER),cv::Scalar( 0, 0, 255 ),1,0);
// 				}

				//Drawing ball
				if(CameraFrame.orangeBallBag.ballFoundOnTheField){
					cv::circle( fullrgbframe,cv::Point(CameraFrame.orangeBallBag.pos_x,CameraFrame.orangeBallBag.pos_y),CameraFrame.orangeBallBag.radiusB, cv::Scalar( 255, 0, 0 ), 3,8 );
				}

			
				//Drawing convexhull of yellow regions
				for (int co=0; co<CameraFrame.goalHulls.getSectionCounter(); co++){
					for (int hp=0; hp<CameraFrame.goalHulls.goalSections[co].size()-1;hp++){

					HullPoint HPoint1= CameraFrame.goalHulls.goalSections[co].getPixelAtPos(hp);
					HullPoint HPoint2= CameraFrame.goalHulls.goalSections[co].getPixelAtPos(hp+1);
					cv::circle( fullrgbframe,cv::Point(HPoint1.x*SUB_SAMPLING_PARAMETER,HPoint1.y*SUB_SAMPLING_PARAMETER) ,2, cv::Scalar( 255, 0, 255 ), 3,8 );
					cv::line( fullrgbframe,cv::Point(HPoint1.x*SUB_SAMPLING_PARAMETER,HPoint1.y*SUB_SAMPLING_PARAMETER),
							cv::Point(HPoint2.x*SUB_SAMPLING_PARAMETER,HPoint2.y*SUB_SAMPLING_PARAMETER),cv::Scalar( 255, 0, 255 ),1,0);

				}
				}

				//Drawing lower points of yellow regions
				for (int co=0; co<CameraFrame.goalLowerPoints.getSectionCounter(); co++){
						for (int hp=0; hp<CameraFrame.goalLowerPoints.goalSections[co].size();hp++){
							HullPoint HPoint1= CameraFrame.goalLowerPoints.goalSections[co].getPixelAtPos(hp);
							cv::circle( fullrgbframe,cv::Point(HPoint1.x*SUB_SAMPLING_PARAMETER,HPoint1.y*SUB_SAMPLING_PARAMETER) ,2, cv::Scalar( 0, 255, 0 ), 5,8 );
				}
				}

				//Drawing middle points of yellow regions
					for (int co=0; co<CameraFrame.goalMiddlePoints.getSectionCounter(); co++){
						for (int hp=0; hp<CameraFrame.goalMiddlePoints.goalSections[co].size();hp++){
							HullPoint HPoint1= CameraFrame.goalMiddlePoints.goalSections[co].getPixelAtPos(hp);
							cv::circle( fullrgbframe,cv::Point(HPoint1.x*SUB_SAMPLING_PARAMETER,HPoint1.y*SUB_SAMPLING_PARAMETER) ,2, cv::Scalar( 0, 255, 255 ), 5,8 );

				}
				}
				
				int colorscale = 15;

				
				sensor_msgs::Image::Ptr rgbout;
				rgbout = boost::make_shared<sensor_msgs::Image>();	
				rgbout->width = ORG_IMAGE_WIDTH;
				rgbout->height = ORG_IMAGE_HEIGHT;
				rgbout->step = 3*ORG_IMAGE_WIDTH;
				rgbout->encoding = std::string("bgr8",4);
				rgbout->data.resize(ORG_IMAGE_WIDTH*ORG_IMAGE_HEIGHT*3);	
				memcpy(&rgbout->data[0],fullrgbframe.data, 800*600*3);
				rgbout->header.stamp = this_frame_time_stamp;
				image_pub_.publish(rgbout);	
						

				sensor_msgs::Image::Ptr segmentationout = boost::make_shared<sensor_msgs::Image>();
				segmentationout->width = ORG_IMAGE_WIDTH;
				segmentationout->height = ORG_IMAGE_HEIGHT;
				segmentationout->step =  ORG_IMAGE_WIDTH;
				segmentationout->encoding = std::string("mono8");
				segmentationout->data.resize(ORG_IMAGE_WIDTH*ORG_IMAGE_HEIGHT);					
				cv::Mat segmentation = cv::Mat(600,800,CV_8UC1,CameraFrame.classified_data)*63;
				memcpy(&segmentationout->data[0],segmentation.data, 800*600);
				segmentationout->header.stamp = this_frame_time_stamp;
				image_pub0_.publish(segmentationout);	
				
				
				sensor_msgs::Image::Ptr out1 = boost::make_shared<sensor_msgs::Image>();	
				out1->width = SUB_SAMPLING_WIDTH;
				out1->height = SUB_SAMPLING_HEIGHT;
				out1->step = SUB_SAMPLING_WIDTH;
				out1->encoding = std::string("mono8");
				out1->data.resize(SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT);					
				cv::Mat smallimg1 = 255 - cv::Mat(SUB_SAMPLING_HEIGHT,SUB_SAMPLING_WIDTH,CV_8U,CameraFrame.currentLUT.objects[0].subimg)*colorscale;
				memcpy(&out1->data[0], smallimg1.data, SUB_SAMPLING_HEIGHT*SUB_SAMPLING_WIDTH);
				out1->header.stamp = this_frame_time_stamp;
				image_pub1_.publish(out1);			

				
				sensor_msgs::Image::Ptr out2 = boost::make_shared<sensor_msgs::Image>();	
				out2->width = SUB_SAMPLING_WIDTH;
				out2->height = SUB_SAMPLING_HEIGHT;
				out2->step = SUB_SAMPLING_WIDTH;
				out2->encoding = std::string("mono8");
				out2->data.resize(SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT);
				cv::Mat smallimg2 = 255 - cv::Mat(SUB_SAMPLING_HEIGHT,SUB_SAMPLING_WIDTH,CV_8U,CameraFrame.currentLUT.objects[1].subimg)*colorscale;
				memcpy(&out2->data[0], smallimg2.data, SUB_SAMPLING_HEIGHT*SUB_SAMPLING_WIDTH);
				out2->header.stamp = this_frame_time_stamp;
				image_pub2_.publish(out2);		
				

				
				sensor_msgs::Image::Ptr out3 = boost::make_shared<sensor_msgs::Image>();
				out3->width = SUB_SAMPLING_WIDTH;
				out3->height = SUB_SAMPLING_HEIGHT;
				out3->step = SUB_SAMPLING_WIDTH;
				out3->encoding = std::string("mono8");
				out3->data.resize(SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT);
				cv::Mat smallimg3 = 255 - cv::Mat(SUB_SAMPLING_HEIGHT,SUB_SAMPLING_WIDTH,CV_8U,CameraFrame.currentLUT.objects[2].subimg)*colorscale;
				memcpy(&out3->data[0], smallimg3.data, SUB_SAMPLING_HEIGHT*SUB_SAMPLING_WIDTH);
				out3->header.stamp = this_frame_time_stamp;
				image_pub3_.publish(out3);

				
				sensor_msgs::Image::Ptr out4 = boost::make_shared<sensor_msgs::Image>();	
				out4->width = SUB_SAMPLING_WIDTH;
				out4->height = SUB_SAMPLING_HEIGHT;
				out4->step = SUB_SAMPLING_WIDTH;
				out4->encoding = std::string("mono8");
				out4->data.resize(SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT);
				cv::Mat smallimg4 = 255 - cv::Mat(SUB_SAMPLING_HEIGHT,SUB_SAMPLING_WIDTH,CV_8U,CameraFrame.currentLUT.objects[3].subimg)*colorscale;
				memcpy(&out4->data[0], smallimg4.data, SUB_SAMPLING_HEIGHT*SUB_SAMPLING_WIDTH);
				out4->header.stamp = this_frame_time_stamp;
				image_pub4_.publish(out4);
				
				
				sensor_msgs::Image::Ptr out5 = boost::make_shared<sensor_msgs::Image>();
				out5->width = SUB_SAMPLING_WIDTH;
				out5->height = SUB_SAMPLING_HEIGHT;
				out5->step = SUB_SAMPLING_WIDTH;
				out5->encoding = std::string("mono8");
				out5->data.resize(SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT);	
				cv::Mat smallimg5 = 255 - cv::Mat(SUB_SAMPLING_HEIGHT,SUB_SAMPLING_WIDTH,CV_8U,CameraFrame.currentLUT.objects[4].subimg)*colorscale;
				memcpy(&out5->data[0], smallimg5.data, SUB_SAMPLING_HEIGHT*SUB_SAMPLING_WIDTH);
				out5->header.stamp = this_frame_time_stamp;
				image_pub5_.publish(out5);

				
				sensor_msgs::Image::Ptr out6 = boost::make_shared<sensor_msgs::Image>();
				out6->width = SUB_SAMPLING_WIDTH;
				out6->height = SUB_SAMPLING_HEIGHT;
				out6->step = SUB_SAMPLING_WIDTH;
				out6->encoding = std::string("mono8");
				out6->data.resize(SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT);
				cv::Mat smallimg6 = cv::Mat(SUB_SAMPLING_HEIGHT,SUB_SAMPLING_WIDTH,CV_8U,CameraFrame.FieldMask);				
				memcpy(&out6->data[0], smallimg6.data, SUB_SAMPLING_HEIGHT*SUB_SAMPLING_WIDTH);
				out6->header.stamp = this_frame_time_stamp;
				image_pub6_.publish(out6);	
				
				sensor_msgs::Image::Ptr out7 = boost::make_shared<sensor_msgs::Image>();
				out7->width = SUB_SAMPLING_WIDTH;
				out7->height = SUB_SAMPLING_HEIGHT;
				out7->step = SUB_SAMPLING_WIDTH;
				out7->encoding = std::string("mono8");
				out7->data.resize(SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT);											
				cv::Mat smallimg7 = cv::Mat(SUB_SAMPLING_HEIGHT,SUB_SAMPLING_WIDTH,CV_8U,MarksAndLinesFinder.img_skeleton2_copy)*255;
				memcpy(&out7->data[0], smallimg7.data, SUB_SAMPLING_HEIGHT*SUB_SAMPLING_WIDTH);
				out7->header.stamp = this_frame_time_stamp;
				image_pub7_.publish(out7);	
								
				
				
			}//END IF GUI enable.
		
						
			CameraFrame.X_landmarks.clear();
			CameraFrame.T_landmarks.clear();			
			CameraFrame.L_landmarks.clear();
			CameraFrame.lines.clear();		 
			
		  
}//END: SoccerVision::processImage

