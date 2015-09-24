// Soccer Vision node
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

/**
* @file soccer_vision.h
* @brief Implements the NimbRo-OP Soccer Vison package.
* 
**/

/**
* @defgroup SoccerVisionPackageDoc Soccer Vision
*
* @section socvissec1 Overview
* This section describes in full the implementation of the Soccer Vision package. This documentation will start with the 
* method used to segment the image. This will be followed by a detail explanation of the procedures that take care of the identification
* of objects while playing robot soccer.
*
* @author Julio Pastrana (<pastrana@ais.uni-bonn.de>)
* @date No release date yet
* @version 0.1
*
* @section socvissec2 Dependencies
*  - V4L2    
*  - OpenCV  
*  - GTK2    (Only the soccer vision GUI)
* 
* @section socvissec2_a  Image analysis and object recognition procedure
* 
* Like any other image processing process the first step is to acquire the data from the source (camera). Then, the data has to be manipulated in a way that 
* it is useful for procedures that will execute the identification of objects.  The following diagram shows the general idea of the whole process.
* 
* 
* \image html soccer_vision/soccer_vison_diagram.png Image processing and object detection diagram 
* 
*
* @section socvissec3  Image Acquisition
* 
* The camera used by Nimbro-OP is the USB Logitech C905, which is the same camera that Darwin-OP currently  has. However, in order to 
* have a better field of view (ca. 180°) a fish eye wide-angle lens was integrated. This lens allows the robot to have more information 
* about the word in each image, which means, more objects in sight at the same time (ball, goal, obstacles, etc.).
* 
* \image html soccer_vision/cameraNimbroOPsmall.jpg Webcam Logitech C905 with wide-angle lens
* 
* The image acquisition is carried out by the plugin `camera_v4l2`. This plugin uses the linux `uvc-camera` driver to 
* retrieve images of a certain width and height, in our case is 800x600 YUYV images to achieve a frame rate above 24 fps. Naturally, it is possible to set other
* parameters to improve the quality of the images, for example: brightness, contrast, saturation, sharpness, exposure, etc.
*
* After retrieving the raw data from the camera, a 1D buffer stores the values that correspond to each pixel of the current image. The size of this buffer 
* is "width*height*2", where the data is arranged using a YUYV format. This means that 1 touple [Y1,U,Y2,V] contaings the in
* formation of 2 pixels [Y1,U,V] and [Y2,U,V]. 
* 
* 
* The following code exemplifies how to retrieve the YUV values from the image buffer.
* @code
* unsigned char Y1,Y2,U,V;
* for (unsigned int i = 0; i<(img->width*img->height*2); i+=4){
*	Y1 = (unsigned char)img->data[i];
*	U  = (unsigned char)img->data[i+1];
*	Y2 = (unsigned char)img->data[i+2];
*	V  = (unsigned char)img->data[i+3];
* }
* @endcode 
* 
* 
* For more details see `soccer_vision.h` and `soccer_vision.cpp`.
*  
*
* @section socvissec4 Frame Grabber 
* 
* Before describing any further the detection steps, it is worth introducing  the class that stores all the images and objects that are required during the execution. An object 
* of the  FrameGrabber class has static references to images that containg the yuv raw data, rgb data, classified data, subsampled data, etc. This means that the rest of 
* the classes involved in the detection process are going to receive a pointer to a FrameGrabber object in order to be able to perform its task. 
* 
* 
* The images of these framework are 1D buffers of size (WIDTH*HEIGHT). For example, accessing individial pixels of an image is done as follows:
* @code
* // For a given pixel position (x,y) 
* // and a 1D buffer image[WIDTH*HEIGHT];
* 
* pixel_possition_in_buffer = WIDTH*y + x;
* @endcode 
* 
* For more information see `soccer_vision/src/frameGrabber.h` 
* 
* @section socvissec4b Color classification (segmentation process)
* 
* As previously mentioned the color space in which the camera delivers the data is 
* the YUV space. In order to avoid extra computations transforming the pixel data 
* into a different color space the data stream will be classified as it comes 
* from the camera. The use of look-up tables that store the specific data of each 
* color class will facilitate this procedure.
* 
* 
* For more information see `soccer_vision/src/yuvClasses.h` 
* 
* @section socvissec5 Find Field
* Finding the field is a very important part of every soccer vision system. A valid assumption while playing soccer is that every thing happens on the field 
* and all the things that have to be detected are within this area. Additionally, knowing exactly where the field is helps speeding up the 
* computer analysis because the detection algorithms can concentrate specific areas of the image.
* 
* For more information see `soccer_vision/src/findField.h` 
*
* @section socvissec6 Find Ball
* 
* The ball is probably one of the most important objects on the field. In robotic soccer the color orange 
* is used to denote the ball. Naturally, only the orange class image will be analyzed in order to find 
* regions that are round or ball like. 
* 
* For more information see `soccer_vision/src/findBall.h` 
* 
* @section socvissec7 Find Goal 
* 
* An other very important object on the field to be identified is the goal. The robot should know its position to know 
* the direction in which it has to kick the ball. The image that is used to perform the identification step is the one that
* contains the yellow class.
* 
* For more information see `soccer_vision/src/findGoal.h` 
* 
* 
* @section socvissec8 Find Obstacles
* 
* Detecting the obstacles on the field is also an imprtant task, which is done using the image that containg the dark areas of the image. 
* 
* For more information see `soccer_vision/src/findObstacles.h` 
* 
*
* @section socvissec9 Soccer Vision GUI
* 
* The Soccer Vision GUI was implemented as a tool that allows the user to visualize the results of the computer vision package.
*
* Running the gui:
* 
* 1) Execute ROS
* @code
* myshell> roscore
* @endcode 
* 2) Execute the GUI. The user interface will wait untill the proper images are published.
* @code
* myshell> rosrun soccer_vision soccercv_gui
* @endcode
* 3) If you have a ROS bag with saved data
* @code
* myshell> rosbag play -l BAGNAME.bag
* @endcode
* 
* \image html soccer_vision/soccervisiongui.png Nimbro-OP Soccer GUI
*  
*
* 
**/




#ifndef SOCCER_VISION_H
#define SOCCER_VISION_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <vector>
#include <stdint.h>

#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <soccer_vision/Detections.h>
#include <field_model/field_model.h>

#include <config_server/parameter.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>


#include "frameGrabber.h"
#include "findField.h"
#include "findBall.h"
#include "findGoal.h"
#include "findObstacles.h"
#include "findLandmarksAndLines.h"
#include "findBodyMask.h"
#include "yuv2rgb.h"
#include "soccer_markers.h"

//CM
//WARNING: experimental
#include "findLinesNimbroStyle.h"




/**
* @namespace soccervision
*
* @ingroup SoccerVisionPackageDoc
*
* @brief This namespace defines everything that is required for the @ref SoccerVisionPackageDoc "Soccer Vision Package".
**/
namespace soccervision
{
	
struct Parameters
{
	Parameters()
	: landmark_filter_dist_x("landmark_filter_dist/x", 0.0, 0.001, 2.0, 0.09)
	, landmark_filter_dist_y("landmark_filter_dist/y", 0.0, 0.001, 2.0, 0.13)
	{}

	config_server::Parameter<float> landmark_filter_dist_x;
	config_server::Parameter<float> landmark_filter_dist_y;
};


	/**
	* @class SoccerVision
	* @brief Soccer vision class.
	* 
	* This class contains all the objects and functions that will be executed during the computer vision process. The two main functions of this class are
	* onInit() and processImage(const sensor_msgs::Image::ConstPtr& img). Where: OnInit will initialize all the structures that will be used in the process, and processImage will call each 
	* identification procedure for each camera frame.
	**/
	class SoccerVision : public nodelet::Nodelet
	{
		public:			
			
			SoccerVision(); 
			virtual ~SoccerVision();

			/*! \fn virtual void onInit();
			*    \brief initialize all the necesary structures and variables.
			*/
			virtual void onInit();

			/*! \fn void processImage(const sensor_msgs::Image::ConstPtr& img); 
			*    \brief Executes the the image processing algorithms for each camera frame.
			*    \param img  pointer to the raw image.
			*/
			void processImage(const sensor_msgs::Image::ConstPtr& img); 

			//! CameraFrame
 			/*! Object that contains static buffers that will be used during the identification process.*/
			FrameGrabber CameraFrame;

			//! Field Finder
 			/*! Object that contains all the functions to enable the localization of the green field on the image.*/
			FindField  FieldFinder;

			//! Ball Finder
 			/*! Object that contains all the functions to enable the localization of the ball on the field.*/			
			FindBall   BallFinder;

			//! Goal Finder
 			/*! Object that contains all the functions to enable the localization of the goal(s) posts on the field.*/						
			FindGoal   GoalFinder;	
			
			//! Obstacle Finder
 			/*! Object that contains all the functions to enable the localization of obstacles on the field.*/									
			FindObstacles ObstacleFinder;
			
			
			FindLandmarksAndLines MarksAndLinesFinder;
			
			//CM
			//experimental nimbro style line finder
			FindLinesNimbroStyle nimbroStyleLineFinder;

		private:

			ros::Publisher m_pub_output;
			message_filters::Subscriber<sensor_msgs::Image> m_sub_input;
			
			
			tf::TransformListener m_tf;
			tf::MessageFilter<sensor_msgs::Image> * m_tf_filter;
			

			ros::Publisher observations_pub;
			soccer_vision::Detections msg;
			soccer_vision::ObjectDetection objs;
			soccer_vision::ObstacleDetection detected_obstacle;
			
			ros::Publisher ball_detected_camera_vector_pub;
			geometry_msgs::PointStamped ball_detected_camera_vector;

			ros::Publisher ball_detected_ego_vector_pub;
			geometry_msgs::PointStamped ball_detected_ego_vector;
			
			
			ros::Publisher detected_objects_markers_pub;
			ros::Publisher detected_obstacles_markers_pub;
			
			int detected_objects_counter;
			int detected_obstacles_counter;
			soccermarker objmarker;
			visualization_msgs::MarkerArray detected_objects_array;
			visualization_msgs::Marker detected_object_marker;
			visualization_msgs::Marker detected_obstacles_line_list;
			
			//ros::Publisher landmarks_on_shoulder_pub;
			//visualization_msgs::MarkerArray landmarks_on_shoulder_array;
			
			//! Soccer GUI
 			/*! Allows publishing images for the soccer gui. */
			bool m_use_gui;
			
			Parameters* m_param;
			
			//SOCCER GUI declaration
			image_transport::ImageTransport * it;
			image_transport::Publisher image_pub_;
			image_transport::Publisher image_pub0_;
			image_transport::Publisher image_pub1_;
			image_transport::Publisher image_pub2_;
			image_transport::Publisher image_pub3_;
			image_transport::Publisher image_pub4_;
			image_transport::Publisher image_pub5_;
			image_transport::Publisher image_pub6_;		
			image_transport::Publisher image_pub7_;
			//END Soccer Gui declaration
			
	};

}//End namespace soccervision
#endif
