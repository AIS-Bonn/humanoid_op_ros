// basic opencv based face detection
// author: Sebastian Schüller

#ifndef FACE_DETECTION_H
#define FACE_DETECTION_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <config_server/parameter.h>


namespace face_detection
{

class FaceDetecter : public nodelet::Nodelet
{
public:
	FaceDetecter();
	~FaceDetecter(){};

	void onInit();

private:
	void imgCb(const sensor_msgs::ImageConstPtr& img);
	void publish(std::vector<cv::Rect> faces);
	void publishFaceImage(cv_bridge::CvImageConstPtr cvImg, std::vector< cv::Rect > faces);

	cv::CascadeClassifier m_face_cascade;

	boost::shared_ptr<image_transport::ImageTransport> m_it;
	image_transport::Subscriber m_img_sub;
	image_transport::Publisher  m_img_pub;
	ros::Publisher m_faces_pub;

	config_server::Parameter< float > m_scale_factor;
	config_server::Parameter< int > m_min_neighbors;
	config_server::Parameter< int > m_min_size;

	unsigned int m_frame_height;
	unsigned int m_frame_width;
};



}

#endif