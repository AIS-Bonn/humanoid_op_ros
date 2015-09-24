// basic opencv based face detection
// author: Sebastian Schüller

#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <face_detection/Faces.h>

#include <pluginlib/class_list_macros.h>

#include <face_detection/face_detection.h>

#include <config_server/parameter.h>

namespace face_detection
{

FaceDetecter::FaceDetecter()
 : m_scale_factor("/face_detection/scale_factor", 0, 0.1, 10.0, 1.5)
 , m_min_neighbors("/face_detection/min_neighbors", 0, 1, 10, 3)
 , m_min_size("/face_detection/min_size", 0, 5, 500, 40)
 , m_frame_height(600)
 , m_frame_width(800)
{
}

void FaceDetecter::onInit()
{

	ros::NodeHandle nh = getPrivateNodeHandle();
	std::string face_cascade_name;
	nh.param<std::string>("cascade", face_cascade_name, "haarcascade_frontalface_default.xml");

	m_it = boost::make_shared<image_transport::ImageTransport>(nh);

	if (!m_face_cascade.load(
		ros::package::getPath("face_detection")
		+ "/haarcascades/"
		+ face_cascade_name)
	)
	{
		ROS_ERROR("Cascade file %s not found.", face_cascade_name.c_str());
		throw std::runtime_error("File not found");
	}

	m_img_sub = m_it->subscribe("image", 1, &FaceDetecter::imgCb, this);
	m_img_pub = m_it->advertise("face_image", 1);
	m_faces_pub = nh.advertise<face_detection::Faces>("faces", 1);
}

void FaceDetecter::imgCb(const sensor_msgs::ImageConstPtr& img)
{
	cv_bridge::CvImageConstPtr cvImg;
	
	if(img->encoding == "YUYV")
	{
		cv_bridge::CvImagePtr mImg = boost::make_shared<cv_bridge::CvImage>();
		mImg->header = img->header;
		mImg->encoding = "bgr8";
		
		const cv::Mat source((int)img->height, (int)img->width, CV_8UC2, const_cast<unsigned char *>(&img->data[0]), img->step);
		
		cv::cvtColor(source, mImg->image, CV_YUV2BGR_YUYV);
		cvImg = mImg;
	}
	else
		cvImg = cv_bridge::toCvShare(img, "bgr8");
	
	std::vector<cv::Rect> faces;
	cv::Mat face_grayscale;
	
	cv::cvtColor(cvImg->image, face_grayscale, cv::COLOR_BGR2GRAY);

	m_face_cascade.detectMultiScale(
		face_grayscale,
		faces,
		m_scale_factor(),
		m_min_neighbors(),
		0|CV_HAAR_SCALE_IMAGE,
		cv::Size(m_min_size(), m_min_size())
	);

	m_frame_height = img->height;
	m_frame_width  = img->width;
	publish(faces);
	if (m_img_pub.getNumSubscribers() > 0)
		publishFaceImage(cvImg, faces);

}

void FaceDetecter::publish(std::vector< cv::Rect > faces)
{
	Faces facesmsg;
	unsigned int i;
	for(i = 0; i < faces.size(); ++i)
	{
		Rectangle face;
		face.x = faces[i].x - (m_frame_width  / 2); // Match cv and ros coordinate system
		face.y = faces[i].y - (m_frame_height / 2);
		face.height = faces[i].height;
		face.width  = faces[i].width;

		facesmsg.faces.push_back(face);
	}
	m_faces_pub.publish(facesmsg);
}

void FaceDetecter::publishFaceImage(cv_bridge::CvImageConstPtr cvImg, std::vector< cv::Rect > faces)
{
	cv::Mat img(cvImg->image);
	unsigned int i;
	for (i = 0; i < faces.size(); ++i)
		cv::rectangle(img, faces[i], cv::Scalar(255, 0, 0), 4, 8);

	cv_bridge::CvImage pubImage(cvImg->header, cvImg->encoding, img);
	m_img_pub.publish(pubImage.toImageMsg());
}

}

PLUGINLIB_EXPORT_CLASS(face_detection::FaceDetecter, nodelet::Nodelet)

