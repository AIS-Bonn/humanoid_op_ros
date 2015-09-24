// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

// camera_parameters.h
// Provides the constants required to distort and undistort points in the camera.

// Ensure header is only included once
#ifndef CAMERA_PARAMETERS_H
#define CAMERA_PARAMETERS_H

// Notes:
// The constants were obtained using the OpenCV camera calibration (version 2.4.6) using a checkerboard pattern
// fx and fy are the camera focal lengths
// cx and cy specify the camera optical center
// k1 k2 p1 p2 k3 k4 k5 k6 are the distortion parameters
//
// Refer to:
// http://docs.opencv.org/doc/tutorials/calib3d/camera_calibration/camera_calibration.html

// CamParam struct
struct CamParam
{
	// Camera resolution
    static const float rx;
    static const float ry;

	// Camera parameters
    static const float fx;
	static const float fy;
	static const float cx;
	static const float cy;

	// Radial distortion parameters
	static const float k1;
	static const float k2;
	static const float k3;
	static const float k4;
	static const float k5;
	static const float k6;

	// Tangential distortion parameters
	static const float p1;
	static const float p2;

	// Linear distortion extension parameters
	static const float api; // The naming scheme is [a]/[b] + [p]ositive/[n]egative + [i]nner/[o]uter
	static const float apo;
	static const float ani;
	static const float ano;
	static const float bpi;
	static const float bpo;
	static const float bni;
	static const float bno;
	static const float mx;
	static const float bx;
	static const float my;
	static const float by;
};

#endif /* CAMERA_PARAMETERS_H */
// EOF