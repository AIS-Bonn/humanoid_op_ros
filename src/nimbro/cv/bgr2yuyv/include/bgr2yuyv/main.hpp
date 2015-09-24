// Main Loop for the Vision Module
// Main Loop for the Vision Module
// Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <bgr2yuyv/CameraDummy.hpp>

int main(int,char**);

 unsigned char saturateValue(float f)
{
	return (unsigned char) (f >= 255 ? 255 : (f < 0 ? 0 : f));
}

 void yuv2rgb(unsigned char yuvimg[], unsigned char rgbbuffer[],
		int W, int H)
{

	for (int i = 0; i < (W * H * 3); i += 3)
	{

		rgbbuffer[i] = saturateValue(
				yuvimg[i] + 1.402f * (yuvimg[i + 2] - 128));
		rgbbuffer[i + 1] = saturateValue(
				yuvimg[i] - 0.34414f * (yuvimg[i + 1] - 128)
						- 0.71414f * (yuvimg[i + 2] - 128));
		rgbbuffer[i + 2] = saturateValue(
				yuvimg[i] + 1.772f * (yuvimg[i + 1] - 128));
	}

}
