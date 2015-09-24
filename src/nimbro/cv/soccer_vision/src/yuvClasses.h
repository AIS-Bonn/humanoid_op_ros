// Soccer Vision node
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>
/**
* @file  soccer_vision/src/yuvClasses.h
* @brief Implements the Color classification and Image-subsampling of the Nimbro-OP Soccer package.
* 
* \b Color \b classification
* 
* The classification of pixels is done individually, every pixel will be checked in the look up tables to 
* determine to which class it belongs to (i.e.: Black, White, Green, Orange, Yellow, No Class).
* 
*  \image html soccer_vision/ColorClassesSmall.png Color classification diagram
* 
* A YUV look up table is composed of the following data:
* 
* 1) Y range [minY,maxY]
* 
* 2) UV image, which is a 255x255 image that stores true/false  values 
*    to denote the coordinates in the color space that belong or not to an specific color class.
*
* \image html soccer_vision/LUTdefinition2.png Y range and UV Look up table
*  
* For a pixel to belong to a specific color class, its Y value has to be within a certain range and the (U,V) coordinate has to be set to true
* in the look up table.
* 
* There are 5 color classes in the NimbRo-OP vision code, naturally, each one has an id-number that identifies: 
* * [0] Black 
* * [1] White
* * [2] Green (field) 
* * [3] Orange (ball)
* * [4] Yellow (goals)
*
* The following image is an exaple of the color classification procedure, where every gray scale tone represents a color class.
* 
* \image html soccer_vision/colorClassification.png Soccer field after color classification
* 
* \b Image-subsampling
* 
* 
* An other important function of this class is to carry out image sub-sampling, which is called after the color 
* classification of each YUV frame. As aforedmentioned, the original size of an image is 800x600 with 
* ca. 25 fps.  However, all the detections are done using images 4 times smaller than the original 
* one (size 200x150) to speed up the detection process.  Furthermore, there are 5 binary images
* that contain only the pixels that correspond to its specific class.
*
* 
* These sub-images are created using a 4x4 window (16 pixels) from the original image that maps into 
* 1 pixel of the the sub-image. The following image shows the top-left (4x4 pixel) part of an image 
* which is being mapped into the different color classes. 
* 
* \image html soccer_vision/subsampling.png Subsampling the classified image
* 
* In the above example we can the see general idea of the method: the original image
* has 4 (4x4 pixel) windows, each one corresponds to 1 pixel of the subimage. 
* 
* The approach to fill out the sub images is as follows:
* 
* For all the pixels in the  4x4 windows of the original image:
*    - Count the incidences of each color class
*    - Put the total count in the corresponding color class subimage
* 
* It should be noted that the maximum number of incidences is 16, meaning that  the analized 
* window only had pixels of one color class.
* 
* 
* The following set of images show the result of the color classification and the image subsampling.
* 
* <table border="0">
* <tr>     
*     <th>\image html soccer_vision/cc0.png </th>
*     <th>\image html soccer_vision/cc1.png </th>
*     <th>\image html soccer_vision/cc2.png </th>
*     <th>\image html soccer_vision/cc3.png </th>
*     <th>\image html soccer_vision/cc4.png </th>
* </tr>
* <tr>     
*     <th>Black</th>
*     <th>White</th>
*     <th>Green</th>
*     <th>Orange</th>
*     <th>Yellow</th>
* </tr>
* </table>
* 
**/
#ifndef YUVCLASSES_H
#define YUVCLASSES_H

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

#include "globaldefinitions.h"


using namespace std;

namespace soccervision{

//---------Begin class definition---------------------------------------------------

/*! \class YUVColorClass
    \brief This class defines a particular color class object.

    It contains the data after classification that corresponds to a specific type of color on the soccer field.
*/
class YUVColorClass{
	public:

	unsigned char id;

//! Min Y value 
	unsigned char minY;


//! Max Y value 
	unsigned char maxY;
	
	string name;

	unsigned char subcounter;
	

//! Buffer that containts the classified data 
	unsigned char subimg[SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT];

//! Costructor
	YUVColorClass()
	 : minY(100)
	 , maxY(0)
	{}

//! Destructor	
	~YUVColorClass(){}

};
//---------End class definition---------------------------------------------------



//---------Begin class definition---------------------------------------------------

/*! \class YUVColorLUT
    \brief Class that stores all the necessary LUTs.
    This class stores all the look up tables that are necesary to perform the soccer vision color classification. It also stores the 
    subimages that contain the data that corresponds to single color classes.
*/
class YUVColorLUT{
	public:
	
		//! Objects look up table
		unsigned char lut_data[LUT_HEIGHT*LUT_WIDTH];
		
		//! White look up table
		unsigned char lut_white[LUT_HEIGHT*LUT_WIDTH];
		
		//! Black look up table
		unsigned char lut_black[LUT_HEIGHT*LUT_WIDTH];
		vector <YUVColorClass> objects;

		YUVColorLUT();
		~YUVColorLUT();

		//! Initialize the look up tables with zero values.
		void initialize();
		
/*! \fn unsigned char classifyYUVFrameIntoColors(unsigned char Y, unsigned char U, unsigned char V)
    \brief Classifies a YUV pixel.
    \param  Y value
    \param  U value
    \param  V value
*/
		unsigned char classifyYUVFrameIntoColors(unsigned char Y, unsigned char U, unsigned char V);

/*! \fn ReadYUVLookUpTable(const string inputName)
    \brief Reads a Look up table from a file
    \param inputName
*/
		bool ReadYUVLookUpTable(const string inputName);
		
/*! \fn ReadYUVWhiteLookUpTable(const string inputName)
    \brief Reads a Look up table from a file
    \param inputName
*/
		bool ReadYUVWhiteLookUpTable(const string inputName);
		
/*! \fn ReadYUVBlackLookUpTable(const string inputName)
    \brief Reads a Look up table from a file
    \param inputName
*/
		bool ReadYUVBlackLookUpTable(const string inputName);

};
//---------End class definition---------------------------------------------------



/*! \fn  yuvImageSubsampling(unsigned char classificationBuffer[],vector <YUVColorClass> & objects, unsigned char mask[])
    \brief Subsamples the image to reduce its size.
    \param classificationBuffer buffer with the clissified colors
    \param objects vector of YUV objects that will store the data that belongs to specific color classes
    \param mask coordinates of the camera that ase useful
*/
void yuvImageSubsampling(unsigned char classificationBuffer[],vector <YUVColorClass> & objects, unsigned char mask[]);

}

#endif
