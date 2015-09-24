// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

#ifndef FIELDINFORMATIONBAG_H
#define FIELDINFORMATIONBAG_H

#include <iostream>
#include <string>
#include <iostream>
#include <fstream>
#include <string>



class FieldInformationBag
{
public:
	int bounding_box_minx;
	int bounding_box_miny;
	int bounding_box_maxx;
	int bounding_box_maxy;
	
	FieldInformationBag(){};
	~FieldInformationBag(){};
	
	void reset(){
		bounding_box_minx = 1; // Set minimum values to greater than maximum values  
		bounding_box_miny = 1; // so that for-loops exit immediately by default
		bounding_box_maxx = 0;
		bounding_box_maxy = 0;
	}
};


#endif