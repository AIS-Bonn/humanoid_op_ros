// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

#include "yuvClasses.h"

using namespace std;
using namespace soccervision;



YUVColorLUT::YUVColorLUT(){
	objects.clear();
}

YUVColorLUT::~YUVColorLUT(){}

void YUVColorLUT::initialize(){
	objects.resize(CC_LAST+1);
	//Note that 255 is the id of all uv values without a class
	for (int i=0;i<(LUT_WIDTH*LUT_HEIGHT);i++){
		lut_data[i]  = CC_NO_CLASS;
		lut_white[i]  = CC_NO_CLASS;
		lut_black[i]  = CC_NO_CLASS;
	}	
}
//---------------------------------------------------------------------------------------------------------------------------
//This funstion receives a YUV value and classifies it into its corresponding color class. 
//That is: Black, White, Ball, Green field, Yellow goal
unsigned char YUVColorLUT::classifyYUVFrameIntoColors(unsigned char Y, unsigned char U, unsigned char V){

	 unsigned char colorid;

	 colorid = lut_data[LUT_WIDTH*V+U];
	 if (colorid != CC_NO_CLASS && Y>=objects[colorid].minY && Y<=objects[colorid].maxY ){
		return colorid;
	 }
	 
	//check for Black Colors
	 colorid = lut_black[LUT_WIDTH*V+U];		 	 
	 if (colorid == CC_BLACK && Y>=objects[CC_BLACK].minY  && Y<=objects[CC_BLACK].maxY ){
		return CC_BLACK;
	 }

	//check for White Colors
	 colorid = lut_white[LUT_WIDTH*V+U];
	 if (colorid == CC_WHITE && Y>=objects[CC_WHITE].minY  && Y<=objects[CC_WHITE].maxY ){
		return CC_WHITE;
	 }

	
	 //check for Pure Black Color
	 if (Y<=PURE_BLACK_Y){
		return CC_BLACK;
	 }

	 //check for Pure White Color
	 if (Y>=PURE_WHITE_Y){
		return CC_WHITE;
	 }


	 
	 return CC_NO_CLASS; //Note: CC_NO_CLASS (255) is the id that identifies objects without a color class
}
//---------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------------
//This function reads the object Look up table
//Ball, Field and Goal (Organge, Green, Yellow)
bool YUVColorLUT::ReadYUVLookUpTable(const string inputName){
	//Read YUV data of one object
	//the data is stored as follows (where values are integers [0,256])
	//Color class: name=none colorid=255
	//Color class: name=field colorid=2 miny=68 maxy=103
	//Color class: name=ball colorid=3 miny=56 maxy=113
	//u_v_id values: 83 205 1
	
	//Note that "255" (CC_NO_CLASS) is the id of all uv values without a class
	

	YUVColorClass tmpobject;

	string line;
	char classname [100];
	int id, miny, maxy;
	int tmp_u, tmp_v;

	std::string absName = ros::package::getPath("soccer_vision") + "/" + inputName;


		ROS_INFO("Reading from: %s", inputName.c_str() );

		ifstream objectfile (absName.c_str());
		if (objectfile.is_open())
		{
			while ( objectfile.good() )
			{
				getline (objectfile,line);

				if ( sscanf(line.c_str(), "Color class: name=%s colorid=%d miny=%d maxy=%d", classname, &id, &miny, &maxy) == 4){

					tmpobject.id = id;
					tmpobject.minY = miny;
					tmpobject.maxY = maxy;
					tmpobject.name = classname;
					if((id < 0 || id > CC_LAST) && id != CC_NO_CLASS)
					{
						ROS_ERROR("Invalid color code in Objects.lut: %d", id);
						return false;
					}
					if(id != CC_NO_CLASS)
						objects[id] = tmpobject;
				}
				if ( sscanf(line.c_str(), "u_v_id values: %d %d %d", &tmp_u, &tmp_v,&id) == 3){
					lut_data[LUT_WIDTH*tmp_v + tmp_u] = (unsigned char) id;
				}

			}
			objectfile.close();
		}
		else{
			ROS_ERROR("Unable to open file where the YUV data of the object is stored:%s", inputName.c_str() );
			return false;
		}



	return true;
	//End reading file
}
//---------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------------
//This function reads the white Look up table
bool YUVColorLUT::ReadYUVWhiteLookUpTable(const string inputName){
	//Read YUV data of the white lines (Special Case)
	//the data is stored as follows (where values are integers [0,256])
	//Color class: name=none colorid=255
	
	//Note that "255" (CC_NO_CLASS) is the id of all uv values without a class
	
	
	YUVColorClass tmpobject;

	string line;
	char classname [100];
	int id, miny, maxy;
	int tmp_u, tmp_v;

	std::string absName = ros::package::getPath("soccer_vision") + "/" + inputName;

		ROS_INFO("Reading from: %s", inputName.c_str() );

		ifstream objectfile (absName.c_str());
		if (objectfile.is_open())
		{
			while ( objectfile.good() )
			{
				getline (objectfile,line);

				if ( sscanf(line.c_str(), "Color class: name=%s colorid=%d miny=%d maxy=%d", classname, &id, &miny, &maxy) == 4){

					tmpobject.id = id;
					tmpobject.minY = miny;
					tmpobject.maxY = maxy;
					tmpobject.name = classname;
					if(id > CC_LAST && id != CC_NO_CLASS)
					{
						ROS_ERROR("Invalid color code in Objects.lut: %d", id);
						return false;
					}
					if(id != CC_NO_CLASS)
						objects[id] = tmpobject;
				}
				if ( sscanf(line.c_str(), "u_v_id values: %d %d %d", &tmp_u, &tmp_v,&id) == 3){
					lut_white[LUT_WIDTH*tmp_v + tmp_u] = (unsigned char) id;
				}

			}
			objectfile.close();
		}
		else{
			ROS_ERROR("Unable to open file where the YUV data of the object is stored:%s", inputName.c_str() );
			return false;
		}



	return true;
	//End reading file
}
//---------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------------
//This function reads the black Look up table
bool YUVColorLUT::ReadYUVBlackLookUpTable(const string inputName){
	//Read YUV data of the white lines (Special Case)
	//the data is stored as follows (where values are integers [0,256])
	//Color class: name=none colorid=255


	//Note that "255" (CC_NO_CLASS) is the id of all uv values without a class

	YUVColorClass tmpobject;

	string line;
	char classname [100];
	int id, miny, maxy;
	int tmp_u, tmp_v;

	std::string absName = ros::package::getPath("soccer_vision") + "/" + inputName;


		ROS_INFO("Reading from: %s", inputName.c_str() );

		ifstream objectfile (absName.c_str());
		if (objectfile.is_open())
		{
			while ( objectfile.good() )
			{
				getline (objectfile,line);

				if ( sscanf(line.c_str(), "Color class: name=%s colorid=%d miny=%d maxy=%d", classname, &id, &miny, &maxy) == 4){

					tmpobject.id = id;
					tmpobject.minY = miny;
					tmpobject.maxY = maxy;
					tmpobject.name = classname;
					if(id > CC_LAST && id != CC_NO_CLASS)
					{
						ROS_ERROR("Invalid color code in Objects.lut: %d", id);
						return false;
					}
					if(id != CC_NO_CLASS)
						objects[id] = tmpobject;
				}
				if ( sscanf(line.c_str(), "u_v_id values: %d %d %d", &tmp_u, &tmp_v,&id) == 3){
					lut_black[LUT_WIDTH*tmp_v + tmp_u] = (unsigned char) id;
				}

			}
			objectfile.close();
		}
		else{
			ROS_ERROR("Unable to open file where the YUV data of the object is stored:%s", inputName.c_str() );
			return false;
		}



	return true;
	//End reading file
}
//---------------------------------------------------------------------------------------------------------------------------



void soccervision::yuvImageSubsampling(unsigned char classificationBuffer[], vector <YUVColorClass> & objects, unsigned char mask[]){

 //the sub-sampling will be carried out according
//to the values of the following definitions Check: globaldefinitions.h

	int vectorSize = objects.size();
	for (int k=0; k<vectorSize;k++){objects[k].subcounter = 0;}//initializing
	int pixelInSubImage = 0;


	for(int i=0; i<ORG_IMAGE_HEIGHT; i+=SUB_SAMPLING_PARAMETER){
	 for(int j=0; j<ORG_IMAGE_WIDTH; j+=SUB_SAMPLING_PARAMETER){


			 for (int  smi=i; smi<(i+SUB_SAMPLING_PARAMETER); smi++){
				 for (int  smj=j; smj<(j+SUB_SAMPLING_PARAMETER); smj++){


					 unsigned char val = classificationBuffer[ORG_IMAGE_WIDTH*smi + smj];

					 //val can have different color classes {Field, Ball, Black, White, Goal}
					 if(val > CC_LAST && val != CC_NO_CLASS)
					 {
						    ROS_WARN_THROTTLE(0.1, "Invalid color code 0x%02X", val);
							continue;
					 }

					 if (val !=  CC_NO_CLASS){ objects[val].subcounter +=1; }

				 }
			 }

			 
			 //Using the subimage mask to reject some areas of the image
			 if (mask[pixelInSubImage] == 1){
				 for (int k=0; k<vectorSize;k++){
					 objects[k].subimg[pixelInSubImage] = objects[k].subcounter;
					 objects[k].subcounter = 0; //initializing the value
				 }
			 }
			 else{
				 //the value was rejected by the mask
				 for (int k=0; k<vectorSize;k++){
					 objects[k].subimg[pixelInSubImage] = 0;
					 objects[k].subcounter = 0;
				 }
			 }
			 

			 pixelInSubImage++;


	  }
	}
}
//---------------------------------------------------------------------------------------------------------------------------
