// Test the contrib RobotModel class
// File: test_robotmodel.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <ros/init.h>
#include <cap_gait/contrib/RobotModel.h>
#include <gait/gait_common.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

// Namespaces
using namespace std;
using namespace gait;

// Rip open the RobotModel class
class TestRobotModel : public margait_contrib::RobotModel
{
public:
	// Constructor
	TestRobotModel() : RobotModel(config()) {}
	
	// Expose normally protected functions
	void alignModel(double fusedX, double fusedY) { RobotModel::alignModel(fusedX, fusedY); }
	void setPose(const margait_contrib::Pose& pose) { RobotModel::setPose(pose); }
	
	// CapConfig object
	static cap_gait::CapConfig* config()
	{
		if(!s_config) s_config = new cap_gait::CapConfig();
		return s_config;
	}
	
private:
	// Static variables
	static cap_gait::CapConfig* s_config;
};

// Static TestRobotModel variables
cap_gait::CapConfig* TestRobotModel::s_config = NULL;

// Enumerations
enum ValueIDs
{
	V_TIME,
	V_JOINTPOS,
	V_LASTJOINTPOS = V_JOINTPOS + NUM_JOINTS - 1,
	V_FUSEDX,
	V_FUSEDY,
	V_RXRM_SUPPX,
	V_RXRM_SUPPY,
	V_RX_SUPPLEG,
	V_RX_COMX,
	V_RX_COMY,
	V_RX_COMVX,
	V_RX_COMVY,
	NUM_VALUES
};

// Data structs
template <int NUM> struct GenRData
{
	GenRData() { for(int i = 0; i < NUM; i++) data[i] = 0.0; }
	double data[NUM];
	static const int num = NUM;
};
template <int NUM> struct GenRDataArr
{
	GenRDataArr() {}
	inline std::string nameOf(int index) const
	{
		if(index < 0 || index >= NUM || name[index].empty())
		{
			stringstream ss;
			ss << "Data" << index;
			return ss.str();
		}
		else
			return name[index];
	}
	std::string name[NUM];
	std::vector<GenRData<NUM> > list;
	static const int num = NUM;
};

// Typedefs
typedef GenRData<NUM_VALUES> RData;
typedef GenRDataArr<NUM_VALUES> RDataArr;

// Look up a joint ID by string name
int jointIDByName(const std::string& name)
{
	// Look for a matching name
	for(int i = 0; i < NUM_JOINTS; i++)
	{
		if(jointName[i] == name)
			return i;
	}

	// Return that no match was found
	return -1;
}

// Parse a CSV file for its contents
bool parseCSV(const std::string& csvname, RDataArr& arr)
{
	// Constants
	const std::string jointStatesPositionsStr = "Joint states/Positions/";

	// Declare variables
	std::map<int,int> colmap;
	std::string line, cell;

	// Open the input file for reading
	std::ifstream file(csvname.c_str());
	if(!file.is_open())
	{
		cout << "Failed to open file '" << csvname << "'!" << endl;
		return false;
	}

	// Get the column header line
	if(!std::getline(file, line))
	{
		cout << "The file '" << csvname << "' doesn't seem to have any data in it!" << endl;
		file.close();
		return false;
	}

	// Parse the column headers
	int colnum = 0;
	std::stringstream liness(line);
	cout << "Parsing CSV data column headers:" << endl;
	while(std::getline(liness, cell, ','))
	{
		// Trim the whitespace and quotation marks from the front/end of the cell
		size_t startchar = cell.find_first_not_of(" \t\"");
		size_t endchar = cell.find_last_not_of(" \t\"");
		if(startchar == string::npos || endchar == string::npos || startchar > endchar)
		{
			colmap[colnum] = NUM_VALUES; // Ignore data in this column...
			continue;
		}
		std::string colname = cell.substr(startchar, endchar - startchar + 1);

		// Populate the column map
		if((colname.length() >= jointStatesPositionsStr.length()) && (colname.compare(0, jointStatesPositionsStr.length(), jointStatesPositionsStr) == 0))
		{
			int id = jointIDByName(colname.substr(jointStatesPositionsStr.length()));
			colmap[colnum] = V_JOINTPOS + id;
		}
		else if(colname == "Time")
			colmap[colnum] = V_TIME;
		else if(colname == "cap_gait/fusedAngle/fusedY")
			colmap[colnum] = V_FUSEDY;
		else if(colname == "cap_gait/fusedAngle/fusedX")
			colmap[colnum] = V_FUSEDX;
		else if(colname == "cap_gait/rxRobotModel/supportVector/x")
			colmap[colnum] = V_RXRM_SUPPX;
		else if(colname == "cap_gait/rxRobotModel/supportVector/y")
			colmap[colnum] = V_RXRM_SUPPY;
		else if(colname == "cap_gait/rxModel/supportLegSign")
			colmap[colnum] = V_RX_SUPPLEG;
		else if(colname == "cap_gait/rxModel/vx")
			colmap[colnum] = V_RX_COMVX;
		else if(colname == "cap_gait/rxModel/vy")
			colmap[colnum] = V_RX_COMVY;
		else if(colname == "cap_gait/rxModel/x")
			colmap[colnum] = V_RX_COMX;
		else if(colname == "cap_gait/rxModel/y")
			colmap[colnum] = V_RX_COMY;
		else
		{
			colmap[colnum] = NUM_VALUES;
			cout << "[IGNORE] " << colname << endl;
		}
		if(colmap[colnum] < NUM_VALUES)
		{
			arr.name[colmap[colnum]] = colname;
			cout << "[FOUND]  " << colname << endl;
		}

		// Increment the column number
		colnum++;
	}

	// Trailing display
	cout << endl;

	// Declare that we're parsing the file
	cout << "Parsing '" << csvname << "'..." << endl;

	// Declare variables
	int row = 0;
	RData d, oldd;

	// Parse the data in the csv file
	while(std::getline(file, line))
	{
		// Declare variables
		int col = -1;
		bool haveposdata = false;

		// Back up the current robot data struct
		oldd = d; // TODO: It is a problem that d is not zeroed in between output rows, and so missing data gets duplicated...
		
		// Split the line up by comma delimiters
		std::stringstream liness(line);
		while(std::getline(liness, cell, ','))
		{
			// Increment the column number
			col++;

			// Retrieve the robot data index
			std::map<int,int>::iterator it = colmap.find(col);
			if(it == colmap.end()) continue;
			if(it->second < 0 || it->second >= NUM_VALUES) continue;
			int index = it->second;

			// Trim the whitespace and quotation marks from the front/end of the cell
			size_t startchar = cell.find_first_not_of(" \t\"");
			size_t endchar = cell.find_last_not_of(" \t\"");
			if(startchar == string::npos || endchar == string::npos || startchar > endchar)
				continue; // No interesting data in this cell
			std::istringstream datass(cell.substr(startchar, endchar - startchar + 1));

			// Convert the cell to a double
			double value;
			if(!(datass >> value))
				continue; // Could not parse a double from the cell

			// Save the data value
			d.data[index] = value;

			// Note if this was position data
			haveposdata |= (index >= V_JOINTPOS) && (index <= V_LASTJOINTPOS);
		}

		// Save the old data struct if we just saw position data
		if(haveposdata)
			arr.list.push_back(oldd);
		
		// Increment the row number
		row++;
	}

	// Display how many rows were read
	cout << "Read " << row << " rows." << endl;
	cout << "Extracted " << arr.list.size() << " data points." << endl;
	cout << endl;

	// Close the CSV file
	file.close();

	// Return success
	return true;
}

// Export the robot data to a CSV file
template <int NUM> bool exportData(const std::string& csvname, const GenRDataArr<NUM>& arr)
{
	// Display which file we're opening
	cout << "Opening file '" << csvname << "' for output..." << endl;
	
	// Open a file for output
	std::ofstream file(csvname.c_str());
	if(!file.is_open())
	{
		cout << "Failed to open file '" << csvname << "'!" << endl << endl;
		return false;
	}

	// Configure the output file double representation (%.15g)
	file.unsetf(ios::floatfield);
	file.precision(15);

	// Declare variables
	int col;
	bool printed;
	
	// See which columns are valid
	bool valid[NUM] = {false};
	for(col = 0; col < NUM; col++)
		valid[col] = !arr.name[col].empty();
	
	// Write the header row
	printed = false;
	for(col = 0; col < NUM; col++)
	{
		if(!valid[col]) continue;
		if(printed)
			file << ",";
		file << '"' << arr.nameOf(col) << '"';
		printed = true;
	}
	file << endl;

	// Write all the rows to the output
	for(size_t i = 1; i < arr.list.size(); i++)
	{
		const double* val = &(arr.list.at(i).data[0]);
		printed = false;
		for(col = 0; col < NUM; col++)
		{
			if(!valid[col]) continue;
			if(printed)
				file << ",";
			file << val[col];
			printed = true;
		}
		file << endl;
	}

	// Display how many rows were written
	cout << "Wrote " << arr.list.size() << " rows." << endl;
	cout << endl;

	// Close the output file
	file.close();

	// Return success
	return true;
}

// Merge two files into a third one
bool mergeFiles(const std::string& csvnameA, const std::string& csvnameB, const std::string& outname)
{
	// Declare variables
	std::string headerA, headerB, lineA, lineB, cell;
	std::vector<std::string> outheaders;
	std::vector<int> mapA, mapB;

	// Open input file A for reading
	cout << "Opening file '" << csvnameA << "' for reading..." << endl;
	std::ifstream fileA(csvnameA.c_str());
	if(!fileA.is_open())
	{
		cout << "Failed to open file '" << csvnameA << "'!" << endl;
		return false;
	}
	if(!std::getline(fileA, headerA))
	{
		cout << "The file '" << csvnameA << "' doesn't seem to have any data in it!" << endl;
		fileA.close();
		return false;
	}
	
	// Open input file B for reading
	cout << "Opening file '" << csvnameB << "' for reading..." << endl;
	std::ifstream fileB(csvnameB.c_str());
	if(!fileB.is_open())
	{
		cout << "Failed to open file '" << csvnameB << "'!" << endl;
		fileA.close();
		return false;
	}
	if(!std::getline(fileB, headerB))
	{
		cout << "The file '" << csvnameB << "' doesn't seem to have any data in it!" << endl;
		fileA.close();
		fileB.close();
		return false;
	}
	
	// Open the output file for writing
	cout << "Opening file '" << outname << "' for writing..." << endl;
	std::ofstream outfile(outname.c_str());
	if(!outfile.is_open())
	{
		cout << "Failed to open file '" << outname << "'!" << endl << endl;
		fileA.close();
		fileB.close();
		return false;
	}
	
	// Configure the output file double representation (%.15g)
	outfile.unsetf(ios::floatfield);
	outfile.precision(15);
	
	// Trailing display
	cout << endl;
	
	// Initialise the output headers array
	cout << "Parsing column headers and constructing output column header list..." << endl;
	outheaders.assign(1, ""); // Reserve space for a time column at the very left
	
	// Process the headers extracted from fileA
	mapA.clear();
	std::stringstream headerAss(headerA);
	while(std::getline(headerAss, cell, ','))
	{
		// Trim the whitespace and quotation marks from the front/end of the cell
		size_t startchar = cell.find_first_not_of(" \t\"");
		size_t endchar = cell.find_last_not_of(" \t\"");
		if(startchar == string::npos || endchar == string::npos || startchar > endchar)
		{
			mapA.push_back(-1);
			continue;
		}
		std::string colname = cell.substr(startchar, endchar - startchar + 1);
		
		// If this is a time column then treat it specially
		if(colname == "Time")
		{
			cout << "Found a time column in file A." << endl;
			outheaders.at(0) = colname;
			mapA.push_back(0);
			continue;
		}
		
		// If this isn't a duplicate column then use it
		if(std::find(outheaders.begin(), outheaders.end(), colname) == outheaders.end())
		{
			outheaders.push_back(colname);
			mapA.push_back(outheaders.size() - 1);
		}
		else
		{
			cout << "Ignoring duplicated column '" << colname << "'!" << endl;
			mapA.push_back(-1);
		}
	}
	
	// Process the headers extracted from fileB
	mapB.clear();
	std::stringstream headerBss(headerB);
	while(std::getline(headerBss, cell, ','))
	{
		// Trim the whitespace and quotation marks from the front/end of the cell
		size_t startchar = cell.find_first_not_of(" \t\"");
		size_t endchar = cell.find_last_not_of(" \t\"");
		if(startchar == string::npos || endchar == string::npos || startchar > endchar)
		{
			mapB.push_back(-1);
			continue;
		}
		std::string colname = cell.substr(startchar, endchar - startchar + 1);
		
		// If this is a time column then treat it specially
		if(colname == "Time")
		{
			cout << "Found a time column in file B." << endl;
			outheaders.at(0) = colname;
			mapB.push_back(0);
			continue;
		}
		
		// If this isn't a duplicate column then use it
		if(std::find(outheaders.begin(), outheaders.end(), colname) == outheaders.end())
		{
			outheaders.push_back(colname);
			mapB.push_back(outheaders.size() - 1);
		}
		else
		{
			cout << "Ignoring duplicated column '" << colname << "'!" << endl;
			mapB.push_back(-1);
		}
	}
	
	// Create an output time column 
	if(outheaders.at(0).empty())
	{
		cout << "No time column was found in either file => Simply enumerating rows!" << endl;
		outheaders.at(0) = "Time";
	}
	
	// Save how many output columns there is going to be
	size_t NumOut = outheaders.size();
	
	// Trailing display
	cout << endl;
	
	// Output the output headers
	cout << "Output headers..." << endl;
	for(size_t i = 0; i < outheaders.size(); i++)
	{
		if(i > 0) outfile << ",";
		outfile << '"' << outheaders.at(i) << '"';
		printf("Column %d: %s\n", (int) i, outheaders.at(i).c_str());
	}
	outfile << endl;
	cout << endl;
	
	// Declare variables
	std::vector<double> outputrow;
	bool writtenTime;
	int row = 0, col;
	
	// Parse, merge and output the data in the CSV files
	while(std::getline(fileA, lineA) && std::getline(fileB, lineB))
	{
		// Increment row counter
		row++;
		
		// Initialise variables
		outputrow.assign(NumOut, 0);
		writtenTime = false;
		
		// Parse the data from fileA
		col = -1;
		std::stringstream lineAss(lineA);
		while(std::getline(lineAss, cell, ','))
		{
			// Increment the column counter
			col++;

			// Trim the whitespace and quotation marks from the front/end of the cell
			size_t startchar = cell.find_first_not_of(" \t\"");
			size_t endchar = cell.find_last_not_of(" \t\"");
			if(startchar == string::npos || endchar == string::npos || startchar > endchar) continue; // No interesting data in this cell
			std::istringstream datass(cell.substr(startchar, endchar - startchar + 1));

			// Convert the cell to a double
			double value;
			if(!(datass >> value)) continue; // Could not parse a double from the cell

			// Insert the parsed data into the output row
			if(col < (int) mapA.size())
			{
				int mappedcol = mapA.at(col); // The array mapA maps fileA column number to output column number (-1 => No corresponding output column)
				if(mappedcol > 0)
					outputrow.at(mappedcol) = value;
				else if(mappedcol == 0)
				{
					if(writtenTime)
					{
						if(outputrow.at(mappedcol) != value)
							printf("Mismatching timestamp %.7f vs %.7f!\n", outputrow.at(mappedcol), value);
					}
					else
						outputrow.at(mappedcol) = value;
					writtenTime = true;
				}
			}
		}
		
		// Parse the data from fileB
		col = -1;
		std::stringstream lineBss(lineB);
		while(std::getline(lineBss, cell, ','))
		{
			// Increment the column counter
			col++;

			// Trim the whitespace and quotation marks from the front/end of the cell
			size_t startchar = cell.find_first_not_of(" \t\"");
			size_t endchar = cell.find_last_not_of(" \t\"");
			if(startchar == string::npos || endchar == string::npos || startchar > endchar) continue; // No interesting data in this cell
			std::istringstream datass(cell.substr(startchar, endchar - startchar + 1));

			// Convert the cell to a double
			double value;
			if(!(datass >> value)) continue; // Could not parse a double from the cell

			// Insert the parsed data into the output row
			if(col < (int) mapB.size())
			{
				int mappedcol = mapB.at(col); // The array mapB maps fileB column number to output column number (-1 => No corresponding output column)
				if(mappedcol > 0)
					outputrow.at(mappedcol) = value;
				else if(mappedcol == 0)
				{
					if(writtenTime)
					{
						if(outputrow.at(mappedcol) != value)
							printf("Mismatching timestamp %.7f vs %.7f!\n", outputrow.at(mappedcol), value);
					}
					else
						outputrow.at(mappedcol) = value;
					writtenTime = true;
				}
			}
		}
		
		// Populate the time column if there is no input column mapped to it
		if(!writtenTime)
			outputrow.at(0) = row - 1;
		
		// Output the data row to file
		for(size_t i = 0; i < outputrow.size(); i++)
		{
			if(i > 0) outfile << ",";
			outfile << outputrow.at(i);
		}
		outfile << endl;
	}
	
	// Display how many rows were read
	cout << "Parsed and merged " << row << " rows." << endl;
	cout << endl;
	
	// Close all opened files
	fileA.close();
	fileB.close();
	outfile.close();
	
	// Return success
	return true;
}

// Display the robot model state
void displayModelState(const margait_contrib::RobotModel* rxRobotModel)
{
	// Declare variables
	qglviewer::Quaternion orient;
	qglviewer::Vec pos;

	// Get the support foot and the other foot
	const qglviewer::Frame& footFloorPoint = (rxRobotModel->supportLegSign == 1 ? rxRobotModel->rFootFloorPoint : rxRobotModel->lFootFloorPoint);
	const qglviewer::Frame& otherFootPt    = (rxRobotModel->supportLegSign != 1 ? rxRobotModel->rFootFloorPoint : rxRobotModel->lFootFloorPoint);

	// Display info
	pos = rxRobotModel->base.position();
	orient = rxRobotModel->base.orientation();
	printf("Base Pos:    %.6f %.6f %.6f\n", pos.x, pos.y, pos.z);
	printf("Base Orient: %.6f %.6f %.6f %.6f\n", orient[3], orient[0], orient[1], orient[2]);
	printf("Base FYaw:   %.6f\n\n", margait_contrib::RobotModel::fusedYaw(orient));
	pos = rxRobotModel->leftFootstep.position();
	orient = rxRobotModel->leftFootstep.orientation();
	printf("LeftFootstep Pos:    %.6f %.6f %.6f\n", pos.x, pos.y, pos.z);
	printf("LeftFootstep Orient: %.6f %.6f %.6f %.6f\n", orient[3], orient[0], orient[1], orient[2]);
	printf("LeftFootstep FYaw:   %.6f\n\n", margait_contrib::RobotModel::fusedYaw(orient));
	pos = rxRobotModel->rightFootstep.position();
	orient = rxRobotModel->rightFootstep.orientation();
	printf("RightFootstep Pos:    %.6f %.6f %.6f\n", pos.x, pos.y, pos.z);
	printf("RightFootstep Orient: %.6f %.6f %.6f %.6f\n", orient[3], orient[0], orient[1], orient[2]);
	printf("RightFootstep FYaw:   %.6f\n\n", margait_contrib::RobotModel::fusedYaw(orient));
	pos = footFloorPoint.position();
	orient = footFloorPoint.orientation();
	printf("FootFloorPt Pos:    %.6f %.6f %.6f [%c]\n", pos.x, pos.y, pos.z, (rxRobotModel->supportLegSign == 1 ? 'R' : 'L'));
	printf("FootFloorPt Orient: %.6f %.6f %.6f %.6f\n", orient[3], orient[0], orient[1], orient[2]);
	printf("FootFloorPt FYaw:   %.6f\n\n", margait_contrib::RobotModel::fusedYaw(orient));
	pos = otherFootPt.position();
	orient = otherFootPt.orientation();
	printf("OtherFootPt Pos:    %.6f %.6f %.6f [%c]\n", pos.x, pos.y, pos.z, (rxRobotModel->supportLegSign != 1 ? 'R' : 'L'));
	printf("OtherFootPt Orient: %.6f %.6f %.6f %.6f\n", orient[3], orient[0], orient[1], orient[2]);
	printf("OtherFootPt FYaw:   %.6f\n\n", margait_contrib::RobotModel::fusedYaw(orient));
}

// Display the model vectors
void displayModelVectors(const margait_contrib::RobotModel* rxRobotModel)
{
	// Retrieve the required vectors
	qglviewer::Vec suppVec = rxRobotModel->suppComVector();
	qglviewer::Vec stepVec = rxRobotModel->suppStepVector();
	qglviewer::Vec swingVec = rxRobotModel->suppSwingVector();
	double stepYaw = rxRobotModel->suppStepYaw();

	// Display the required vectors
	printf("supportVector: %.6f %.6f %.6f\n", suppVec.x, suppVec.y, suppVec.z);
	printf("stepVector:    %.6f %.6f %.6f (yaw %.6f)\n", stepVec.x, stepVec.y, stepVec.z, stepYaw);
	printf("swingVector:   %.6f %.6f %.6f\n\n", swingVec.x, swingVec.y, swingVec.z);
}

// Run the RobotModel forward kinematics on some robot joint position and fused angle data
// Assumes: TIME, FUSEDX, FUSEDY, JOINTPOS, RXRM_SUPPX, RXRM_SUPPY
bool runFwdKin(const RDataArr& arr, const std::string& outname)
{
	// Display a header
	cout << "Executing function " << __FUNCTION__ << "():" << endl;

	// Create robotmodel object
	TestRobotModel rxRobotModel;

	// Set the initial support leg sign
	double rawsign = arr.list.at(0).data[V_RX_SUPPLEG];
	if(rawsign == 0.0 && arr.list.size() >= 2) rawsign = arr.list.at(1).data[V_RX_SUPPLEG];
	int sign = (rawsign >= 0.0 ? 1 : -1);
	rxRobotModel.setSupportLeg(sign);
	rxRobotModel.supportExchangeLock = false;
	
	// Initialise the output data array
	GenRDataArr<8> Out;
	Out.list.resize(arr.list.size());
	Out.name[0] = "Time";
	Out.name[1] = "CALC/cap_gait/rxRobotModel/supportVector/x";
	Out.name[2] = "CALC/cap_gait/rxRobotModel/supportVector/y";
	Out.name[3] = "CALC/cap_gait/rxRobotModel/supportVector/z";
	Out.name[4] = "CALC/cap_gait/rxRobotModel/stepVector/x";
	Out.name[5] = "CALC/cap_gait/rxRobotModel/stepVector/y";
	Out.name[6] = "CALC/cap_gait/rxRobotModel/stepVector/z";
	Out.name[7] = "CALC/cap_gait/rxRobotModel/stepVector/fyaw";

	// Declare variables
	qglviewer::Vec fusedAngle;
	const double* val;
	int suppDiffCount = 0;

	// Loop through each data point
	for(size_t pt = 1; pt < arr.list.size(); pt++)
	{
		// Save a pointer to the current data point
		val = &(arr.list.at(pt).data[0]);

		// Populate the fused angle vector
		fusedAngle.x = val[V_FUSEDX];
		fusedAngle.y = val[V_FUSEDY];
		fusedAngle.z = 0.0;

		// Retrieve the robot's measured pose in terms of joint angles and populate a Pose struct with it
		// Note: We only need to populate the x, y, z fields for each joint, as this is all that our rxRobotModel needs.
		const double* jointPos = &(val[V_JOINTPOS]);
		margait_contrib::Pose measuredPose;
		measuredPose.headPose.neck.setPos(0.0, 0.0, 0.0);
		measuredPose.trunkPose.spine.setPos(0.0, 0.0, 0.0);
		measuredPose.leftArmPose.shoulder.setPos(jointPos[L_SHOULDER_ROLL], jointPos[L_SHOULDER_PITCH], 0.0);
		measuredPose.leftArmPose.elbow.setPos(0.0, jointPos[L_ELBOW_PITCH], 0.0);
		measuredPose.leftLegPose.hip.setPos(jointPos[L_HIP_ROLL], jointPos[L_HIP_PITCH], jointPos[L_HIP_YAW]);
		measuredPose.leftLegPose.knee.setPos(0.0, jointPos[L_KNEE_PITCH], 0.0);
		measuredPose.leftLegPose.ankle.setPos(jointPos[L_ANKLE_ROLL], jointPos[L_ANKLE_PITCH], 0.0);
		measuredPose.rightArmPose.shoulder.setPos(jointPos[R_SHOULDER_ROLL], jointPos[R_SHOULDER_PITCH], 0.0);
		measuredPose.rightArmPose.elbow.setPos(0.0, jointPos[R_ELBOW_PITCH], 0.0);
		measuredPose.rightLegPose.hip.setPos(jointPos[R_HIP_ROLL], jointPos[R_HIP_PITCH], jointPos[R_HIP_YAW]);
		measuredPose.rightLegPose.knee.setPos(0.0, jointPos[R_KNEE_PITCH], 0.0);
		measuredPose.rightLegPose.ankle.setPos(jointPos[R_ANKLE_ROLL], jointPos[R_ANKLE_PITCH], 0.0);

		// Make the robot model walk
		rxRobotModel.update(measuredPose, fusedAngle.x, fusedAngle.y);

		// Get the robot model state
		qglviewer::Vec suppvec = rxRobotModel.suppComVector();
		qglviewer::Vec stepvec = rxRobotModel.suppStepVector();
		double stepyaw = rxRobotModel.suppStepYaw();

		// Verify the match of calculated to plotted robotmodel data
		double ErrSuppX = fabs(suppvec.x - val[V_RXRM_SUPPX]);
		double ErrSuppY = fabs(suppvec.y - val[V_RXRM_SUPPY]);
		if(ErrSuppX > 2e-7 || ErrSuppY > 2e-7)
		{
			suppDiffCount++;
			printf("WARNING: Found a difference of (%.7f,%.7f) in row %lu between calculated and measured RobotModel support vector data [#%d]!\n", ErrSuppX, ErrSuppY, pt+1, suppDiffCount);
		}
		
		// Save the calculated data
		Out.list.at(pt).data[0] = val[V_TIME];
		Out.list.at(pt).data[1] = suppvec.x;
		Out.list.at(pt).data[2] = suppvec.y;
		Out.list.at(pt).data[3] = suppvec.z;
		Out.list.at(pt).data[4] = stepvec.x;
		Out.list.at(pt).data[5] = stepvec.y;
		Out.list.at(pt).data[6] = stepvec.z;
		Out.list.at(pt).data[7] = stepyaw;
	}
	
	// Export the data if required
	if(!outname.empty())
	{
		printf("Writing calculated columns to file...\n");
		for(int i = 0; i < Out.num; i++)
			printf("Column %d: %s\n", i, Out.nameOf(i).c_str());
		printf("\n");
		exportData(outname, Out);
	}

	// Return success
	return true;
}

// Do a test on some robot data
// Assumes: TIME, FUSEDX, FUSEDY, JOINTPOS
void runRobotModel(const RDataArr& arr)
{
	// Display a header
	cout << "Executing test function " << __FUNCTION__ << "():" << endl;

	// Create robotmodel object
	TestRobotModel rxRobotModel;

	// Set the initial support leg sign
	double rawsign = arr.list.at(0).data[V_RX_SUPPLEG];
	if(rawsign == 0.0 && arr.list.size() >= 2) rawsign = arr.list.at(1).data[V_RX_SUPPLEG];
	int sign = (rawsign >= 0.0 ? 1 : -1);
	rxRobotModel.setSupportLeg(sign);
	rxRobotModel.supportExchangeLock = false;

	// Declare variables
	qglviewer::Vec fusedAngle;
	const double* val;

	// Loop through each required data point
	for(size_t pt = 1; pt < arr.list.size(); pt++)
	{
		// Save a pointer to the current data point
		val = &(arr.list.at(pt).data[0]);

		// Populate the fused angle vector
		fusedAngle.x = val[V_FUSEDX];
		fusedAngle.y = val[V_FUSEDY];
		fusedAngle.z = 0.0;

		// Retrieve the robot's measured pose in terms of joint angles and populate a Pose struct with it
		// Note: We only need to populate the x, y, z fields for each joint, as this is all that our rxRobotModel needs.
		const double* jointPos = &(val[V_JOINTPOS]);
		margait_contrib::Pose measuredPose;
		measuredPose.headPose.neck.setPos(0.0, 0.0, 0.0);
		measuredPose.trunkPose.spine.setPos(0.0, 0.0, 0.0);
		measuredPose.leftArmPose.shoulder.setPos(jointPos[L_SHOULDER_ROLL], jointPos[L_SHOULDER_PITCH], 0.0);
		measuredPose.leftArmPose.elbow.setPos(0.0, jointPos[L_ELBOW_PITCH], 0.0);
		measuredPose.leftLegPose.hip.setPos(jointPos[L_HIP_ROLL], jointPos[L_HIP_PITCH], jointPos[L_HIP_YAW]);
		measuredPose.leftLegPose.knee.setPos(0.0, jointPos[L_KNEE_PITCH], 0.0);
		measuredPose.leftLegPose.ankle.setPos(jointPos[L_ANKLE_ROLL], jointPos[L_ANKLE_PITCH], 0.0);
		measuredPose.rightArmPose.shoulder.setPos(jointPos[R_SHOULDER_ROLL], jointPos[R_SHOULDER_PITCH], 0.0);
		measuredPose.rightArmPose.elbow.setPos(0.0, jointPos[R_ELBOW_PITCH], 0.0);
		measuredPose.rightLegPose.hip.setPos(jointPos[R_HIP_ROLL], jointPos[R_HIP_PITCH], jointPos[R_HIP_YAW]);
		measuredPose.rightLegPose.knee.setPos(0.0, jointPos[R_KNEE_PITCH], 0.0);
		measuredPose.rightLegPose.ankle.setPos(jointPos[R_ANKLE_ROLL], jointPos[R_ANKLE_PITCH], 0.0);

		// Make the robot model walk
		rxRobotModel.update(measuredPose, fusedAngle.x, fusedAngle.y);

		// Get the robot model state
		double leftz = rxRobotModel.lFootFloorPoint.position().z;
		double rightz = rxRobotModel.rFootFloorPoint.position().z;

		// Display stuff
		printf("[%.3f] Lz=%7.4f Rz=%7.4f SL=%s Lock=%d\n", val[V_TIME], leftz, rightz, (rxRobotModel.supportLegSign > 0.0 ? "Right" : "Left "), rxRobotModel.supportExchangeLock);
	}

	// Trailing display
	cout << endl;
}

// Test worker function for resetOdom()
void test_resetOdom(margait_contrib::RobotModel* rxRobotModel)
{
	// Display divider
	printf("------------------------------------------------------\n");
	
	// Test resetting the odometry => Before reset
	printf("Before reset odometry...\n\n");
	displayModelState(rxRobotModel);
	displayModelVectors(rxRobotModel);

	// Test resetting the odometry => Reset
	rxRobotModel->resetOdom();

	// Test resetting the odometry => After reset
	printf("After reset odometry...\n\n");
	displayModelState(rxRobotModel);
	displayModelVectors(rxRobotModel);
}

// Test worker function for setOdom()
void test_setOdom(margait_contrib::RobotModel* rxRobotModel, double comX, double comY, double rotZ)
{
	// Display divider
	printf("------------------------------------------------------\n");

	// Test resetting the odometry => Before reset
	printf("Before set odometry...\n\n");
	displayModelState(rxRobotModel);
	displayModelVectors(rxRobotModel);

	// Test resetting the odometry => Reset
	rxRobotModel->setOdom(comX, comY, rotZ);

	// Test resetting the odometry => After reset
	printf("After set odometry to (%g, %g, %g)...\n\n", comX, comY, rotZ);
	displayModelState(rxRobotModel);
	displayModelVectors(rxRobotModel);
}

// Test worker function for alignModel()
void test_alignCase(TestRobotModel* rxRobotModel, double fusedX, double fusedY)
{
	// Align the model
	printf("------------------------------------------------------\n");
	printf("Aligning by: fusedX = %g, fusedY = %g\n\n", fusedX, fusedY);
	rxRobotModel->alignModel(fusedX, fusedY);

	// Display the model state
	displayModelState(rxRobotModel);
}

// Test worker function for setSupportLeg()
void test_setSupportLeg(margait_contrib::RobotModel* rxRobotModel, int supportLegSign)
{
	// Set the support leg
	printf("------------------------------------------------------\n");
	printf("Setting support leg to [%c]\n\n", (supportLegSign == 1 ? 'R' : 'L'));
	rxRobotModel->setSupportLeg(supportLegSign);

	// Display the model state
	displayModelState(rxRobotModel);
}

// Test worker function for supportVector(), stepVector() and swingVector()
void test_getVector(const margait_contrib::RobotModel* rxRobotModel)
{
	// Display the required vectors
	printf("------------------------------------------------------\n");
	displayModelVectors(rxRobotModel);
}

// Another test
bool test_alignModel()
{
	// Display a header
	cout << "Executing test function " << __FUNCTION__ << "():" << endl << endl;

	// Create robotmodel object
	TestRobotModel rxRobotModel;

	// Set the pose of the robot
	margait_contrib::Pose pose;
	pose.leftLegPose.hip.setPos(-0.04, 0.07, 0.10);
	pose.leftLegPose.knee.setPos(0.0, 0.08, 0.0);
	pose.leftLegPose.ankle.setPos(0.15, -0.20, 0.0);
	pose.rightLegPose.hip.setPos(0.07, 0.09, -0.05);
	pose.rightLegPose.knee.setPos(0.0, 0.05, 0.0);
	pose.rightLegPose.ankle.setPos(-0.16, 0.11, 0.0);
	rxRobotModel.setPose(pose);

	// Manually move the footStep frame
	rxRobotModel.rightFootstep.setTranslation(3.0, 2.0, 0.0);

	// Display the base/footStep frames
	displayModelState(&rxRobotModel);

	// Test cases
	test_alignCase(&rxRobotModel, 0.0, 0.0);
	test_getVector(&rxRobotModel);
	test_alignCase(&rxRobotModel, 0.4, 0.0);
	test_getVector(&rxRobotModel);
	test_alignCase(&rxRobotModel, 0.0, -0.3);
	test_getVector(&rxRobotModel);
	test_setSupportLeg(&rxRobotModel, -1);
	test_getVector(&rxRobotModel);
	test_alignCase(&rxRobotModel, 0.75, -0.26);
	test_setSupportLeg(&rxRobotModel, 1);
	test_alignCase(&rxRobotModel, -0.63, 0.44);
	test_setSupportLeg(&rxRobotModel, -1);
	test_setSupportLeg(&rxRobotModel, 1);
	test_resetOdom(&rxRobotModel);
	test_setOdom(&rxRobotModel, 0.0, 0.0, 0.0);
	test_setOdom(&rxRobotModel, 14.2, -7.6, -0.524);

	// Return success
	return true;
}

// Parse subfunction
int main_parse(const std::string& csvname, RDataArr& arr)
{
	// Verify that we have a CSV
	if(csvname.empty()) { printf("Expect a valid CSV name as an argument!\n"); return 1; }

	// Parse the CSV file
	if(!parseCSV(csvname, arr))
	{
		cout << "Error occurred while parsing file!" << endl;
		return 1;
	}
	
	// Return success
	return 0;
}

// Parse/out subfunction
int main_parse_out(const std::string& csvname, RDataArr& arr, const std::string& outname)
{
	// Parse the CSV file
	if(main_parse(csvname, arr)) return 1;

	// Check whether an output CSV filename was provided
	if(outname.empty()) { printf("Expect a valid output CSV name following the input CSV name!\n"); return 1; }
	
	// Return success
	return 0;
}

// Main function
int main(int argc, char **argv)
{
	// Initialise the ROS node (needed for CapConfig)
	ros::init(argc, argv, "test_robotmodel");

	// Print header
	cout << "Test RobotModel (margait_contrib)..." << endl;

	// Check whether CSV filenames were provided
	std::string cmdname, csvname, outname;
	if(argc >= 2) cmdname = std::string(argv[1]);
	if(argc >= 3) csvname = std::string(argv[2]);
	if(argc >= 4) outname = std::string(argv[3]);
	
	// Print the command we received
	if(!cmdname.empty())
		cout << "Command: " << argv[1] << endl;
	cout << endl;

	// Declare data array
	RDataArr arr;

	// Execute the required command
	if(cmdname.empty())
	{
		// Complain that we have nothing to do
		printf("No command parameter was provided!\nExpect: test_robotmodel <COMMAND> [<INPUTCSV>] [<ARGS>]\n");
		printf("Available commands: parse, fix, merge, run-robotmodel, run-fwdkin, test-alignmodel\n");
		return 1;
	}
	else if(cmdname == "parse")
	{
		// Parse the CSV file
		if(main_parse(csvname, arr)) return 1;
	}
	else if(cmdname == "fix")
	{
		// Parse the CSV file and get the output CSV filename
		if(main_parse_out(csvname, arr, outname)) return 1;
		
		// Export the fixed data
		if(!exportData(outname, arr)) { printf("Encountered error while exporting data!\n"); return 1; }
	}
	else if(cmdname == "merge")
	{
		// Process arguments
		std::string csvnameA = csvname;
		std::string csvnameB = outname;
		if(argc >= 5)
			outname = std::string(argv[4]);
		else
			outname.clear();
		
		// Make sure all arguments are valid
		if(csvnameA.empty() || csvnameB.empty() || outname.empty())
		{
			printf("Expect two input CSV filenames to merge, followed by an output filename!\n");
			return 1;
		}
		
		// Perform the required merge
		if(!mergeFiles(csvnameA, csvnameB, outname)) { printf("Encountered error while merging data!\n"); return 1; }
	}
	else if(cmdname == "run-robotmodel")
	{
		// Parse the CSV file
		if(main_parse(csvname, arr)) return 1;
		   
		// Execute the tests
		runRobotModel(arr);
	}
	else if(cmdname == "run-fwdkin")
	{
		// Parse the CSV file
		if(main_parse_out(csvname, arr, outname)) return 1;
		   
		// Run the robot model forward kinematics to calculate complementary data
		runFwdKin(arr, outname);
	}
	else if(cmdname == "test-alignmodel")
	{
		// Test align model
		if(test_alignModel()) cout << "Test returned positive!" << endl;
		else                  cout << "Test returned negative!" << endl;
		cout << endl;
	}

	// Return success
	return 0;
}
// EOF