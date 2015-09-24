#ifndef FINDLINESNIMBROSTYLE_H
#define FINDLINESNIMBROSTYLE_H

#include "globaldefinitions.h"
#include "Line.h"
#include "frameGrabber.h"

class FindLinesNimbroStyle
{
public:
	
	//Constants
	enum{
		MAX_FIELDLINES=50,
		MAX_FIELDLINE_INTERSECTIONS=12
	};
		
	// * LINE DETECTION * //
	ObjectRecognition::Line* ln_Detector; //<- Line Detection Algorithms Class
	int ln_iTmp        [ SUB_SAMPLING_HEIGHT ][ SUB_SAMPLING_WIDTH ];
	int ln_iSmoothWM   [ SUB_SAMPLING_HEIGHT ][ SUB_SAMPLING_WIDTH ]; //<- WM stands for White Matrix.
	int ln_iSkeletonWM [ SUB_SAMPLING_HEIGHT ][ SUB_SAMPLING_WIDTH ];
	int ln_iNodesWM    [ SUB_SAMPLING_HEIGHT ][ SUB_SAMPLING_WIDTH ];
	int ln_iMergedBlack[ SUB_SAMPLING_HEIGHT ][ SUB_SAMPLING_WIDTH ]; //<- Black plus Team Marker colors ( cyan and magenta ).
	int ln_iPureWhite  [ SUB_SAMPLING_HEIGHT ][ SUB_SAMPLING_WIDTH ]; //<- White without DarkWhite.
	int ln_iMergedGreen[ SUB_SAMPLING_HEIGHT ][ SUB_SAMPLING_WIDTH ]; //<- Green with ln_iMergedBlack and ln_iPureWhite.
	WorldObjectType ln_Type;

	int ln_white_wo_markers[ SUB_SAMPLING_HEIGHT ][ SUB_SAMPLING_WIDTH ]; //<- White lines without markers.
	int ln_field_lines_num;
	ObjectRecognition::FieldLine2 ln_field_lines[ MAX_FIELDLINES ];
	const float MIN_FIELDLINE_CONF;
	int ln_field_line_intersections_num;
	EgoObject ln_field_line_intersections[ MAX_FIELDLINE_INTERSECTIONS ];

	int goal_target_x_min[ 2 ];
	int goal_target_y_min[ 2 ];
	int goal_target_x_max[ 2 ];
	int goal_target_y_max[ 2 ];
	int goal_target_area_conf[ 2 ];

	int ln_main_orientation;
	int ln_theta[ 2 ][ 2 ];
	int ln_indexof_rho[ 2 ][ 2 ];
	float ln_conf[ 2 ][ 2 ];
	int ln_rho[ 2 ][ 2 ];
	float ln_u[ 2 ][ 2 ];
	float ln_stdev_u[ 2 ][ 2 ];
	int av_x[ 2 ][ 2 ];
	int av_y[ 2 ][ 2 ];
	
	enum{
		Black,
		White,
		Green,
		BallColor,
		NumColors
	};
	
	int colors[NumColors][SUB_SAMPLING_HEIGHT][SUB_SAMPLING_WIDTH];
	
	
	//Methods
	FindLinesNimbroStyle();
	~FindLinesNimbroStyle();
	void findLines(FrameGrabber &camFrame);
	
};

#endif