#include "findLinesNimbroStyle.h"
#include <string.h>
#include "nimbroStyleTools.h"

FindLinesNimbroStyle::FindLinesNimbroStyle() 
					: MIN_FIELDLINE_CONF( 0.04f )
{
	ln_Detector = new ObjectRecognition::Line;
}

FindLinesNimbroStyle::~FindLinesNimbroStyle()
{
	delete ln_Detector;
}

void FindLinesNimbroStyle::findLines(FrameGrabber &camFrame)
{
	
	for(int i = 0; i < SUB_SAMPLING_HEIGHT; i++){
		for(int j = 0; j < SUB_SAMPLING_WIDTH; j++){
			colors[FindLinesNimbroStyle::Black][i][j] = static_cast<int>(camFrame.currentLUT.objects[CC_BLACK].subimg[(SUB_SAMPLING_HEIGHT-i)*SUB_SAMPLING_WIDTH+j]);
			colors[FindLinesNimbroStyle::White][i][j] = static_cast<int>(camFrame.currentLUT.objects[CC_WHITE].subimg[(SUB_SAMPLING_HEIGHT-i)*SUB_SAMPLING_WIDTH+j]);
			colors[FindLinesNimbroStyle::Green][i][j] = static_cast<int>(camFrame.currentLUT.objects[CC_FIELD].subimg[(SUB_SAMPLING_HEIGHT-i)*SUB_SAMPLING_WIDTH+j]);
			colors[FindLinesNimbroStyle::BallColor][i][j] = static_cast<int>(camFrame.currentLUT.objects[CC_BALL].subimg[(SUB_SAMPLING_HEIGHT-i)*SUB_SAMPLING_WIDTH+j]);
		}
	}
	
	ln_field_line_intersections_num = 0; //<- This is equal to clear the ln_field_line_intersections[].
	ln_Detector->m_NodeBuffer->clear();  //<- Clear NodeBuffer.
// * LiuW: Do the Line/Crossing Detection Processing * //
	
	
	//CM
	//copied the these two lines from processColors
	// * 1. Save these two matrix for Line/Crossing Detection.
	memcpy( ln_iMergedBlack, &colors[ Black ][ 0 ][ 0 ], sizeof ( colors[ Black ] ) );
	memcpy( ln_iPureWhite  , &colors[ White ][ 0 ][ 0 ], sizeof ( colors[ White ] ) );

	// * Reset Boundary. * //
	memset( ln_Detector->m_BoundaryForLine, 5, sizeof ( unsigned int ) * SUB_SAMPLING_WIDTH ); //<- Start from the sixth pixel.
	                                                                            //<- This operation was supposed to be done inside ObjectRecognition::Line
	                                                                            //<- Move it here because of the following preprocessing.
	memcpy( ln_iMergedGreen, &colors[ Green ][ 0 ][ 0 ], sizeof ( ln_iMergedGreen ) );
	NimbroStyleTools::writePGM(ln_iMergedGreen, "pureGreen");
	// * Merge certain area of Black and White into Green for scanning. * //
	for ( int y = 0; y < ( int ) ( SUB_SAMPLING_HEIGHT * 0.2 ); y++ ) {
		for ( int x = 10; x < SUB_SAMPLING_WIDTH - 10; x++ ) {
			ln_iMergedGreen[ y ][ x ] += ln_iMergedBlack[ y ][ x ] + ln_iPureWhite[ y ][ x ];
		}
	}
	
	for ( int y = ( int ) ( SUB_SAMPLING_HEIGHT * 0.2 ); y < ( int ) ( SUB_SAMPLING_HEIGHT * 0.5 ); y++ ) {
		for ( int x = 10; x < SUB_SAMPLING_WIDTH - 10; x++ ) {
			ln_iMergedGreen[ y ][ x ] += ln_iPureWhite[ y ][ x ];
		}
	}
	
	//CM
	//maybe put an ignore where robot is white here 

	// * [0] Scan Boundary of Green Field * //
	// * Rough scan.
	ln_Detector->ScanBoundary( ln_iMergedGreen );
	// * Find a polygon that can cover the Boudary from previous step.
	ln_Detector->FindLocalConvex( );
	// * If it's Front Camera, then only detects the lower part of the image. * //
	
	for ( int i = 0; i < SUB_SAMPLING_WIDTH; i++ ) {
		ln_Detector->m_BoundaryForLine[ i ] = std::min( ( int ) ( .75f * SUB_SAMPLING_HEIGHT ), ln_Detector->m_BoundaryForLine[ i ] );
	}
	ln_Detector->m_Top = std::min( ( int ) ( .75f * SUB_SAMPLING_HEIGHT ), ln_Detector->m_Top );

	// * [1] Smooth with Low-pass Filter * //
	memcpy( ln_iSmoothWM, &ln_iPureWhite[ 0 ][ 0 ], sizeof ( ln_iSmoothWM ) );
	//writeArrayToFile(ln_iSmoothWM,"ln_iSmoothWM.txt"); //<- Same
	ln_Detector->Smooth( ln_iSmoothWM, ln_iTmp );
	//writeArrayToFile(ln_iSmoothWM,"ln_iSmoothWM.txt"); //<- Different starts from here!!
	// * [2] Retrieve Skeleton * //
	ln_Detector->RetrieveSkeleton( ln_iSmoothWM, ln_iSkeletonWM );
// 	NimbroStyleTools::writePGM(ln_iMergedGreen, "merged_classic_green");
// 	NimbroStyleTools::writePGM(ln_iPureWhite, "classic_white");
// 	NimbroStyleTools::writePGM(ln_iSkeletonWM, "classic_skeleton.pgm");
	// * [3] Find Nodes * //
	memcpy( ln_iNodesWM, ln_iSkeletonWM, sizeof ( ln_iNodesWM ) );
	//writeArrayToFile(ln_iNodesWM,"ln_iNodesWM.txt"); //<- Different
	ln_Detector->FindNodes( ln_iNodesWM, ln_iTmp );
	// * [4] Connect Touching Nodes * //
	ln_Detector->ConnectTouchingNodes( ln_iNodesWM );
	// * [5] Find More Nodes * //
	ln_Detector->FindMoreNodes( ln_iSkeletonWM );
	// * [6] Find Close Nodes * //
	ln_Detector->FindCloseNodes( ln_iSmoothWM );
	// * [7] Connect Close Nodes * //
	ln_Detector->ConnectCloseNodes( ln_iSmoothWM );
	std::vector<ObjectRecognition::Line::Node> nodes_tmp = *ln_Detector->m_NodeBuffer;
	ln_Detector->FindL10nLine(ln_white_wo_markers, &ln_field_lines[0], ln_field_lines_num, MAX_FIELDLINES, camFrame);
	*ln_Detector->m_NodeBuffer = nodes_tmp;
	// * [8] Smooth Nodes * //
	ln_Detector->SmoothNodes( );
	// * [9] Delete Nodes * //
	ln_Detector->DeleteNodes( SUB_SAMPLING_HEIGHT );
	
	// * [B] Find Crossings * //
	memcpy( ln_iMergedGreen, &colors[ Green ][ 0 ][ 0 ], sizeof ( ln_iMergedGreen ) );
	for ( int y = 0; y < SUB_SAMPLING_HEIGHT; y++ ) {
		for ( int x = 0; x < SUB_SAMPLING_WIDTH; x++  ) {
			ln_iMergedGreen[ y ][ x ] += colors[ Black ][ y ][ x ];
		}
	}
	//CM
	//change this back if you find a location where BallColor is mergend into Black, White or Green
	ln_Detector->FindCrossings( ln_white_wo_markers, ln_iMergedGreen, camFrame/*, colors[BallColor], this */);
	
	// TODO: remove lines containing L-Xings
	
	//CM 
	//commented out because the OP-Robot does not yet look for cyan or magenta
	// * [ C ] BYPRODUCT - Field Boundary Detection * //
// 	memcpy( ln_iMergedGreen, &colors[ Green ][ 0 ][ 0 ], sizeof ( ln_iMergedGreen ) );
// 	for ( int y = 0; y < SUB_SAMPLING_HEIGHT; y++ ) {
// 		for ( int x = 0; x < SUB_SAMPLING_WIDTH; x++  ) {
// 			ln_iMergedGreen[ y ][ x ] += ( colors[ Cyan ][ y ][ x ] + colors[ Magenta ][ y ][ x ] );
// 		}
// 	}
	ln_Detector->ScanField( colors[ Green ], ln_iPureWhite, colors[ Black ] );


	// * Check out the results, do the post processing and then update them in CV * //


	// * Set confidence based on pre-defined knowledge.
	float ln_Conf   = 0.0; //<- Confidence
	float ln_L_conf = 1.0;
	float ln_T_conf = 1.0;
	float ln_X_conf = 1.0;

	float ln_Angle = 0.0F;

	if ( ln_Detector->m_L_number > 2 ) {
		// too many L-crossings, reduce confidence
		ln_L_conf = 0.4;
	}
	if ( ln_Detector->m_X_number > 0 ) {
		// if we are at the circle, we should not be able to see L-Xings
		ln_L_conf = 0.0;
		//ln_T_conf = 0.6;
	}
	if ( ln_Detector->m_X_number > 2 ) {
		// we should never see >2 X-xings
		ln_X_conf = 0.4;
	}
	if ( ln_Detector->m_T_number > 1 ) {
		// we should never see >1 T-xing
		ln_T_conf = 0.4;
	}

	//if ( 1 == ivNumOfPoles ) { //<- If one Pole has been detected, then set the confidence of detetced L to 0.
	//	ln_L_conf = 0.0;
	//}

	// * Add all the recognized crossings to CV
	//std::cout << "NodeBuffer Size: " << ln_Detector->m_NodeBuffer->size() << std::endl;
	ObjectRecognition::Line::Node_Buffer::const_iterator it;
	for ( it = ln_Detector->m_NodeBuffer->begin(); it != ln_Detector->m_NodeBuffer->end(); it++ ) {

		if ( ObjectRecognition::Line::Node::CT_Unknown == it->crossing_type ) { //<- Not a crossing.
			continue;
		}
		else if ( ObjectRecognition::Line::Node::CT_LXing == it->crossing_type ) { //<- L
			ln_Type = LineXingL;
			//if ( isFrontCamera( ) ) { //<- If it's FrontCamera, then set the confidence of detetced L above middle line to 0.
			//	if ( it->y_pos > SUB_SAMPLING_HEIGHT / 2 ) {
			//		ln_L_conf = 0.0;
			//	}
			//}
			ln_Conf = ln_L_conf * it->conf;
			ln_Angle = it->lambda;
		}
		else if ( ObjectRecognition::Line::Node::CT_TXing == it->crossing_type ) { //<- T
			
			ln_Type = LineXingT;
			ln_Conf = ln_T_conf;

			ln_Angle = it->lambda;

		}
		else if ( ObjectRecognition::Line::Node::CT_XXing == it->crossing_type ) { //<- X

			ln_Type = LineXingX;
			ln_Conf = ln_X_conf;

		}else{
			continue;
		}

		//if ( isFrontCamera( ) ) { //<- If it's FrontCamera, then set the confidence of detetced crossings to 0 if it's above the threshold line.
		//	if ( it->y_pos > ( int ) ( SUB_SAMPLING_HEIGHT * 0.65f ) ) {
		//		ln_Conf = 0.0;
		//	}
		//}

		if ( 0.0f == ln_Conf ) { //<- If confidence is 0, then don't pass it to ln_field_line_intersections.
			continue;
		}

		const Vec2i ln_ImgPos( it->x_pos, it->y_pos ); //<- image position.
		tf::Vector3 vecFromCameraToLandmark = soccervision::pixelCameraCorrection(static_cast<float>(ln_ImgPos.x), static_cast<float>(ln_ImgPos.y));
		tf::Vector3 vecEgoCoordinates = soccervision::compute_ego_position(camFrame.tf_egorot_cameraoptical_OriginBasis, camFrame.tf_egorot_cameraoptical_OriginVector, vecFromCameraToLandmark);			
		Vec2i ln_ImgPos_to_World = Vec2i(static_cast<int>(vecEgoCoordinates.x()), static_cast<int>(vecEgoCoordinates.y()));

		// * Create a world object from the crossing
		EgoObject eo;
		eo.initCartesian( ln_ImgPos_to_World, ln_Conf );
		eo.wotype( ) = ln_Type;
		eo.img_pos( ) = ln_ImgPos;
		eo.setEgoOrientation( ln_Angle );
		std::cout<<"Found Crossing of type: " << ln_Type << " at " << ln_ImgPos.x << " " << ln_ImgPos.x << std::endl;

		ln_field_line_intersections[ ln_field_line_intersections_num++ ] = eo;
		if ( ln_field_line_intersections_num == MAX_FIELDLINE_INTERSECTIONS ) {
			return;
		}

	} // END of FOR

	// post-process found FieldLines.
	static const int maxValidLines = 4;
	int validLinesNum = 0;
	for(int ln_idx = 0;ln_idx < ln_field_lines_num; ++ln_idx){
		ObjectRecognition::FieldLine2& line = ln_field_lines[ln_idx];
		line.wo = NumWorldObjects;
		if(fabs(line.curvature)> 0.15) {
			line.conf = 0;
			// we found a line with strong curvature
			for ( int i = 0; i < ln_field_line_intersections_num; i++ ) {
				// invalidate all L-crossings in the vicinity of this line
				// preventing false detections of L-Xings in the center circle
				EgoObject & eo = ln_field_line_intersections[ i ];
				if(eo.wotype() == LineXingL && NimbroStyleTools::abstandPunktGerade(eo.pos(),line.s_world,line.e_world)<50 )
					eo.conf() = 0.f;
			}
			continue;
		}
		Vec2f diff   = line.e_world - line.s_world;
		if(diff.norm2() < 100*100)
		{   // the line is too short
			line.conf = 0;
			continue;
		}
		line.ang     = atan2(diff.x,diff.y);

		line.lot_world = NimbroStyleTools::lotPunktAufGerade(Vec2f(0,0),line.s_world,line.e_world);
		float distToLot2 = (float)line.lot_world.norm2();
		if(distToLot2 > 200*200 && diff.norm2() < 150*150)
		{   // we are not close to the (short) line.
			line.conf = 0;
			continue;
		}
		validLinesNum++;
		if(validLinesNum>=maxValidLines)
			break;
	}	
	
	//CM
	//push the found crossings into the correct CameraFrame datastruct for visualization
	for(int i = 0; i < ln_field_line_intersections_num; i++)
	{
		HullPoint landmark;
		landmark.x = ln_field_line_intersections[i].img_x();
		landmark.y = ln_field_line_intersections[i].img_y();

		if(ln_field_line_intersections[i].wotype() == LineXingL){
			camFrame.L_landmarks.push_back(landmark);
		}
		else  if(ln_field_line_intersections[i].wotype() == LineXingT){
			camFrame.T_landmarks.push_back(landmark);
		}
		else  if(ln_field_line_intersections[i].wotype() == LineXingX){
			camFrame.X_landmarks.push_back(landmark);
		}
	}
	
}
