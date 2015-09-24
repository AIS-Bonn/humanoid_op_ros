// Contains extra dimensions not present in the robot model
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DIMENSIONS_H
#define DIMENSIONS_H

namespace nimbro_op_model
{

//! Z offset of ankle coordinate system to bottom of the foot
const double ANKLE_Z_HEIGHT = 0.039; // m

//! X offset of geometric center of foot in ankle joint coordinates
const double FOOT_CENTER_OFFSET_X = 0.1049 - 0.208/2.0; // m

//! Y offset of geometric center of foot in ankle joint coordinates
const double FOOT_CENTER_OFFSET_Y_LEFT = 0.077 - 0.132/2.0; // m

//! Foot size
const double FOOT_WIDTH = 0.132;
const double FOOT_LENGTH = 0.208;

}

#endif
