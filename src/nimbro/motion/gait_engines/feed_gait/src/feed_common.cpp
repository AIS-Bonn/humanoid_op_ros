// Feedback gait common definitions
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <feed_gait/feed_common.h>

// Namespaces
using namespace feed_gait;

//
// PoseCommand class
//

// Stream insertion operator
std::ostream& feed_gait::operator<<(std::ostream& os, const PoseCommand& poseCmd)
{
	// Stream the position commands
	os << "pos[" << poseCmd.pos.size() << "] = {";
	for(std::size_t i = 0; i < poseCmd.pos.size(); i++)
	{
		if(i != 0) os << ", ";
		os << poseCmd.pos[i];
	}
	os << "}" << std::endl;

	// Stream the effort commands
	os << "eff[" << poseCmd.effort.size() << "] = {";
	for(std::size_t i = 0; i < poseCmd.effort.size(); i++)
	{
		if(i != 0) os << ", ";
		os << poseCmd.effort[i];
	}
	os << "}" << std::endl;

	// Stream the support coefficient commands
	os << "supp = {" << poseCmd.suppCoeff[hk::LEFT] << ", " << poseCmd.suppCoeff[hk::RIGHT] << "}";

	// Return the stream
	return os;
}
// EOF