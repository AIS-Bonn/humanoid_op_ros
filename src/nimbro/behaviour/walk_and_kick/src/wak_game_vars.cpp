// Walk and kick: Game variables
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/wak_game_vars.h>
#include <walk_and_kick/wak_beh_manager.h>

// Namespaces
using namespace walk_and_kick;

//
// GameVars class
//

// Ball target type characters and names
const char GameVars::BallTargetTypeChar[BTT_COUNT] = {
	'U',
	'G',
	'P',
	'C'
};
const std::string GameVars::BallTargetTypeName[BTT_COUNT] = {
	"Unknown Target",
	"Goal",
	"Pose",
	"Compass"
};

// Foot selection names
const std::string GameVars::FootSelectionName[3] = {
	"LeftFoot",
	"EitherFoot",
	"RightFoot"
};

// Reset function
void GameVars::reset()
{
	// Reset the data members
	forceBehStateByID = WAKBehManager::BS_UNKNOWN;
	suggestFoot = FS_EITHER_FOOT;
	dribbleIfPossible = false;
	kickIfPossible = false;
	diveIfPossible = DD_NONE;
	ballTargetConf = 0.0f;
	ballTargetDir.setZero();
	ballTargetWedge = 0.0f;
	ballTargetType = BTT_UNKNOWN;
	targetPose.setZero();
	targetPoseTol = -1.0f;
	targetPoseValid = false;
}

// Coerce enumeration values function
bool GameVars::coerceEnums()
{
	// Save the original values
	FootSelection suggestFootOld = suggestFoot;
	DiveDirection diveIfPossibleOld = diveIfPossible;
	BTType ballTargetTypeOld = ballTargetType;

	// Coerce any enumeration values that are out of range
	if(suggestFoot > FS_RIGHT_FOOT)
		suggestFoot = FS_RIGHT_FOOT;
	else if(suggestFoot < FS_LEFT_FOOT)
		suggestFoot = FS_LEFT_FOOT;
	if(!ballTargetTypeValid(ballTargetType))
		ballTargetType = BTT_UNKNOWN;
	if(!diveDirectionValid(diveIfPossible))
		diveIfPossible = DD_NONE;

	// Return whether any enumeration value had to be coerced
	return (suggestFoot != suggestFootOld || ballTargetType != ballTargetTypeOld || diveIfPossible != diveIfPossibleOld);
}
// EOF