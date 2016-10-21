// Enumeration which is used to define particular mouse events
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#pragma once

namespace rqt_vision_module
{

enum MouseEvent
{
	Move,
	
	LeftClick,
	LeftRelease,
	
	RightClick,
	RightRelease,
	
	MiddleClick,
	MiddleRelease,
	
	MiddleRotatedForward,   // Away from the user
	MiddleRotatedBackwards, // To the user
};

}
