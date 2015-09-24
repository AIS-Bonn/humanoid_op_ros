#ifndef MOTIONLAYER_H
#define MOTIONLAYER_H

#include "Halt.h"
#include "DynamicGait.h"
#include "Fall.h"
#include "MotionInterfaceTest.h"
#include "MotionTest.h"

namespace indep_cpg_gait
{
	class MotionLayer
	{

	public:

		MotionLayer();
	~MotionLayer(){};

	Action actuate();
	void init();
	void update();
	void reset();

	bool walking;

	private:

	bool robotDown;

		Halt halt;
		Fall fall;

	public:

		MotionInterfaceTest motionInterfaceTest;
		MotionTest motionTest;
		DynamicGait gait;
	};
}
#endif // MOTIONLAYER_H
