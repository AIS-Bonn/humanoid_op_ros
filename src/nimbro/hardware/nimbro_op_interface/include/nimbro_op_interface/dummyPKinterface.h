// NimbRo-OP robot hardware interface (dummy parallel kinematics)
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef DUMMYPKINTERFACE_H
#define DUMMYPKINTERFACE_H

// Includes
#include <nimbro_op_interface/robotPKinterface.h>
#include <nimbro_op_interface/dummyinterface.h>

// NimbRo-OP interface namespace
namespace nimbro_op_interface
{
	// DummyPKInterface class
	class DummyPKInterface : public RobotPKInterface, public DummyInterface
	{
	public:
		// Constructor/destructor
		DummyPKInterface() = default;
		virtual ~DummyPKInterface() = default;
	};
}

#endif

// EOF
