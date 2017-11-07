// NimbRo-OP robot hardware interface (parallel kinematics)
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef ROBOTPKINTERFACE_H
#define ROBOTPKINTERFACE_H

// Includes
#include <nimbro_op_interface/robotinterface.h>

// NimbRo-OP interface namespace
namespace nimbro_op_interface
{
	// RobotPKInterface class
	class RobotPKInterface : public virtual RobotInterface
	{
	public:
		// Constructor/destructor
		RobotPKInterface() = default;
		virtual ~RobotPKInterface() = default;

		// Virtual function overrides
		virtual bool initJointDependencies();

	protected:
		// Joint dependency functions
		bool addPKJointDependencies();

		// Parallel kinematics set class
		class PKSet
		{
		public:
			// Constructor
			PKSet() : joint() {}

			// Parallel kinematics joint enumeration
			enum PKJoint
			{
				START = 0,
				PAR_START = START,
				PAR_TP = PAR_START,  // Thigh pitch
				PAR_SP,              // Shank pitch
				PAR_AR,              // Ankle roll
				PAR_END,
				SER_START = PAR_END,
				SER_HP = SER_START,  // Hip pitch
				SER_KP,              // Knee pitch
				SER_AP,              // Ankle pitch
				SER_AR,              // Ankle roll
				SER_END,
				END = SER_END,
				COUNT = END
			};
			static const std::string PKJointName[COUNT];

			// List of joints in the parallel kinematics set
			DXLJoint* joint[COUNT];
		};
	};
}

#endif
// EOF
