// Header for unit testing of the Behaviour Control Framework
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef TEST_BEHAVIOUR_CONTROL_H
#define TEST_BEHAVIOUR_CONTROL_H

// Include configurations
#ifndef VERBOSE_TEST
//#define VERBOSE_TEST // Comment this line out to inhibit printing to console
#endif /* VERBOSE_TEST */

// Includes
#include <iostream>
#include <gtest/gtest.h>
#include <test_utilities/test_utilities.h>
#include <behaviour_control/behaviour_control.h>

// Behaviour control test namespace
namespace behaviourcontroltest
{
	// Namespaces
	using namespace testutilities;
	using namespace behaviourcontrol;

	//
	// Class declarations
	//

	// MyM manager
	class MyM;

	// RosIL layer
	class RosIL;
	class RosILSM;
	class RosILAM;

	// MyL1 layer
	class MyL1;
	class MyL1SM;
	class MyL1AM;
	class MyB1;
	class MyB2;

	// MyL2 layer
	class MyL2;
	class MyL2SM;
	class MyL2AM;
	class MyB3;
	class MyB4;

	//
	// MyM manager
	//

	// MyM class
	class MyM : public BehaviourManager
	{
	public:
		// Constructors
		MyM();
		virtual ~MyM();

		// Child layers
		RosIL* IL;
		MyL1* L1;
		MyL2* L2;

		// Function overrides
		virtual ret_t init() { DISPLAY(std::cout << "    Initialising class " << name << std::endl); initCheckSum = 2*initCheckSum + 1; return RET_OK; }
		virtual void preStepCallback()  { DISPLAY(std::cout << "    Pre-step callback in manager '"  << name << "'" << std::endl); }
		virtual void postStepCallback() { DISPLAY(std::cout << "    Post-step callback in manager '" << name << "'" << std::endl); }

		// Error notification function
		virtual void reportErrorUser(const std::string& msg, bool fatal, const std::string& funcName, const std::string& fileName, int line);

		// Test variables
		long long initCheckSum;
	};

	//
	// RosIL layer
	//

	// RosIL class
	class RosIL : public BehaviourLayer
	{
	public:
		// Constructors
		RosIL(MyM* M);
		virtual ~RosIL();

		// Parent manager
		MyM* const M;

		// Sensor and actuator managers
		RosILSM* SM;
		RosILAM* AM;

		// Function overrides
		virtual ret_t init() { DISPLAY(std::cout << "    Initialising class " << name << std::endl); M->initCheckSum = 13*M->initCheckSum + 1; return RET_OK; }
		virtual void update() { DISPLAY(std::cout << "    User-updating layer '" << name << "'" << std::endl); }
		virtual void postExecuteCallback() { DISPLAY(std::cout << "    Post-executing layer '" << name << "'" << std::endl); }
	};

	// RosILSM class
	class RosILSM : public SensorManager
	{
	public:
		// Constructors
		RosILSM(RosIL* L);

		// Parent layer and manager
		RosIL* const L;
		MyM* const M;

		// RosIL sensors (the data values read internally from these sensors are published on ROS topics)
		SensorInt mode;
		SensorLong count;
		SensorULong target;

		// Function overrides
		virtual ret_t init() { DISPLAY(std::cout << "    Initialising class " << LBase->name << "SM" << std::endl); M->initCheckSum = 14*M->initCheckSum + 1; return RET_OK; }
		virtual void writeExternalData();
	};

	// RosILAM class
	class RosILAM : public ActuatorManager
	{
	public:
		// Constructors
		RosILAM(RosIL* L);

		// Parent layer and manager
		RosIL* const L;
		MyM* const M;

		// RosIL actuators (the data values read from ROS topics are published internally by these actuators)
		ActuatorInt mode;
		ActuatorFloat targetX;
		ActuatorFloat targetY;

		// Function overrides
		virtual ret_t init() { DISPLAY(std::cout << "    Initialising class " << LBase->name << "AM" << std::endl); M->initCheckSum = 15*M->initCheckSum + 1; return RET_OK; }
		virtual void readExternalData();
	};

	//
	// MyL1 layer
	//

	// MyL1 class
	class MyL1 : public BehaviourLayer
	{
	public:
		// Constructors
		MyL1(MyM* M);
		virtual ~MyL1();

		// Parent manager
		MyM* const M;

		// Child behaviours
		MyB1* B1;
		MyB2* B2;

		// Sensor and actuator managers
		MyL1SM* SM;
		MyL1AM* AM;

		// Function overrides
		virtual ret_t init();
		virtual void update() { DISPLAY(std::cout << "    User-updating layer '" << name << "'" << std::endl); }
		virtual void postExecuteCallback() { DISPLAY(std::cout << "    Post-executing layer '" << name << "'" << std::endl); }
	};

	// MyL1SM class
	class MyL1SM : public SensorManager
	{
	public:
		// Constructors
		MyL1SM(MyL1* L);

		// Parent layer and manager
		MyL1* const L;
		MyM* const M;

		// MyL1 sensors
		SensorInt mode;
		SensorFloat targetX;
		SensorFloat targetY;

		// Function overrides
		virtual ret_t init() { DISPLAY(std::cout << "    Initialising class " << LBase->name << "SM" << std::endl); M->initCheckSum = 9*M->initCheckSum + 1; return RET_OK; }
	};

	// MyL1AM class
	class MyL1AM : public ActuatorManager
	{
	public:
		// Constructors
		MyL1AM(MyL1* L);

		// Parent layer and manager
		MyL1* const L;
		MyM* const M;

		// MyL1 actuators
		ActuatorBool mode;
		ActuatorInt xgoal;
		ActuatorDouble vgoal;

		// Function overrides
		virtual ret_t init() { DISPLAY(std::cout << "    Initialising class " << LBase->name << "AM" << std::endl); M->initCheckSum = 10*M->initCheckSum + 1; return RET_OK; }
	};

	// MyB1 class
	class MyB1 : public Behaviour
	{
	public:
		// Constructors
		MyB1(MyL1* L) : Behaviour(L, "MyB1") , L(L), M(L->M), SM(L->SM), AM(L->AM) {}

		// Parent layer and manager
		MyL1* const L;
		MyM* const M;

		// Sensor and actuator managers
		MyL1SM* const SM;
		MyL1AM* const AM;

		// Function overrides
		virtual ret_t init() { DISPLAY(std::cout << "    Initialising class " << name << std::endl); M->initCheckSum = 4*M->initCheckSum + 1; return RET_OK; }
		virtual void update() { DISPLAY(std::cout << "    User-updating behaviour '" << name << "'" << std::endl); }
		virtual level_t computeActivationLevel() { return 0.6F; }
		virtual void execute() { AM->vgoal.write(SM->targetX.read() + 5.0, this); AM->xgoal.write(SM->mode.read() + 3, this); DISPLAY(std::cout << "    Executing behaviour '" << name << "'" << std::endl); }
	};

	// MyB2 class
	class MyB2 : public Behaviour
	{
	public:
		// Constructors
		MyB2(MyL1* L) : Behaviour(L, "MyB2") , L(L), M(L->M), SM(L->SM), AM(L->AM) {}

		// Parent layer and manager
		MyL1* const L;
		MyM* const M;

		// Sensor and actuator managers
		MyL1SM* const SM;
		MyL1AM* const AM;

		// Function overrides
		virtual ret_t init() { DISPLAY(std::cout << "    Initialising class " << name << std::endl); M->initCheckSum = 5*M->initCheckSum + 1; return RET_OK; }
		virtual void update() { DISPLAY(std::cout << "    User-updating behaviour '" << name << "'" << std::endl); }
		virtual level_t computeActivationLevel() { return 0.9F; }
		virtual void execute() { AM->vgoal.write(SM->targetX.read() + 10.0, this); AM->xgoal.write(SM->mode.read() - 7, this); DISPLAY(std::cout << "    Executing behaviour '" << name << "'" << std::endl); }
	};

	//
	// MyL2 layer
	//

	// MyL2 class
	class MyL2 : public BehaviourLayer
	{
	public:
		// Constructors
		MyL2(MyM* M);
		virtual ~MyL2();

		// Parent manager
		MyM* const M;

		// Child behaviours
		MyB3* B3;
		MyB4* B4;

		// Sensor and actuator managers
		MyL2SM* SM;
		MyL2AM* AM;

		// Function overrides
		virtual ret_t init() { DISPLAY(std::cout << "    Initialising class " << name << std::endl); M->initCheckSum = 6*M->initCheckSum + 1; return RET_OK; }
		virtual void update() { DISPLAY(std::cout << "    User-updating layer '" << name << "'" << std::endl); }
		virtual void postExecuteCallback() { DISPLAY(std::cout << "    Post-executing layer '" << name << "'" << std::endl); }
	};

	// MyL2SM class
	class MyL2SM : public SensorManager
	{
	public:
		// Constructors
		MyL2SM(MyL2* L);

		// Parent layer and manager
		MyL2* const L;
		MyM* const M;

		// MyL2 sensors
		SensorBool mode;
		SensorInt xgoal;
		SensorDouble vgoal;

		// Function overrides
		virtual ret_t init() { DISPLAY(std::cout << "    Initialising class " << LBase->name << "SM" << std::endl); M->initCheckSum = 11*M->initCheckSum + 1; return RET_OK; }
	};

	// MyL2AM class
	class MyL2AM : public ActuatorManager
	{
	public:
		// Constructors
		MyL2AM(MyL2* L);

		// Parent layer and manager
		MyL2* const L;
		MyM* const M;

		// MyL2 actuators
		ActuatorInt mode;
		ActuatorLong count;
		ActuatorULong target;

		// Function overrides
		virtual ret_t init() { DISPLAY(std::cout << "    Initialising class " << LBase->name << "AM" << std::endl); M->initCheckSum = 12*M->initCheckSum + 1; return RET_OK; }
	};

	// MyB3 class
	class MyB3 : public Behaviour
	{
	public:
		// Constructors
		MyB3(MyL2* L) : Behaviour(L, "MyB3") , L(L), M(L->M), SM(L->SM), AM(L->AM) {}

		// Parent layer and manager
		MyL2* const L;
		MyM* const M;

		// Sensor and actuator managers
		MyL2SM* const SM;
		MyL2AM* const AM;

		// Function overrides
		virtual ret_t init() { DISPLAY(std::cout << "    Initialising class " << name << std::endl); M->initCheckSum = 7*M->initCheckSum + 1; return RET_OK; }
		virtual void update() { DISPLAY(std::cout << "    User-updating behaviour '" << name << "'" << std::endl); }
		virtual level_t computeActivationLevel() { return 0.3F; }
		virtual void execute() { AM->mode.write(SM->mode.read(), this); AM->count.write(SM->xgoal.read()*2, this); AM->target.write(SM->xgoal.read() + 323, this); DISPLAY(std::cout << "    Executing behaviour '" << name << "'" << std::endl); }
	};

	// MyB4 class
	class MyB4 : public Behaviour
	{
	public:
		// Constructors
		MyB4(MyL2* L) : Behaviour(L, "MyB4") , L(L), M(L->M), SM(L->SM), AM(L->AM) {}

		// Parent layer and manager
		MyL2* const L;
		MyM* const M;

		// Sensor and actuator managers
		MyL2SM* const SM;
		MyL2AM* const AM;

		// Function overrides
		virtual ret_t init() { DISPLAY(std::cout << "    Initialising class " << name << std::endl); M->initCheckSum = 8*M->initCheckSum + 1; return RET_OK; }
		virtual void update() { DISPLAY(std::cout << "    User-updating behaviour '" << name << "'" << std::endl); }
		virtual level_t computeActivationLevel() { return 0.1F; }
		virtual void execute() { AM->count.write(SM->xgoal.read()*4, this); AM->target.write(SM->xgoal.read() + 176, this); DISPLAY(std::cout << "    Executing behaviour '" << name << "'" << std::endl); }
	};
}

#endif /* TEST_BEHAVIOUR_CONTROL_H */
// EOF