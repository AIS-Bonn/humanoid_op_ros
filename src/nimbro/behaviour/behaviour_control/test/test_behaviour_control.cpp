// Unit testing of the Behaviour Control Framework
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <../test/test_behaviour_control.h>

// Namespaces
using namespace std;
using namespace behaviourcontroltest;

//
// MyM class
//

// Constructor
MyM::MyM() : BehaviourManager("MyM")
{
	// Create child layers
	IL = new RosIL(this);
	L1 = new MyL1(this);
	L2 = new MyL2(this);

	// Initialise test variables
	initCheckSum = 0LL;
}

// Destructor
MyM::~MyM()
{
	// Delete child layers
	delete IL;
	delete L1;
	delete L2;
}

// Error managements functions
void MyM::reportErrorUser(const std::string& msg, bool fatal, const std::string& funcName, const std::string& fileName, int line)
{
	// Display a message to the console for now
	DISPLAY_NEWLINE;
	DISPLAY(cout << "\r" << setColour(Attr::RED) << (fatal ? "Fatal error: " : "Non-fatal error: ") << msg << setColour() << endl);
	DISPLAY(cout << "\rIn function " << setColour(Attr::MAGENTA) << funcName << "()" << setColour() << " at " << setColour(Attr::MAGENTA) << "line " << line << setColour() << " in " << setColour(Attr::MAGENTA) << fileName << setColour() << endl);
}

//
// RosIL class
//

// Constructor
RosIL::RosIL(MyM* M) : BehaviourLayer(M, "RosIL", true) , M(M)
{
	// Create sensor and actuator managers
	SM = new RosILSM(this);
	AM = new RosILAM(this);
}

// Destructor
RosIL::~RosIL()
{
	// Delete sensor and actuator managers
	delete SM;
	delete AM;
}

//
// RosILSM class
//

// Constructor
RosILSM::RosILSM(RosIL* L) : SensorManager(L), L(L), M(L->M)
	, mode  (this, "MyL2/mode")
	, count (this, "MyL2/count")
	, target(this, "MyL2/target")
{
}

// External data write function
void RosILSM::writeExternalData()
{
	// Display to the user what would be written
	DISPLAY(cout << "    Writing " << mode.read()   << " to the 'mode_out' ROS topic" << endl);
	DISPLAY(cout << "    Writing " << count.read()  << " to the 'count' ROS topic"    << endl);
	DISPLAY(cout << "    Writing " << target.read() << " to the 'target' ROS topic"   << endl);
}

//
// RosILAM class
//

// Constructor
RosILAM::RosILAM(RosIL* L) : ActuatorManager(L), L(L), M(L->M)
	, mode   (this, "ROS/mode")
	, targetX(this, "ROS/targetX")
	, targetY(this, "ROS/targetY")
{
}

// External data read function
void RosILAM::readExternalData()
{
	// Pretend we just read some values from a ROS topic
	int new_mode = mode.read() + 4;
	float new_targetX = targetX.read() - 1.50F;
	float new_targetY = targetY.read() + 5.12F;

	// Display to the user what we supposedly read
	DISPLAY(cout << "    Read " << new_mode    << " from the 'mode_in' ROS topic" << endl);
	DISPLAY(cout << "    Read " << new_targetX << " from the 'targetX' ROS topic" << endl);
	DISPLAY(cout << "    Read " << new_targetY << " from the 'targetY' ROS topic" << endl);

	// Write the new data on the internal actuators
	mode.writeHard(new_mode);
	targetX.writeHard(new_targetX);
	targetY.writeHard(new_targetY);
}

//
// MyL1 class
//

// Constructor
MyL1::MyL1(MyM* M) : BehaviourLayer(M, "MyL1") , M(M)
{
	// Create sensor and actuator managers
	SM = new MyL1SM(this);
	AM = new MyL1AM(this);

	// Create child behaviours
	B1 = new MyB1(this);
	B2 = new MyB2(this);

	// Add inhibitions
	addInhibition(B2, B1);
}

// Destructor
MyL1::~MyL1()
{
	// Delete child behaviours
	delete B1;
	delete B2;

	// Delete sensor and actuator managers
	delete SM;
	delete AM;
}

// Initialisation function
ret_t MyL1::init()
{
	DISPLAY(std::cout << "    Initialising class " << name << std::endl);
	M->initCheckSum = 3*M->initCheckSum + 1;
	AM->mode.writeHard(true);
	AM->xgoal.writeHard(42);
	AM->vgoal.writeHard(42.42);
	return RET_OK;
}

//
// MyL1SM class
//

// Constructor
MyL1SM::MyL1SM(MyL1* L) : SensorManager(L), L(L), M(L->M)
	, mode   (this, "ROS/mode")
	, targetX(this, "ROS/targetX")
	, targetY(this, "ROS/targetY")
{
}

//
// MyL1AM class
//

// Constructor
MyL1AM::MyL1AM(MyL1* L) : ActuatorManager(L), L(L), M(L->M)
	, mode (this, "MyL1/mode")
	, xgoal(this, "MyL1/xgoal")
	, vgoal(this, "MyL1/vgoal")
{
}

//
// MyL2 class
//

// Constructor
MyL2::MyL2(MyM* M) : BehaviourLayer(M, "MyL2") , M(M)
{
	// Create sensor and actuator managers
	SM = new MyL2SM(this);
	AM = new MyL2AM(this);

	// Create child behaviours
	B3 = new MyB3(this);
	B4 = new MyB4(this);
}

// Destructor
MyL2::~MyL2()
{
	// Delete child behaviours
	delete B3;
	delete B4;

	// Delete sensor and actuator managers
	delete SM;
	delete AM;
}

//
// MyL2SM class
//

// Constructor
MyL2SM::MyL2SM(MyL2* L) : SensorManager(L), L(L), M(L->M)
	, mode (this, "MyL1/mode")
	, xgoal(this, "MyL1/xgoal")
	, vgoal(this, "MyL1/vgoal")
{
}

//
// MyL2AM class
//

// Constructor
MyL2AM::MyL2AM(MyL2* L) : ActuatorManager(L), L(L), M(L->M)
	, mode  (this, "MyL2/mode")
	, count (this, "MyL2/count")
	, target(this, "MyL2/target")
{
}

//
// Unit Tests
//

// Test entire Behaviour Control Framework (the classes are too interlinked to really be able to test things individually)
TEST(BehaviourControlTest, test_BehaviourControlFrameWork)
{
	// Declare variables
	behaviourcontrol::index_t i; // The namespace is given explicitly because it helps the syntax highlighting in my editor... :-/ (it has trouble working out what TEST(...) expands to above)

	// Declare start of testing
	DISPLAY_NEWLINE;
	DISPLAY(FText(Attr::GREEN) << "Starting test of Behaviour Control Framework...");

	//
	// Automated testing
	//

	// Construct a MyM behaviour manager object
	DISPLAY(cout << "Constructing MyM behaviour manager..." << endl);
	MyM m;

	// Retrieve pointers to created layers
	DISPLAY(cout << "Retrieving child layers..." << endl);
	MyL1* l1 = m.L1;
	MyL2* l2 = m.L2;

	// Retrieve pointers to created behaviours
	DISPLAY(cout << "Retrieving child behaviours of the retrieved layers..." << endl);
	MyB1* b1 = l1->B1;
	MyB2* b2 = l1->B2;
	MyB3* b3 = l2->B3;
	MyB4* b4 = l2->B4;

	// Retrieve base pointers
	DISPLAY(cout << "Checking retrieval of object base pointers..." << endl);
	BehaviourManager* mptr = m.getBasePtr();
	BehaviourLayer* l1ptr = l1->getBasePtr();
	BehaviourLayer* l2ptr = l2->getBasePtr();
	Behaviour* b1ptr = b1->getBasePtr();
	Behaviour* b2ptr = b2->getBasePtr();
	Behaviour* b3ptr = b3->getBasePtr();
	Behaviour* b4ptr = b4->getBasePtr();

	// Check the parent pointers
	DISPLAY(cout << "Checking parent base pointers..." << endl);
	EXPECT_EQ(mptr, l1->MBase);
	EXPECT_EQ(mptr, l2->MBase);
	EXPECT_EQ(mptr, b1->MBase);
	EXPECT_EQ(mptr, b2->MBase);
	EXPECT_EQ(mptr, b3->MBase);
	EXPECT_EQ(mptr, b4->MBase);
	EXPECT_EQ(l1ptr, b1ptr->LBase);
	EXPECT_EQ(l1ptr, b2ptr->LBase);
	EXPECT_EQ(l2ptr, b3ptr->LBase);
	EXPECT_EQ(l2ptr, b4ptr->LBase);

	// Check that the naming is consistent (this is actually checking that the pointers are the same, which is a stronger assertion than just the strings being equal!)
	DISPLAY(cout << "Checking consistent object naming..." << endl);
	EXPECT_EQ(m.name, l1->Mname);
	EXPECT_EQ(m.name, l2->Mname);
	EXPECT_EQ(m.name, b1->Mname);
	EXPECT_EQ(m.name, b2->Mname);
	EXPECT_EQ(m.name, b3->Mname);
	EXPECT_EQ(m.name, b4->Mname);
	EXPECT_EQ(l1->name, b1ptr->Lname);
	EXPECT_EQ(l1->name, b2ptr->Lname);
	EXPECT_EQ(l2->name, b3ptr->Lname);
	EXPECT_EQ(l2->name, b4ptr->Lname);

	// Check sensor manager parent pointers
	DISPLAY(cout << "Checking sensor manager parent base pointers..." << endl);
	if(l1->hasSM())
	{
		EXPECT_EQ(mptr, l1->SM->MBase);
		EXPECT_EQ(l1ptr, l1->SM->LBase);
	}
	if(l2->hasSM())
	{
		EXPECT_EQ(mptr, l2->SM->MBase);
		EXPECT_EQ(l2ptr, l2->SM->LBase);
	}

	// Check actuator manager parent pointers
	DISPLAY(cout << "Checking actuator manager parent base pointers..." << endl);
	if(l1->hasAM())
	{
		EXPECT_EQ(mptr, l1->AM->MBase);
		EXPECT_EQ(l1ptr, l1->AM->LBase);
	}
	if(l2->hasAM())
	{
		EXPECT_EQ(mptr, l2->AM->MBase);
		EXPECT_EQ(l2ptr, l2->AM->LBase);
	}

	// Initialise the architecture
	DISPLAY(cout << "Initialising the architecture..." << endl);
	EXPECT_EQ(0LL, m.initCheckSum);
	ret_t ret = m.initialiseArchitecture();
	if(ret != RET_OK) { DISPLAY(cout << "    --> Could not initialise the architecture! (Error " << ret << ")" << endl); }
	else { DISPLAY(cout << "    Initialisation succeeded!" << endl); }
	ASSERT_EQ(RET_OK, ret);

	// Check that initialisation happened as expected
	DISPLAY(cout << "Checking the order in which initialisation occurred..." << endl);
	EXPECT_EQ(708054185729LL, m.initCheckSum); // This will only be correct if initialisation happened in exactly the right order

	// Check that subsequent initialisation attempts are ignored
	DISPLAY(cout << "Checking that a duplicate call to initialisation is ignored (expect non-fatal error)..." << endl);
	ASSERT_EQ(RET_ALREADY_INITIALISED, m.initialiseArchitecture());
	if(!m.hadError()) m.clearErrorStatus();
	DISPLAY_NEWLINE;

	// Call the step function one or more times
	for(i = 0;i < 2;i++)
	{
		DISPLAY(cout << "Calling the step function..." << endl);
		m.step();
	}

	// End of automated testing
	DISPLAY(cout << "Done!" << endl);

	// Display the architecture tree for the user
	DISPLAY_NEWLINE;
	DISPLAY_NO_PAD(cout << m.toString("    "));

	// Declare end of testing
	DISPLAY(FText(Attr::GREEN) << "Completed test of Behaviour Control Framework!\n");
}

// Main function
int main(int argc, char **argv)
{
	// Run the required tests
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF