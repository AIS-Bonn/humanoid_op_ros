#include "State.h"
#include "Globals.h"
#include "Config.h"
#include <QMutexLocker>

/*
 * The State is a globally accessible, bufferable object with built in introspection
 * capabilities.
 *
 * The primary function of the state is to provide a central and easily accessible place for
 * the storage of system variables and objects, such as raw sensor values, higher level
 * sensors values, outputs of behaviors, and debug values. The state
 * is globally accessible, because it is needed and used everywhere in the RC, the
 * behaviors, and the visualization parts. It makes it easy to provide read and write
 * access to system variables, so that they don't have to be passed around between
 * functions, and it doesn't require to declare pointers in every logical unit that
 * wants to have access to the state. Anywhere in the code, just write for example
 * state.time to access the current time. All relevant state members are public. You can
 * read and write them anywhere without cumbersome getters and setters. Also, the state
 * object gets changed frequently during development. New sensors are introduced, behaviors
 * are changed and more and more details are visualized. All this comes with the constant
 * need to create or abandon state members. It would slow us down if we had to maintain
 * numerous accessor functions that in most cases do nothing more than just passing on a
 * value.
 *
 * Here is an overview of what the state object is used for in each cycle. First, raw
 * sensor values are written into the state by the serial RobotInterface and the computer
 * vision. Then the raw sensor values are used to compute higher level sensors which are
 * added to the state by the robot control. The hierarchical behaviors read the sensor data,
 * make decisions, write intermediate results and debug vectors into the state along with
 * the motor commands, the final result of the behavior architecture. The GUI reads pretty
 * much all state members for visualization, but it doesn't write anything into the state.
 *
 * The state object is not thread safe. Even though the vision thread, the RC thread
 * and the GUI thread all access it at the same time. The reason why this works is due
 * to the following facts. No state member is written from two different threads. Raw sensor
 * values are only written by the RobotInterface or by the computer vision. The RC writes the
 * higher level sensors and behaviors write their output. The GUI only reads state members.
 * Therefore, it cannot happen that two threads try to concurrently write the same state
 * member, where there result would be undefined. On x86 read and write operations on
 * up to 8 aligned bytes (such as doubles) are atomic. That means that a half
 * written double does not exist, which would be an undefined number that can do bad
 * things to your robot. It is possible, however, that structures such a Vec3f are
 * partially written by one thread, while another thread is reading it. Atomic doubles
 * prevent the appearance of corrupt numbers, but it can happen that for example that x and y
 * components are not in sync with the z component of the vector. As we expect sensor values
 * to change continuously, and sensor values are the only input to the system after all, this
 * is not a problem that would result in system failure.
 *
 * The state object is bufferable. This means that upon calling state.buffer(), the state
 * object will add a copy of itself into an internal history with bounded length. To access
 * a historical state object use for example state[-1].supportLegSign, which would tell you
 * on which leg the robot was standing one buffered state before the current one. You can
 * provide positive numbers as well, they are the same as using negative ones. Of course,
 * the state is buffered once in each cycle. The few private members the state object needs
 * for keeping the history and to implement the introspection features are static, so that
 * they don't get copied when the state is buffered into history.
 *
 * The introspection capabilities are used for visualization. All state members can be displayed
 * as time series on a graph. To select which variables you want to see, you want to click on
 * their names, so they need to be associated with a string (key). Unlike Java, C++ does not
 * have a native introspection interface, so the class meta data (keys) have to be provided
 * manually. With each new or deleted state member, the header, the constructor, the init()
 * method, and optionally the reset() method have to be changed. The header to declare the
 * variable, the constructor to initialize it, the init() method to register a key and a reference
 * to the member variable and the reset() method if you want the variable to be reset to an
 * initial value when the RC is restarted. Of course it is displeasing having to go through
 * four places just to declare one variable, but at least it's all in one object and there is
 * an easy pattern to follow. On the upside, what you get for the extra effort is index based and
 * key based read access to the state members, such as state(0), which would be the program time,
 * or state("txAction.pose.leftLegPose.hip.y") for the commanded position of the left hip y
 * servo. These access methods are used by the GUI to map an index or a string selected by the
 * user to state members for selective visualization. The introspection interface and the history
 * are combinable. Here are some examples.
 *
 * state[-10](4) gives you the step counter from 10 cycles ago, because the step counter is the 4th
 * member of State counting from 0.
 *
 * state[11]("fusedAngle.x") gives you the lateral fused angle 11 cycles ago.
 */

namespace indep_cpg_gait
{
	State state;

	// These members are static so that buffering into history does not create copies.
	QMutex State::mutex;
	QStringList State::memberNames;
	QList<off_t> State::memberOffsets;
	QList<QString> State::memberTypes;
	QList<State> State::history;

	// In the constructor members should be initialized where needed.
	State::State()
	{
		frameId = 0;
		stepId = 0;
		time = 0;
		lastIterationTime = 0;
		lastExecutionTime = 0;
		supportExchange = false;
		supportLegSign = 0;
		gaitFrequency = 0.0;
		gaitPhase = 0.0;
		debug = 0;
	}

	State::~State()
	{

	}

	// The init() method should be called after construction of the state object.
	// Here, all state object members are registered to build a descriptor meta
	// structure that allows index and key based access to the member values.
	// If you don't want to see a certain state member on the gui, there is no
	// need to register it.
	void State::init()
	{
	//	registerMember("frameId", &frameId);
	//	registerMember("stepId", &stepId);
	//	registerMember("time", &time);
// 		registerMember("lastIterationTime", &lastIterationTime);
// 		registerMember("lastExecutionTime", &lastExecutionTime);
/*
		registerMember("debug", &debug);

		registerMember("fusedAngle.x", &fusedAngle.x);
		registerMember("fusedAngle.y", &fusedAngle.y);
		registerMember("DfusedAngle.x", &DfusedAngle.x);
		registerMember("DfusedAngle.y", &DfusedAngle.y);*/

// 		registerMember("comState.x", &comState.x);
// 		registerMember("comState.vx", &comState.vx);
// 		registerMember("comState.y", &comState.y);
// 		registerMember("comState.vy", &comState.vy);
// 		registerMember("comState.energyX", &comState.energyX);
// 		registerMember("comState.energyY", &comState.energyY);
	//	registerMember("comState.supportLegSign", &comState.supportLegSign);

// 		registerMember("trunkAngleState.x", &trunkAngleState.x);
// 		registerMember("trunkAngleState.vx", &trunkAngleState.vx);
// 		registerMember("trunkAngleState.y", &trunkAngleState.y);
// 		registerMember("trunkAngleState.vy", &trunkAngleState.vy);
// 		registerMember("trunkAngleState.energyX", &trunkAngleState.energyX);
// 		registerMember("trunkAngleState.energyY", &trunkAngleState.energyY);
// 	//	registerMember("trunkAngleState.supportLegSign", &trunkAngleState.supportLegSign);
// 
// 		registerMember("soleAngleState.x", &soleAngleState.x);
// 		registerMember("soleAngleState.vx", &soleAngleState.vx);
// 		registerMember("soleAngleState.y", &soleAngleState.y);
// 		registerMember("soleAngleState.vy", &soleAngleState.vy);
// 		registerMember("soleAngleState.energyX", &soleAngleState.energyX);
// 		registerMember("soleAngleState.energyY", &soleAngleState.energyY);
	//	registerMember("soleAngleState.supportLegSign", &soleAngleState.supportLegSign);

// 		registerMember("supportExchange", &supportExchange);
// 		registerMember("supportLegSign", &supportLegSign);
// 
// 		registerMember("gaitFrequency", &gaitFrequency);
// 		registerMember("gaitPhase", &gaitPhase);
// 
// 		registerMember("gcvTarget.x", &gcvTarget.x);
// 		registerMember("gcvTarget.y", &gcvTarget.y);
// 		registerMember("gcvTarget.z", &gcvTarget.z);
// 		registerMember("gcv.x", &gcv.x);
// 		registerMember("gcv.y", &gcv.y);
// 		registerMember("gcv.z", &gcv.z);
// 		registerMember("commandGCV.x", &command.GCV.x);
// 		registerMember("commandGCV.y", &command.GCV.y);
// 		registerMember("commandGCV.z", &command.GCV.z);
// 
// 
// 		registerMember("txAction.inversePose.leftArmPose.handPosition.x", &txAction.inversePose.leftArmPose.handPosition.x);
// 		registerMember("txAction.inversePose.leftArmPose.handPosition.y", &txAction.inversePose.leftArmPose.handPosition.y);
// 		registerMember("txAction.inversePose.leftArmPose.handPosition.z", &txAction.inversePose.leftArmPose.handPosition.z);
// 		registerMember("txAction.inversePose.leftArmPose.compliance", &txAction.inversePose.leftArmPose.compliance);
// 		registerMember("txAction.inversePose.rightArmPose.handPosition.x", &txAction.inversePose.rightArmPose.handPosition.x);
// 		registerMember("txAction.inversePose.rightArmPose.handPosition.y", &txAction.inversePose.rightArmPose.handPosition.y);
// 		registerMember("txAction.inversePose.rightArmPose.handPosition.z", &txAction.inversePose.rightArmPose.handPosition.z);
// 		registerMember("txAction.inversePose.rightArmPose.compliance", &txAction.inversePose.rightArmPose.compliance);
// 		registerMember("txAction.inversePose.leftLegPose.footPosition.x", &txAction.inversePose.leftLegPose.footPosition.x);
// 		registerMember("txAction.inversePose.leftLegPose.footPosition.y", &txAction.inversePose.leftLegPose.footPosition.y);
// 		registerMember("txAction.inversePose.leftLegPose.footPosition.z", &txAction.inversePose.leftLegPose.footPosition.z);
// 		registerMember("txAction.inversePose.leftLegPose.footAngle.x", &txAction.inversePose.leftLegPose.footAngle.x);
// 		registerMember("txAction.inversePose.leftLegPose.footAngle.y", &txAction.inversePose.leftLegPose.footAngle.y);
// 		registerMember("txAction.inversePose.leftLegPose.footAngle.z", &txAction.inversePose.leftLegPose.footAngle.z);
// 		registerMember("txAction.inversePose.leftLegPose.compliance", &txAction.inversePose.leftLegPose.compliance);
// 		registerMember("txAction.inversePose.rightLegPose.footPosition.x", &txAction.inversePose.rightLegPose.footPosition.x);
// 		registerMember("txAction.inversePose.rightLegPose.footPosition.y", &txAction.inversePose.rightLegPose.footPosition.y);
// 		registerMember("txAction.inversePose.rightLegPose.footPosition.z", &txAction.inversePose.rightLegPose.footPosition.z);
// 		registerMember("txAction.inversePose.rightLegPose.footAngle.x", &txAction.inversePose.rightLegPose.footAngle.x);
// 		registerMember("txAction.inversePose.rightLegPose.footAngle.y", &txAction.inversePose.rightLegPose.footAngle.y);
// 		registerMember("txAction.inversePose.rightLegPose.footAngle.z", &txAction.inversePose.rightLegPose.footAngle.z);
// 		registerMember("txAction.inversePose.rightLegPose.compliance", &txAction.inversePose.rightLegPose.compliance);
// 
// 		registerMember("txAction.abstractPose.leftArmPose.armAngle.x", &txAction.abstractPose.leftArmPose.armAngle.x);
// 		registerMember("txAction.abstractPose.leftArmPose.armAngle.y", &txAction.abstractPose.leftArmPose.armAngle.y);
// 		registerMember("txAction.abstractPose.leftArmPose.armAngle.z", &txAction.abstractPose.leftArmPose.armAngle.z);
// 		registerMember("txAction.abstractPose.leftArmPose.armExtension", &txAction.abstractPose.leftArmPose.armExtension);
// 		registerMember("txAction.abstractPose.leftArmPose.compliance", &txAction.abstractPose.leftArmPose.compliance);
// 		registerMember("txAction.abstractPose.rightArmPose.armAngle.x", &txAction.abstractPose.rightArmPose.armAngle.x);
// 		registerMember("txAction.abstractPose.rightArmPose.armAngle.y", &txAction.abstractPose.rightArmPose.armAngle.y);
// 		registerMember("txAction.abstractPose.rightArmPose.armAngle.z", &txAction.abstractPose.rightArmPose.armAngle.z);
// 		registerMember("txAction.abstractPose.rightArmPose.armExtension", &txAction.abstractPose.rightArmPose.armExtension);
// 		registerMember("txAction.abstractPose.rightArmPose.compliance", &txAction.abstractPose.rightArmPose.compliance);
// 		registerMember("txAction.abstractPose.leftLegPose.legAngle.x", &txAction.abstractPose.leftLegPose.legAngle.x);
// 		registerMember("txAction.abstractPose.leftLegPose.legAngle.y", &txAction.abstractPose.leftLegPose.legAngle.y);
// 		registerMember("txAction.abstractPose.leftLegPose.legAngle.z", &txAction.abstractPose.leftLegPose.legAngle.z);
// 		registerMember("txAction.abstractPose.leftLegPose.legExtension", &txAction.abstractPose.leftLegPose.legExtension);
// 		registerMember("txAction.abstractPose.leftLegPose.footAngle.x", &txAction.abstractPose.leftLegPose.footAngle.x);
// 		registerMember("txAction.abstractPose.leftLegPose.footAngle.y", &txAction.abstractPose.leftLegPose.footAngle.y);
// 		registerMember("txAction.abstractPose.leftLegPose.compliance", &txAction.abstractPose.leftLegPose.compliance);
// 		registerMember("txAction.abstractPose.rightLegPose.legAngle.x", &txAction.abstractPose.rightLegPose.legAngle.x);
// 		registerMember("txAction.abstractPose.rightLegPose.legAngle.y", &txAction.abstractPose.rightLegPose.legAngle.y);
// 		registerMember("txAction.abstractPose.rightLegPose.legAngle.z", &txAction.abstractPose.rightLegPose.legAngle.z);
// 		registerMember("txAction.abstractPose.rightLegPose.legExtension", &txAction.abstractPose.rightLegPose.legExtension);
// 		registerMember("txAction.abstractPose.rightLegPose.footAngle.x", &txAction.abstractPose.rightLegPose.footAngle.x);
// 		registerMember("txAction.abstractPose.rightLegPose.footAngle.y", &txAction.abstractPose.rightLegPose.footAngle.y);
// 		registerMember("txAction.abstractPose.rightLegPose.compliance", &txAction.abstractPose.rightLegPose.compliance);
// 
// 		registerMember("txAction.pose.headPose.neck.y", &txAction.pose.headPose.neck.y);
// 		registerMember("txAction.pose.headPose.neck.z", &txAction.pose.headPose.neck.z);
// 		registerMember("txAction.pose.headPose.neck.compliance", &txAction.pose.headPose.neck.compliance);
// 		registerMember("txAction.pose.leftArmPose.shoulder.x", &txAction.pose.leftArmPose.shoulder.x);
// 		registerMember("txAction.pose.leftArmPose.shoulder.y", &txAction.pose.leftArmPose.shoulder.y);
// 		registerMember("txAction.pose.leftArmPose.shoulder.z", &txAction.pose.leftArmPose.shoulder.z);
// 		registerMember("txAction.pose.leftArmPose.shoulder.compliance", &txAction.pose.leftArmPose.shoulder.compliance);
// 		registerMember("txAction.pose.leftArmPose.elbow.y", &txAction.pose.leftArmPose.elbow.y);
// 		registerMember("txAction.pose.leftArmPose.elbow.compliance", &txAction.pose.leftArmPose.elbow.compliance);
// 		registerMember("txAction.pose.leftLegPose.hip.x", &txAction.pose.leftLegPose.hip.x);
// 		registerMember("txAction.pose.leftLegPose.hip.y", &txAction.pose.leftLegPose.hip.y);
// 		registerMember("txAction.pose.leftLegPose.hip.z", &txAction.pose.leftLegPose.hip.z);
// 		registerMember("txAction.pose.leftLegPose.hip.compliance", &txAction.pose.leftLegPose.hip.compliance);
// 		registerMember("txAction.pose.leftLegPose.knee.y", &txAction.pose.leftLegPose.knee.y);
// 		registerMember("txAction.pose.leftLegPose.knee.compliance", &txAction.pose.leftLegPose.knee.compliance);
// 		registerMember("txAction.pose.leftLegPose.ankle.x", &txAction.pose.leftLegPose.ankle.x);
// 		registerMember("txAction.pose.leftLegPose.ankle.y", &txAction.pose.leftLegPose.ankle.y);
// 		registerMember("txAction.pose.leftLegPose.ankle.compliance", &txAction.pose.leftLegPose.ankle.compliance);
// 		registerMember("txAction.pose.rightArmPose.shoulder.x", &txAction.pose.rightArmPose.shoulder.x);
// 		registerMember("txAction.pose.rightArmPose.shoulder.y", &txAction.pose.rightArmPose.shoulder.y);
// 		registerMember("txAction.pose.rightArmPose.shoulder.z", &txAction.pose.rightArmPose.shoulder.z);
// 		registerMember("txAction.pose.rightArmPose.shoulder.compliance", &txAction.pose.rightArmPose.shoulder.compliance);
// 		registerMember("txAction.pose.rightArmPose.elbow.y", &txAction.pose.rightArmPose.elbow.y);
// 		registerMember("txAction.pose.rightArmPose.elbow.compliance", &txAction.pose.rightArmPose.elbow.compliance);
// 		registerMember("txAction.pose.rightLegPose.hip.x", &txAction.pose.rightLegPose.hip.x);
// 		registerMember("txAction.pose.rightLegPose.hip.y", &txAction.pose.rightLegPose.hip.y);
// 		registerMember("txAction.pose.rightLegPose.hip.z", &txAction.pose.rightLegPose.hip.z);
// 		registerMember("txAction.pose.rightLegPose.hip.compliance", &txAction.pose.rightLegPose.hip.compliance);
// 		registerMember("txAction.pose.rightLegPose.knee.y", &txAction.pose.rightLegPose.knee.y);
// 		registerMember("txAction.pose.rightLegPose.knee.compliance", &txAction.pose.rightLegPose.knee.compliance);
// 		registerMember("txAction.pose.rightLegPose.ankle.x", &txAction.pose.rightLegPose.ankle.x);
// 		registerMember("txAction.pose.rightLegPose.ankle.y", &txAction.pose.rightLegPose.ankle.y);
// 		registerMember("txAction.pose.rightLegPose.ankle.compliance", &txAction.pose.rightLegPose.ankle.compliance);
// 
// 
// 		registerMember("rxAction.inversePose.leftArmPose.handPosition.x", &rxAction.inversePose.leftArmPose.handPosition.x);
// 		registerMember("rxAction.inversePose.leftArmPose.handPosition.y", &rxAction.inversePose.leftArmPose.handPosition.y);
// 		registerMember("rxAction.inversePose.leftArmPose.handPosition.z", &rxAction.inversePose.leftArmPose.handPosition.z);
// 		registerMember("rxAction.inversePose.leftArmPose.compliance", &rxAction.inversePose.leftArmPose.compliance);
// 		registerMember("rxAction.inversePose.rightArmPose.handPosition.x", &rxAction.inversePose.rightArmPose.handPosition.x);
// 		registerMember("rxAction.inversePose.rightArmPose.handPosition.y", &rxAction.inversePose.rightArmPose.handPosition.y);
// 		registerMember("rxAction.inversePose.rightArmPose.handPosition.z", &rxAction.inversePose.rightArmPose.handPosition.z);
// 		registerMember("rxAction.inversePose.rightArmPose.compliance", &rxAction.inversePose.rightArmPose.compliance);
// 		registerMember("rxAction.inversePose.leftLegPose.footPosition.x", &rxAction.inversePose.leftLegPose.footPosition.x);
// 		registerMember("rxAction.inversePose.leftLegPose.footPosition.y", &rxAction.inversePose.leftLegPose.footPosition.y);
// 		registerMember("rxAction.inversePose.leftLegPose.footPosition.z", &rxAction.inversePose.leftLegPose.footPosition.z);
// 		registerMember("rxAction.inversePose.leftLegPose.footAngle.x", &rxAction.inversePose.leftLegPose.footAngle.x);
// 		registerMember("rxAction.inversePose.leftLegPose.footAngle.y", &rxAction.inversePose.leftLegPose.footAngle.y);
// 		registerMember("rxAction.inversePose.leftLegPose.footAngle.z", &rxAction.inversePose.leftLegPose.footAngle.z);
// 		registerMember("rxAction.inversePose.leftLegPose.compliance", &rxAction.inversePose.leftLegPose.compliance);
// 		registerMember("rxAction.inversePose.rightLegPose.footPosition.x", &rxAction.inversePose.rightLegPose.footPosition.x);
// 		registerMember("rxAction.inversePose.rightLegPose.footPosition.y", &rxAction.inversePose.rightLegPose.footPosition.y);
// 		registerMember("rxAction.inversePose.rightLegPose.footPosition.z", &rxAction.inversePose.rightLegPose.footPosition.z);
// 		registerMember("rxAction.inversePose.rightLegPose.footAngle.x", &rxAction.inversePose.rightLegPose.footAngle.x);
// 		registerMember("rxAction.inversePose.rightLegPose.footAngle.y", &rxAction.inversePose.rightLegPose.footAngle.y);
// 		registerMember("rxAction.inversePose.rightLegPose.footAngle.z", &rxAction.inversePose.rightLegPose.footAngle.z);
// 		registerMember("rxAction.inversePose.rightLegPose.compliance", &rxAction.inversePose.rightLegPose.compliance);
// 
// 		registerMember("rxAction.abstractPose.leftArmPose.armAngle.x", &rxAction.abstractPose.leftArmPose.armAngle.x);
// 		registerMember("rxAction.abstractPose.leftArmPose.armAngle.y", &rxAction.abstractPose.leftArmPose.armAngle.y);
// 		registerMember("rxAction.abstractPose.leftArmPose.armAngle.z", &rxAction.abstractPose.leftArmPose.armAngle.z);
// 		registerMember("rxAction.abstractPose.leftArmPose.armExtension", &rxAction.abstractPose.leftArmPose.armExtension);
// 		registerMember("rxAction.abstractPose.leftArmPose.compliance", &rxAction.abstractPose.leftArmPose.compliance);
// 		registerMember("rxAction.abstractPose.rightArmPose.armAngle.x", &rxAction.abstractPose.rightArmPose.armAngle.x);
// 		registerMember("rxAction.abstractPose.rightArmPose.armAngle.y", &rxAction.abstractPose.rightArmPose.armAngle.y);
// 		registerMember("rxAction.abstractPose.rightArmPose.armAngle.z", &rxAction.abstractPose.rightArmPose.armAngle.z);
// 		registerMember("rxAction.abstractPose.rightArmPose.armExtension", &rxAction.abstractPose.rightArmPose.armExtension);
// 		registerMember("rxAction.abstractPose.rightArmPose.compliance", &rxAction.abstractPose.rightArmPose.compliance);
// 		registerMember("rxAction.abstractPose.leftLegPose.legAngle.x", &rxAction.abstractPose.leftLegPose.legAngle.x);
// 		registerMember("rxAction.abstractPose.leftLegPose.legAngle.y", &rxAction.abstractPose.leftLegPose.legAngle.y);
// 		registerMember("rxAction.abstractPose.leftLegPose.legAngle.z", &rxAction.abstractPose.leftLegPose.legAngle.z);
// 		registerMember("rxAction.abstractPose.leftLegPose.legExtension", &rxAction.abstractPose.leftLegPose.legExtension);
// 		registerMember("rxAction.abstractPose.leftLegPose.footAngle.x", &rxAction.abstractPose.leftLegPose.footAngle.x);
// 		registerMember("rxAction.abstractPose.leftLegPose.footAngle.y", &rxAction.abstractPose.leftLegPose.footAngle.y);
// 		registerMember("rxAction.abstractPose.leftLegPose.compliance", &rxAction.abstractPose.leftLegPose.compliance);
// 		registerMember("rxAction.abstractPose.rightLegPose.legAngle.x", &rxAction.abstractPose.rightLegPose.legAngle.x);
// 		registerMember("rxAction.abstractPose.rightLegPose.legAngle.y", &rxAction.abstractPose.rightLegPose.legAngle.y);
// 		registerMember("rxAction.abstractPose.rightLegPose.legAngle.z", &rxAction.abstractPose.rightLegPose.legAngle.z);
// 		registerMember("rxAction.abstractPose.rightLegPose.legExtension", &rxAction.abstractPose.rightLegPose.legExtension);
// 		registerMember("rxAction.abstractPose.rightLegPose.footAngle.x", &rxAction.abstractPose.rightLegPose.footAngle.x);
// 		registerMember("rxAction.abstractPose.rightLegPose.footAngle.y", &rxAction.abstractPose.rightLegPose.footAngle.y);
// 		registerMember("rxAction.abstractPose.rightLegPose.compliance", &rxAction.abstractPose.rightLegPose.compliance);
// 
// 		registerMember("rxAction.pose.headPose.neck.y", &rxAction.pose.headPose.neck.y);
// 		registerMember("rxAction.pose.headPose.neck.z", &rxAction.pose.headPose.neck.z);
// 		registerMember("rxAction.pose.headPose.neck.compliance", &rxAction.pose.headPose.neck.compliance);
// 		registerMember("rxAction.pose.leftArmPose.shoulder.x", &rxAction.pose.leftArmPose.shoulder.x);
// 		registerMember("rxAction.pose.leftArmPose.shoulder.y", &rxAction.pose.leftArmPose.shoulder.y);
// 		registerMember("rxAction.pose.leftArmPose.shoulder.z", &rxAction.pose.leftArmPose.shoulder.z);
// 		registerMember("rxAction.pose.leftArmPose.shoulder.compliance", &rxAction.pose.leftArmPose.shoulder.compliance);
// 		registerMember("rxAction.pose.leftArmPose.elbow.y", &rxAction.pose.leftArmPose.elbow.y);
// 		registerMember("rxAction.pose.leftArmPose.elbow.compliance", &rxAction.pose.leftArmPose.elbow.compliance);
// 		registerMember("rxAction.pose.leftLegPose.hip.x", &rxAction.pose.leftLegPose.hip.x);
// 		registerMember("rxAction.pose.leftLegPose.hip.y", &rxAction.pose.leftLegPose.hip.y);
// 		registerMember("rxAction.pose.leftLegPose.hip.z", &rxAction.pose.leftLegPose.hip.z);
// 		registerMember("rxAction.pose.leftLegPose.hip.compliance", &rxAction.pose.leftLegPose.hip.compliance);
// 		registerMember("rxAction.pose.leftLegPose.knee.y", &rxAction.pose.leftLegPose.knee.y);
// 		registerMember("rxAction.pose.leftLegPose.knee.compliance", &rxAction.pose.leftLegPose.knee.compliance);
// 		registerMember("rxAction.pose.leftLegPose.ankle.x", &rxAction.pose.leftLegPose.ankle.x);
// 		registerMember("rxAction.pose.leftLegPose.ankle.y", &rxAction.pose.leftLegPose.ankle.y);
// 		registerMember("rxAction.pose.leftLegPose.ankle.compliance", &rxAction.pose.leftLegPose.ankle.compliance);
// 		registerMember("rxAction.pose.rightArmPose.shoulder.x", &rxAction.pose.rightArmPose.shoulder.x);
// 		registerMember("rxAction.pose.rightArmPose.shoulder.y", &rxAction.pose.rightArmPose.shoulder.y);
// 		registerMember("rxAction.pose.rightArmPose.shoulder.z", &rxAction.pose.rightArmPose.shoulder.z);
// 		registerMember("rxAction.pose.rightArmPose.shoulder.compliance", &rxAction.pose.rightArmPose.shoulder.compliance);
// 		registerMember("rxAction.pose.rightArmPose.elbow.y", &rxAction.pose.rightArmPose.elbow.y);
// 		registerMember("rxAction.pose.rightArmPose.elbow.compliance", &rxAction.pose.rightArmPose.elbow.compliance);
// 		registerMember("rxAction.pose.rightLegPose.hip.x", &rxAction.pose.rightLegPose.hip.x);
// 		registerMember("rxAction.pose.rightLegPose.hip.y", &rxAction.pose.rightLegPose.hip.y);
// 		registerMember("rxAction.pose.rightLegPose.hip.z", &rxAction.pose.rightLegPose.hip.z);
// 		registerMember("rxAction.pose.rightLegPose.hip.compliance", &rxAction.pose.rightLegPose.hip.compliance);
// 		registerMember("rxAction.pose.rightLegPose.knee.y", &rxAction.pose.rightLegPose.knee.y);
// 		registerMember("rxAction.pose.rightLegPose.knee.compliance", &rxAction.pose.rightLegPose.knee.compliance);
// 		registerMember("rxAction.pose.rightLegPose.ankle.x", &rxAction.pose.rightLegPose.ankle.x);
// 		registerMember("rxAction.pose.rightLegPose.ankle.y", &rxAction.pose.rightLegPose.ankle.y);
// 		registerMember("rxAction.pose.rightLegPose.ankle.compliance", &rxAction.pose.rightLegPose.ankle.compliance);

	//	qDebug() << memberNames;
	//	qDebug() << memberTypes;
	//	qDebug() << memberOffsets;
	}

	// Buffers the current state values into the history.
	void State::buffer()
	{
		QMutexLocker locker(&mutex);
		history.push_front(*this);
	}

	// Returns the amount of buffered historical state objects.
	int State::size()
	{
		QMutexLocker locker(&mutex);
		return history.size();
	}

	// Returns a historical state object.
	// i = 0 returns the current state.
	// i = -1 (or 1) returns the state object from the iteration before and so on.
	// To get the oldest known state you must request i = (+/-)state.size().
	// If abs(i) > state.size() the oldest known state object is returned.
	// Yes, unlike usual arrays, this operator handles items indexes from 0 to state.size()
	// instead of 0 to state.size()-1.
	State& State::operator[](int i)
	{
		QMutexLocker locker(&mutex);

		if (i == 0 or history.empty())
			return *this;

		i = qMin(qAbs(i), (int)history.size()) - 1;
		return history[i];
	}

	// Returns the value of the ith member of this object.
	double State::operator()(int i)
	{
	//	QMutexLocker locker(&mutex);

		if (memberTypes[i].startsWith('i'))
			return (double)(*((int*)((unsigned long int)this+memberOffsets[i])));
		else if (memberTypes[i].startsWith('b'))
			return (double)(*((bool*)((unsigned long int)this+memberOffsets[i])));
		return *((double*)((unsigned long int)this+memberOffsets[i]));
	}

	// Returns the value of the member that was registered with the given key.
	// If no member was registered with this key, 0 is returned.
	// This method is somewhat expensive due to a linear search through the member
	// names. To speed this up, a new implementation based on a hash table is required.
	double State::operator()(QString key)
	{
		int memberIndex = 0;
		bool memberFound = false;
		while (not memberFound)
			if (memberNames[memberIndex++] == key)
				memberFound = true;

		if (memberFound)
			return this->operator()(memberIndex-1);

		return 0;
	}

}