#include "RobotControl.h"
#include "Config.h"
#include "Command.h"
#include "State.h"

// The RobotControl class implements a classic agent function action = f(percept) in a
// sense, act, learn loop. sense() updates the world model of the agent given the latest
// percept. act() generates a new action for the agent to execute in its current situation
// given some kind of a goal or command. And learn() generates rewards and updates its
// internal policy, which is eventually used by the act() function to generate better
// actions. The robot control has a robot model, which needs to be changed in case the
// controlled robot changes during runtime.

namespace indep_cpg_gait
{
	RobotControl::RobotControl()
	{
		lastUpdateTimestamp = 0;
		lastStartTimestamp = 0;
	}

	// Initialization cascade after construction.
	void RobotControl::init()
	{
		motionLayer.init();
	}

	// The sense method updates the world model of the agent given the most recent percept from
	// the physical world. In other frameworks this is called the sensor upstream, where low
	// level sensor input is processed to high level symbols.
	void RobotControl::sense(Percept& percept)
	{
		state.fusedAngle = percept.fusedAngle;
		state.DfusedAngle = percept.DfusedAngle;
		state.command = command;


		// Apply the sensed joint angles and the estimated fused angle to the robot model
		// with the walk algorithm. The walk algorithm also tracks the support leg and the
		// robot's position, so that it can be used as a motion model. This will also
		// calculate the higher level abstract poses.
		robotModel.walk(percept.pose, percept.fusedAngle);

		state.rxAction = robotModel.getAction();
		state.supportLegSign = robotModel.supportLegSign;
		state.supportExchange = robotModel.supportExchange;

		// Extract the feature vectors needed for the limp features.
		// Same for trunk angle state and sole angle state.
// 		Vec comVector = robotModel.comVector();
// 		Vec trunkAngle = robotModel.trunkAngle();
// 		Vec soleAngle = robotModel.soleAngle();

		// Filter the limp states.
// 		LimpState comState = comFilter.update(comVector);
// 		LimpState trunkAngleState = trunkAngleFilter.update(trunkAngle);
// 		LimpState soleAngleState = soleAngleFilter.update(soleAngle);
// 
// 		state.comState = comState;
// 		state.trunkAngleState = trunkAngleState;
// 		state.soleAngleState = soleAngleState;

		// Tick and step counters.
		state.frameId++;
		if (state.supportExchange)
			state.stepId++;
	}

	// Generates an action for the agent given the current state of the world model, goals, commands and a learned policy.
	// In another framework this is called the actuator downstream. The downstream is organized in hierarchical layers that
	// compute from abstract to more and more specific actions down to motor commands.
	Pose RobotControl::act()
	{
		// Execute the motion layer.
		motionLayer.update();
		Action action = motionLayer.actuate();

		// The motion layer and the motions inside don't have access to the robot model, because in case
		// the model is changed, all motion objects would have to be notified too. That would make it
		// more difficult to add new motions. Instead, the motion layer creates a mixture of motions at
		// different abstraction levels (joint positions, abstract poses, inverse poses). These targets
		// are summed up here to a single joint level pose. They are summed up from top to bottom, so
		// that targets at different abstraction levels can be combined with each other.
		robotModel.setInversePose(action.inversePose);
	//	robotModel.setAbstractPose(action.abstractPose);
		robotModel.addAbstractPose(action.abstractPose);
	//	robotModel.setPose(action.pose);
		robotModel.addPose(action.pose);

		state.txAction = robotModel.getAction();

		return state.txAction.pose;
	}

	// Learn. Not implemented yet.
	void RobotControl::learn()
	{

	}

}