// behaviour_exercise_sample.cpp - Philipp Allgeuer - 18/07/13
// Sample solution to the behaviours exercise

// Includes
#include "behaviour_exercise/behaviour_exercise_sample.h"

// Defines
#define TINC 0.4

// Namespaces
using namespace std;
using namespace behaviourexercise;

//
// RobotSC class
//

// Constructor function
void RobotSC::onConstruct()
{
	// Display that the sample is being used
	ROS_WARN("Using sample solution by Philipp Allgeuer (19/07/13)");
}

// Reset function
void RobotSC::reset() // Called when 'run' is checked by the user
{
}

// State controller callbacks
bool RobotSC::preStepCallbackUser()
{
	// Stop the behaviours if the run flag is unset
	if(!m_be_run() && (getCurState()->id != IDLE))
	{
		ROS_INFO_STREAM_THROTTLE(TINC, "Run stop detected - Transferring to idle state...");
		sendGaitCommand(true, 0.0, 0.0, 0.0);
		goToState(NewStateInstance<IdleState>(this));
		return true; // Force a state transition
	}

	// Return that we do not wish to force a state transition
	return false;
}
void RobotSC::postStepCallbackUser()
{
}

//
// IdleState class
//

// Execute callback
action_t IdleState::execute(cycle_t cyc)
{
	// Display current state
	ROS_INFO_STREAM_THROTTLE(TINC, "In state: " << name);

	// Stop the robot from walking
	sc->sendGaitCommand(false);
	
	// Start the behaviours if the run flag is set
	if(sc->m_be_run() && (cyc > 1))
	{
		ROS_INFO_STREAM_THROTTLE(TINC, "Run start detected - Initialising field...");
		sc->initField(); // This sets up the field, including placing the robot, ball, obstacle, etc...
		sc->reset(); // This calls the user-defined reset function
		return sc->goToState(NewStateInstance<SearchForBallState>(sc));
	}
	
	// Stay in the current state
	return HOLD_THIS_STATE;
}

//
// SearchForBallState class
//

// Execute callback
action_t SearchForBallState::execute(cycle_t cyc)
{
	// Display current state
	ROS_INFO_STREAM_THROTTLE(TINC, "In state: " << name);

	// Change state if we have found the ball
	if(sc->haveBall())
	{
		sc->sendGaitCommand(true, 0.0, 0.0, 0.0);
		return sc->goToState(NewStateInstance<ApproachBallState>(sc));
	}

	// Turn on the spot to look for the ball
	sc->sendGaitCommand(true, 0.0, 0.0, 0.6);

	// Stay in the current state
	return HOLD_THIS_STATE;
}

//
// ApproachBallState class
//

// Execute callback
action_t ApproachBallState::execute(cycle_t cyc)
{
	// Display current state
	ROS_INFO_STREAM_THROTTLE(TINC, "In state: " << name);

	// Constants
	const double X_OFFSET = 0.18;    // The target x offset to the ball
	const double Y_OFFSET = -0.09;   // The target y offset to the ball (+ve => left of the mid-sagital plane)
	const double S_OFFSET = 0.35;    // The x offset additional to X_OFFSET at which the robot should start slowing down
	const double CMD_X_WALK = 0.6;   // Gait command for normal walking speed
	const double CMD_X_TIPTOE = 0.2; // Gait command for tip-toeing forward
	const double CMD_X_STOP = 0.0;   // Gait command to halt motion in x direction
	const double GAIN_ALPHA = 3.0;   // Proportional gain for alpha control
	const double GAIN_BETA = 1.2;    // Proportional gain for beta control
	const double FAKE_BETA = 0.5;    // Beta value to set when you haven't seen the goal in a while but still remember what direction it was in
	const double KICK_SLEN = 0.20;   // The maximum distance to allow start kicking
	const double KICK_ALPHA = 0.10;  // Angle accuracy to which to line up the kick to
	const double KICK_BETA = 0.10;   // Angle accuracy to which to line up the goal to before starting a kick

	// Declare variables
	gait_msgs::GaitCommand cmd;
	double alpha, beta, SLen;

	// If you haven't seen the ball for a while then fall back to the search for ball state
	if(!sc->haveBall(1.0))
	{
		sc->sendGaitCommand(true, 0.0, 0.0, 0.0);
		return sc->goToState(NewStateInstance<SearchForBallState>(sc));
	}

	// Save the horizontal offset to the ball in local variables
	double x = sc->ballVector().vec.x();
	double y = sc->ballVector().vec.y();

	// Calculate the length squared of the offset ray vector
	double BLen_sq = x*x + y*y;
	double SLen_sq = BLen_sq - Y_OFFSET*Y_OFFSET;

	// Avoid complex numbers
	if(SLen_sq > 0)
	{
		// Calculate offset ray angle (alpha)
		SLen = sqrt(SLen_sq); // Length of offset ray vector
		alpha = asin((SLen*y-x*Y_OFFSET)/BLen_sq);

		// Calculate offset ray to goal angle (beta)
		if(sc->goalVector().timestamp.isValid())
		{
			Eigen::Vector2d target = sc->goalVector().vec;
			double Ex = target.x() - x;
			double Ey = target.y() - y;
			double Sx = SLen * cos(alpha);
			double Sy = SLen * sin(alpha);
			beta = atan2(Sx*Ey-Sy*Ex,Sx*Ex+Sy*Ey);
			if(!sc->haveGoal(3.0) && sc->haveGoal(10.0))
				beta = (beta > 0.0 ? FAKE_BETA : -FAKE_BETA);
		}
		else
		{
			beta = FAKE_BETA;
		}

		// Calculate dimensionless x parameter
		double u = (SLen - X_OFFSET)/S_OFFSET;

		// Decide on an appropriate action
		if((SLen < KICK_SLEN) && (fabs(alpha) < KICK_ALPHA) && (fabs(beta) < KICK_BETA) && sc->haveBall(0.5) && sc->haveGoal(3.0)) // If your position is good enough for a kick then enter the kick ball state
		{
			sc->sendGaitCommand(true, 0.0, 0.0, 0.0);
			return sc->goToState(NewStateInstance<KickBallState>(sc));
		}
		else if(SLen > X_OFFSET + S_OFFSET)
		{
			cmd.gcvX = CMD_X_WALK;
			cmd.gcvY = 0.0;
			cmd.gcvZ = GAIN_ALPHA * alpha;
			cmd.walk = true;
		}
		else if(SLen > X_OFFSET)
		{
			cmd.gcvX = CMD_X_TIPTOE + u * (CMD_X_WALK - CMD_X_TIPTOE); // Ramp down x velocity for distance control
			cmd.gcvY = (1 - u) * (-GAIN_BETA * beta); // Ramp up y velocity for beta control
			cmd.gcvZ = GAIN_ALPHA * alpha; // Alpha control
			cmd.walk = true;
		}
		else // SLen <= X_OFFSET
		{
			cmd.gcvX = CMD_X_STOP + u * (CMD_X_WALK - CMD_X_TIPTOE); // Keep distance the same
			cmd.gcvY = -GAIN_BETA * beta; // Beta control
			cmd.gcvZ = GAIN_ALPHA * alpha; // Alpha control
			cmd.walk = true;
		}
	}
	else // Ball is closer to robot than |Y_OFFSET|...
	{
		// Define alpha as the angle directly to the ball in this case
		alpha = atan2(y,x);

		// Try to turn towards the ball
		cmd.gcvX = -CMD_X_TIPTOE;
		cmd.gcvY = 0.0;
		cmd.gcvZ = GAIN_ALPHA * alpha;
		cmd.walk = true;
	}

	// Obstacle avoidance
	if(sc->m_be_use_obstacle())
	{
		double obstAvoidStrength = adjustCmdForObstacle(cmd, sc->ballVector().vec, sc->obstVector().vec);
		sc->plotScalar(obstAvoidStrength, "SC/ObstacleAvoidStrength");
	}

	// Publish calculated gait command
	sc->sendGaitCommand(cmd);

	// Stay in the current state
	return HOLD_THIS_STATE;
}

// Obstacle avoidance function
double ApproachBallState::adjustCmdForObstacle(gait_msgs::GaitCommand& cmd, Eigen::Vector2d ballVec, Eigen::Vector2d obstVec)
{
	// Calculate polar coordinates of obstacle and target
	double ballNorm = ballVec.norm();
	double ballAngle = atan2(ballVec.y(), ballVec.x());
	double obstNorm = obstVec.norm();
	double obstAngle = atan2(obstVec.y(), obstVec.x());

	// Calculate additional parameters
	double alpha = obstAngle - ballAngle;
	double perpObstDist = obstNorm * sin(alpha);
// 	double parallelObstDist = obstNorm * cos(alpha);

	// Constants - Obstacle avoidance gains
	const double cmdGainY = 0.7;
	const double cmdGainW = 0.0;

	// Constants - Limits
	const double maxExtraY = 0.7;
	const double maxExtraW = 0.7;
	const double maxPerp = 1.1;
	const double maxAlpha = 1.0;
	const double minObstNorm = 0.15;

	// Variables
	double strength = 0.0; // Strength of zero => no obstacle avoidance performed

	// Calculate whether obstacle is worthy of our attention
	if((fabs(perpObstDist) <= maxPerp) && (fabs(alpha) <= maxAlpha) && (obstNorm <= ballNorm) && (obstNorm >= minObstNorm))
	{
		// Calculate how much of an obstruction the obstacle is (and in which direction)
		strength = 1.0 - 0.6*fabs(perpObstDist)/maxPerp - 0.4*obstNorm/ballNorm;
		if(strength < 0.0) strength = 0.0;
		if(alpha >= 0.0)
			strength = -strength;

		// Calculate what should be added to the gait command vector in order to avoid the obstacle
		double extraCmdY = cmdGainY * strength;
		double extraCmdW = cmdGainW * strength;

		// Saturate by how much the gait command vector is modified
		if(extraCmdY >  maxExtraY) extraCmdY =  maxExtraY;
		if(extraCmdY < -maxExtraY) extraCmdY = -maxExtraY;
		if(extraCmdW >  maxExtraW) extraCmdW =  maxExtraW;
		if(extraCmdW < -maxExtraW) extraCmdW = -maxExtraW;

		// Modify the gait command vector as appropriate
		cmd.gcvY += extraCmdY;
		cmd.gcvZ += extraCmdW;
	}

	// Return value 
	return strength;
}

//
// KickBallState class
//

// Activate callback
void KickBallState::activate(cycle_t cyc)
{
	// Stop the robot from walking
	sc->sendGaitCommand(false);

	// Initialise variables
	haveKicked = false;
}

// Execute callback
action_t KickBallState::execute(cycle_t cyc)
{
	// Display current state
	ROS_INFO_STREAM_THROTTLE(TINC, "In state: " << name);

	// If the robot has stopped walking then start the kick
	if(sc->robotIsStanding())
	{
		if(haveKicked)
			return sc->goToState(NewStateInstance<SearchForBallState>(sc));
		if(sc->playRightKick())
			haveKicked = true;
	}

	// Stay in the current state
	return HOLD_THIS_STATE;
}
// EOF