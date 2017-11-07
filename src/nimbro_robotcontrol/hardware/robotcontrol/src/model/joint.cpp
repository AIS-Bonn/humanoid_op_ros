// Joint information
// Authors: Max Schwarz <max.schwarz@uni-bonn.de>
//          Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <robotcontrol/model/joint.h>
#include <ros/console.h>
#include <cmath>

// Robotcontrol namespace
namespace robotcontrol
{

//
// Joint command class
//

// Default velocity and acceleration limits
double Joint::Command::velLimit = 0.0;
double Joint::Command::accLimit = 0.0;

// Constructor
Joint::Command::Command()
 : pos(0.0)
 , vel(0.0)
 , acc(0.0)
 , rawPos(0.0)
 , effort(0.0)
 , raw(false)
 , m_updateType(SFT_NONE)
 , m_newPos(0.0)
 , m_newVel(0.0)
 , m_newAcc(0.0)
 , m_updateCount(0)
{
}

// Reset the internal buffers of the derivative filters
void Joint::Command::resetDerivs()
{
	// Reset the filter buffers as required
	m_dev_posToVel.reset();
	m_dev_posToAcc.reset();
	m_dev_velToAcc.reset();
}

// Start a joint command update phase
void Joint::Command::startUpdatePhase()
{
	// Reset the update variables
	m_updateType = SFT_NONE;
	m_newPos = 0.0;
	m_newVel = 0.0;
	m_newAcc = 0.0;
	m_updateCount = 0;
}

// Stop a joint command update phase, and actually perform the update
void Joint::Command::stopUpdatePhase(double deltaT)
{
	// Protect against numerical corruption
	bool badCommand = !(std::isfinite(m_newPos) && std::isfinite(m_newVel) && std::isfinite(m_newAcc));
	if(badCommand)
		ROS_WARN_THROTTLE(0.5, "A non-finite joint command (%.3f, %.3f, %.3f) was detected => Ignoring the command!", m_newPos, m_newVel, m_newAcc);

	// Actually update the joint command on a "he with the last word wins" basis
	if(m_updateCount <= 0 || m_updateType <= SFT_NONE || m_updateType >= SFT_COUNT || badCommand)
		actuallySetFromPos(deltaT, pos);
	else if(m_updateType == SFT_POS)
		actuallySetFromPos(deltaT, m_newPos);
	else if(m_updateType == SFT_POSUNSMOOTHED)
		actuallySetFromPosUnsmoothed(deltaT, m_newPos);
	else if(m_updateType == SFT_POSVEL)
		actuallySetFromPosVel(deltaT, m_newPos, m_newVel);
	else if(m_updateType == SFT_POSVELACC)
		actuallySetFromPosVelAcc(m_newPos, m_newVel, m_newAcc);
	else // Should never happen...
		actuallySetFromPos(deltaT, pos);
}

/**
 * @brief This method calculates the vel and acc fields using Savitzky-Golay smoothed differentiation.
 *
 * @param newPos Goal position (rad)
 **/
void Joint::Command::setFromPos(double newPos)
{
	// Save that this method was called
	m_updateType = SFT_POS;
	m_newPos = newPos;
	m_newVel = 0.0;
	m_newAcc = 0.0;
	m_updateCount++;
}

/**
 * @brief This method calculates the vel and acc fields using direct difference equations, use with care!
 *
 * @param newPos Goal position (rad)
 **/
void Joint::Command::setFromPosUnsmoothed(double newPos)
{
	// Save that this method was called
	m_updateType = SFT_POSUNSMOOTHED;
	m_newPos = newPos;
	m_newVel = 0.0;
	m_newAcc = 0.0;
	m_updateCount++;
}

/**
 * @brief This method calculates the acc field using Savitzky-Golay smoothed differentiation of the given velocity values.
 *
 * @param newPos Goal position (rad)
 * @param newVel Goal velocity (rad/s)
 **/
void Joint::Command::setFromPosVel(double newPos, double newVel)
{
	// Save that this method was called
	m_updateType = SFT_POSVEL;
	m_newPos = newPos;
	m_newVel = newVel;
	m_newAcc = 0.0;
	m_updateCount++;
}

/**
 * This function directly sets the target position, velocity and acceleration values.
 *
 * @param newPos Goal position (rad)
 * @param newVel Goal velocity (rad/s)
 * @param newAcc Goal acceleration (rad/s^2)
 **/
void Joint::Command::setFromPosVelAcc(double newPos, double newVel, double newAcc)
{
	// Save that this method was called
	m_updateType = SFT_POSVELACC;
	m_newPos = newPos;
	m_newVel = newVel;
	m_newAcc = newAcc;
	m_updateCount++;
}

// Actually set the command based on a position
void Joint::Command::actuallySetFromPos(double deltaT, double newPos)
{
	// Handle invalid time step
	if(deltaT <= 0.0)
	{
		pos = newPos;
		return;
	}

	// Calculate velocity based on position values
	m_dev_posToVel.put(newPos);
	double newVel = m_dev_posToVel.value() / deltaT;

	// Calculate acceleration based on position values
	m_dev_posToAcc.put(newPos);
	double newAcc = m_dev_posToAcc.value() / (deltaT * deltaT);

	// Update the joint command variables
	pos = newPos;
	vel = newVel;
	acc = newAcc;
	
	// Put data in unused filters in case setFromPos overloads are mixed
	m_dev_velToAcc.put(newVel);
}

// Actually set the command based on a position, but in an unsmoothed manner
void Joint::Command::actuallySetFromPosUnsmoothed(double deltaT, double newPos)
{
	// Handle invalid time step
	if(deltaT <= 0)
	{
		pos = newPos;
		return;
	}

	// Calculate velocity using a direct difference equation
	double newVel = (newPos - pos) / deltaT;

	// Calculate acceleration using a direct difference equation
	double newAcc = (newVel - vel) / deltaT;

	// Update the joint command variables
	pos = newPos;
	vel = newVel;
	acc = newAcc;
	
	// Put data in unused filters in case setFromPos overloads are mixed
	m_dev_posToVel.put(newPos);
	m_dev_posToAcc.put(newPos);
	m_dev_velToAcc.put(newVel);
}

// Actually set the command based on a position and velocity
void Joint::Command::actuallySetFromPosVel(double deltaT, double newPos, double newVel)
{
	// Handle invalid time step
	if(deltaT <= 0)
	{
		pos = newPos;
		vel = newVel;
		return;
	}

	// Calculate acceleration based on velocity values
	m_dev_velToAcc.put(newVel);
	double newAcc = m_dev_velToAcc.value() / deltaT;

	// Update the joint command variables
	pos = newPos;
	vel = newVel;
	acc = newAcc;
	
	// Put data in unused filters in case setFromPos overloads are mixed
	m_dev_posToVel.put(newPos);
	m_dev_posToAcc.put(newPos);
}

// Actually set the command based on a position, velocity and acceleration
void Joint::Command::actuallySetFromPosVelAcc(double newPos, double newVel, double newAcc)
{
	// Update the joint command variables
	pos = newPos;
	vel = newVel;
	acc = newAcc;
	
	// Put data in unused filters in case setFromPos overloads are mixed
	m_dev_posToVel.put(newPos);
	m_dev_posToAcc.put(newPos);
	m_dev_velToAcc.put(newVel);
}

//
// Joint feedback class
//

// Constructor
Joint::Feedback::Feedback()
 : pos(0.0)
 , modelTorque(0.0)
 , torque(0.0)
{
}

}
// EOF