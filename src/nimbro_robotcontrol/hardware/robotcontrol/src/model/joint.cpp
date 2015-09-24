// Joint information
// Authors: Max Schwarz <max.schwarz@uni-bonn.de>
//          Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <robotcontrol/model/joint.h>

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

/**
 * @brief This method calculates the vel and acc fields using Savitzky-Golay smoothed differentiation.
 * 
 * The function assumes that it is called at regular intervals given by the @p deltaT parameter.
 *
 * @param deltaT Time step (s)
 * @param newPos Goal position (rad)
 **/
void Joint::Command::setFromPos(double deltaT, double newPos)
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

/**
 * @brief This method calculates the vel and acc fields using direct difference equations, use with care!
 *
 * The function assumes that it is called at regular intervals given by the @p deltaT parameter.
 *
 * @param deltaT Time step (s)
 * @param newPos Goal position (rad)
 **/
void Joint::Command::setFromPosUnsmoothed(double deltaT, double newPos)
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

/**
 * @brief This method calculates the acc field using Savitzky-Golay smoothed differentiation of the given velocity values.
 *
 * The function assumes that it is called at regular intervals given by the @p deltaT parameter.
 *
 * @param deltaT Time step (s)
 * @param newPos Goal position (rad)
 * @param newVel Goal velocity (rad/s)
 **/
void Joint::Command::setFromPosVel(double deltaT, double newPos, double newVel)
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

/**
 * This function directly sets the target position, velocity and acceleration values.
 *
 * @param newPos Goal position (rad)
 * @param newVel Goal velocity (rad/s)
 * @param newAcc Goal acceleration (rad/s^2)
 **/
void Joint::Command::setFromPosVelAcc(double newPos, double newVel, double newAcc)
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
// Joint feedback struct
//

// Constructor
Joint::Feedback::Feedback()
 : pos(0.0)
 , modelTorque(0.0)
 , torque(0.0)
{
}

}
