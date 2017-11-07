// Joint information
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef JOINT_H
#define JOINT_H

#include <string>
#include <ros/time.h>
#include <urdf/model.h>
#include <rc_utils/golay.h>

namespace robotcontrol
{

/**
 * @brief Single joint representation
 *
 * Contains command values and feedback information for a single joint.
 **/
class Joint
{
public:
	//! Joint-level command
	class Command
	{
	public:
		// SetFromPos type enumeration
		enum SetFromType
		{
			SFT_NONE,
			SFT_POS,
			SFT_POSUNSMOOTHED,
			SFT_POSVEL,
			SFT_POSVELACC,
			SFT_COUNT
		};

		// Constructor
		Command();

		// Position, velocity and acceleration of the command
		double pos;    //!< Goal position
		double vel;    //!< Velocity
		double acc;    //!< Acceleration

		// Raw commanded position
		double rawPos; //!< Lowest level command that is actually sent as a position target to the servo (rad)

		// Command effort
		double effort; //!< Joint effort (stiffness), range [0,1]
		bool raw;      //!< Disables the motor model if true

		//! Reset the internal filters used to differentiate positions and velocities
		void resetDerivs();

		//! Start an update phase
		void startUpdatePhase();

		//! Stop an update phase and actually perform the last seen update
		void stopUpdatePhase(double deltaT);

		//! Position command (Golay derivatives)
		void setFromPos(double newPos);

		//! Position command (direct difference equations)
		void setFromPosUnsmoothed(double newPos);

		//! Position and velocity command
		void setFromPosVel(double newPos, double newVel);

		//! Position, velocity and acceleration command
		void setFromPosVelAcc(double newPos, double newVel, double newAcc);

		/**
		 * @brief Global velocity limit
		 *
		 * The commanded velocity is limited by this global velocity limit. If a
		 * limitation occurs, the goal position is also limited to the position
		 * reachable with the limited velocity.
		 **/
		static double velLimit; // NOTE: Not currently used!

		/**
		 * @brief Global acceleration limit
		 *
		 * This parameter simply limits the calculated acceleration. It does
		 * not, however, constrain the position or velocity commands.
		 **/
		static double accLimit; // NOTE: Not currently used!
		
	private:
		// Internal setFromPos functions that actually do the work
		void actuallySetFromPos(double deltaT, double newPos);
		void actuallySetFromPosUnsmoothed(double deltaT, double newPos);
		void actuallySetFromPosVel(double deltaT, double newPos, double newVel);
		void actuallySetFromPosVelAcc(double newPos, double newVel, double newAcc);

		// Update phase variables
		SetFromType m_updateType;
		double m_newPos;
		double m_newVel;
		double m_newAcc;
		int m_updateCount;

		// Golay derivatives
		rc_utils::GolayDerivative<double, 1, 5> m_dev_posToVel;
		rc_utils::GolayDerivative<double, 2, 5> m_dev_posToAcc;
		rc_utils::GolayDerivative<double, 1, 5> m_dev_velToAcc;
	};

	//! Joint-level feedback
	class Feedback
	{
	public:
		// Constructor
		Feedback();

		// Data mambers
		ros::Time stamp;    //!< Feedback timestamp
		double pos;         //!< Current servo position (rad)
		double modelTorque; //!< Calculated servo torque (Nm)
		double torque;      //!< Estimated torque from position displacement (Nm)
	};

	//! Constructor
	Joint() = default;

	//! Smart pointer type (use this!)
	typedef boost::shared_ptr<Joint> Ptr;

	//! URDF joint name
	std::string name;

	//! URDF joint
	boost::shared_ptr<urdf::Joint> modelJoint;

	//! Mimic joint information
	boost::shared_ptr<urdf::JointMimic> mimic;

	//! Joint command (written by the MotionModule)
	Command cmd;
	Command lastCmd;

	//! Feedback (written by the HardwareInterface)
	Feedback feedback;
};

}

#endif
// EOF