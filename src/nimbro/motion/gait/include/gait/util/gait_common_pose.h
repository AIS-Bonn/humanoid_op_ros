// Gait utility class that provides common pose representation functionality
// File: gait_common_pose.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef GAIT_COMMON_POSE_H
#define GAIT_COMMON_POSE_H

// Gait namespace
namespace gait
{
	/**
	* @struct CommonLegData
	*
	* @brief Data struct that contains the information pertaining to a leg pose that should be common amongst all pose representations.
	*
	* The assumed joint order is `hip yaw` &rarr; `hip roll` &rarr; `hip pitch` &rarr; `knee pitch` &rarr; `ankle pitch` &rarr; `ankle roll`.
	* The upper and lower leg links are assumed to be of the same length (i.e. #linkLength).
	**/
	struct CommonLegData
	{
		//
		// Constructor
		//

		//! Default constructor
		explicit CommonLegData(bool left = true) { reset(left); }

		//! Reset function
		inline void reset(bool left = true)
		{
			setEffort(0.0);
			setSupportCoeff(0.0);
			setLegSign(left);
			setLinkLength(1.0);
		}

		//
		// Set functions
		//

		//! Set the effort of the leg (equal for all joints)
		inline void setEffort(double eff)
		{
			effortHipYaw = eff;
			effortHipRoll = eff;
			effortHipPitch = eff;
			effortKneePitch = eff;
			effortAnklePitch = eff;
			effortAnkleRoll = eff;
		}

		//! Set the effort of the leg (equal for all yaw, pitch and roll joints)
		inline void setEffort(double effYaw, double effPitch, double effRoll)
		{
			effortHipYaw = effYaw;
			effortHipRoll = effRoll;
			effortHipPitch = effPitch;
			effortKneePitch = effPitch;
			effortAnklePitch = effPitch;
			effortAnkleRoll = effRoll;
		}

		//! Set the effort of the leg (independent for each joint)
		inline void setEffort(double effHipYaw, double effHipRoll, double effHipPitch, double effKneePitch, double effAnklePitch, double effAnkleRoll)
		{
			effortHipYaw = effHipYaw;
			effortHipRoll = effHipRoll;
			effortHipPitch = effHipPitch;
			effortKneePitch = effKneePitch;
			effortAnklePitch = effAnklePitch;
			effortAnkleRoll = effAnkleRoll;
		}

		//! Set the support coefficient of the leg
		inline void setSupportCoeff(double coeff)
		{
			supportCoeff = coeff;
		}

		//! Set the sign of the leg
		inline void setLegSign(bool left)
		{
			isLeft = left;
			limbSign = (left ? 1 : -1);
		}

		//! Set the link length used for pose conversions
		inline void setLinkLength(double length)
		{
			linkLength = length;
		}

		//
		// Data members
		//

		// Joint efforts
		double effortHipYaw;     //!< Dimensionless effort value associated with the hip yaw (in the range `[0,1]`, 0 = No effort, 1 = Full effort)
		double effortHipRoll;    //!< Dimensionless effort value associated with the hip roll (in the range `[0,1]`, 0 = No effort, 1 = Full effort)
		double effortHipPitch;   //!< Dimensionless effort value associated with the hip pitch (in the range `[0,1]`, 0 = No effort, 1 = Full effort)
		double effortKneePitch;  //!< Dimensionless effort value associated with the knee pitch (in the range `[0,1]`, 0 = No effort, 1 = Full effort)
		double effortAnklePitch; //!< Dimensionless effort value associated with the ankle pitch (in the range `[0,1]`, 0 = No effort, 1 = Full effort)
		double effortAnkleRoll;  //!< Dimensionless effort value associated with the ankle roll (in the range `[0,1]`, 0 = No effort, 1 = Full effort)

		// Support coefficient
		double supportCoeff;     //!< Dimensionless support coefficient representing how much of the robot's weight is on this leg (in the range `[0,1]`, 0 = No support, 1 = Full support)

		// Leg sign
		bool isLeft;             //!< Boolean flag that is `true` if this is a left leg and `false` if this is a right leg
		int limbSign;            //!< Integral value that is `+1` if this is a left leg and `-1` if this is a right leg

		// Link length
		double linkLength;       //!< The assumed (equal) length of the upper and lower leg links (used for joint pose conversions)
	};

	/**
	* @struct CommonArmData
	*
	* @brief Data struct that contains the information pertaining to an arm pose that should be common amongst all pose representations.
	*
	* The assumed joint order is `shoulder pitch` &rarr; `shoulder roll` &rarr; `elbow pitch`. The upper and lower arm links are assumed
	* to be of the same length (i.e. #linkLength).
	**/
	struct CommonArmData
	{
		//
		// Constructor
		//

		//! Default constructor
		explicit CommonArmData(bool left = true) { reset(left); }

		//! Reset function
		inline void reset(bool left = true)
		{
			setEffort(0.0);
			setArmSign(left);
			setLinkLength(1.0);
		}

		//
		// Set functions
		//

		//! Set the effort of the arm (equal for all joints)
		inline void setEffort(double eff)
		{
			effortShoulderPitch = eff;
			effortShoulderRoll = eff;
			effortElbowPitch = eff;
		}

		//! Set the effort of the arm (equal for all pitch and roll joints)
		inline void setEffort(double effPitch, double effRoll)
		{
			effortShoulderPitch = effPitch;
			effortShoulderRoll = effRoll;
			effortElbowPitch = effPitch;
		}

		//! Set the effort of the arm (independent for each joint)
		inline void setEffort(double effShoulderPitch, double effShoulderRoll, double effElbowPitch)
		{
			effortShoulderPitch = effShoulderPitch;
			effortShoulderRoll = effShoulderRoll;
			effortElbowPitch = effElbowPitch;
		}

		//! Set the sign of the arm
		inline void setArmSign(bool left)
		{
			isLeft = left;
			limbSign = (left ? 1 : -1);
		}

		//! Set the link length used for pose conversions
		inline void setLinkLength(double length)
		{
			linkLength = length;
		}

		//
		// Data members
		//

		// Joint efforts
		double effortShoulderPitch; //!< Dimensionless effort value associated with the shoulder pitch (in the range `[0,1]`, 0 = No effort, 1 = Full effort)
		double effortShoulderRoll;  //!< Dimensionless effort value associated with the shoulder roll (in the range `[0,1]`, 0 = No effort, 1 = Full effort)
		double effortElbowPitch;    //!< Dimensionless effort value associated with the elbow pitch (in the range `[0,1]`, 0 = No effort, 1 = Full effort)

		// Arm sign
		bool isLeft;                //!< Boolean flag that is `true` if this is a left arm and `false` if this is a right arm
		int limbSign;               //!< Integral value that is `+1` if this is a left arm and `-1` if this is a right arm

		// Link length
		double linkLength;          //!< The assumed (equal) length of the upper and lower arm links (used for joint pose conversions)
	};
}

#endif /* GAIT_COMMON_POSE_H */
// EOF