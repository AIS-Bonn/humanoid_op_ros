// Humanoid kinematics - Common kinematics definitions
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef KINEMATICS_COMMON_H
#define KINEMATICS_COMMON_H

// Includes
#include <iostream>
#include <string>

/**
* @namespace humanoid_kinematics
*
* @brief Humanoid kinematics namespace.
**/
namespace humanoid_kinematics
{
	// Constants
	const std::string ROOT_CONFIG_PARAM_PATH = "/humanoid_kinematics/";

	// Limb index enumeration
	enum LimbIndex
	{
		LEFT = 0,  // Left limb
		RIGHT,     // Right limb
		NEUTRAL,   // Neutral limb

		NUM_LR = NEUTRAL,
		NUM_LRN,

		INDEX0 = 0,
		INDEX1,
		INDEX2
	};
	const char* const LimbIndexName[NUM_LRN] = {
		"Left",
		"Right",
		"Neutral"
	};
	const char LimbIndexChar[NUM_LRN] = {
		'L',
		'R',
		'N'
	};
	inline bool limbIndexValid(LimbIndex limbIndex) { return (limbIndex >= LEFT && limbIndex <= RIGHT); }
	inline bool limbIndexValidN(LimbIndex limbIndex) { return (limbIndex >= LEFT && limbIndex <= NEUTRAL); }
	inline LimbIndex ensureLimbIndex(LimbIndex limbIndex) { return (limbIndexValid(limbIndex) ? limbIndex : LEFT); }
	inline LimbIndex ensureLimbIndexN(LimbIndex limbIndex) { return (limbIndexValidN(limbIndex) ? limbIndex : NEUTRAL); }
	inline LimbIndex otherLimbIndex(LimbIndex limbIndex) { return (limbIndex == RIGHT ? LEFT : RIGHT); }
	inline LimbIndex otherLimbIndexN(LimbIndex limbIndex) { return (limbIndex == RIGHT ? LEFT : (limbIndex == LEFT ? RIGHT : NEUTRAL)); }
	inline const char* limbIndexName(LimbIndex limbIndex) { return (limbIndexValidN(limbIndex) ? LimbIndexName[limbIndex] : LimbIndexName[NEUTRAL]); }
	inline char limbIndexChar(LimbIndex limbIndex) { return (limbIndexValidN(limbIndex) ? LimbIndexChar[limbIndex] : LimbIndexChar[NEUTRAL]); }

	// Limb index stream operator
	inline std::ostream& operator<<(std::ostream& os, LimbIndex limbIndex) { return os << limbIndexName(limbIndex); }

	// Limb sign enumeration
	enum LimbSign
	{
		LS_LEFT = 1,
		LS_RIGHT = -1,
		LS_NEUTRAL = 0
	};

	// Limb sign stream operator
	inline std::ostream& operator<<(std::ostream& os, LimbSign limbSign) { return os << (limbSign == LS_LEFT ? "+1" : (limbSign == LS_RIGHT ? "-1" : " 0")); }

	// Limb sign helper functions
	inline bool limbSignIsLeft(LimbSign limbSign) { return (limbSign > 0); }
	inline bool limbSignIsRight(LimbSign limbSign) { return (limbSign < 0); }
	inline bool limbSignIsNeutral(LimbSign limbSign) { return (limbSign == 0); }
	inline LimbSign otherLimbSign(LimbSign limbSign) { return (limbSignIsLeft(limbSign) ? LS_RIGHT : (limbSignIsRight(limbSign) ? LS_LEFT : LS_NEUTRAL)); }
	inline LimbSign limbSignOf(LimbIndex limbIndex) { return (limbIndex == LEFT ? LS_LEFT : (limbIndex == RIGHT ? LS_RIGHT : LS_NEUTRAL)); }
	inline LimbSign otherLimbSignOf(LimbIndex limbIndex) { return (limbIndex == LEFT ? LS_RIGHT : (limbIndex == RIGHT ? LS_LEFT : LS_NEUTRAL)); }

	/**
	* @struct LRLimb
	* 
	* @brief Left/right limb index/sign struct.
	**/
	struct LRLimb
	{
		// Constructors
		LRLimb() : index(LEFT), sign(LS_LEFT), isLeft(true) {}
		LRLimb(LimbIndex limbIndex) { set(limbIndex); }
		explicit LRLimb(bool limbIsLeft) { setIsLeft(limbIsLeft); }

		// Set function
		void set(LimbIndex limbIndex) { setIsLeft(limbIndex != RIGHT); }
		void setIsLeft(bool limbIsLeft)
		{
			isLeft = limbIsLeft;
			if(isLeft)
			{
				index = LEFT;
				sign = LS_LEFT;
			}
			else
			{
				index = RIGHT;
				sign = LS_RIGHT;
			}
		}

		// Get functions
		LRLimb otherLimb() const { return LRLimb(!isLeft); }
		LimbIndex otherIndex() const { return (isLeft ? RIGHT : LEFT); }
		LimbSign otherSign() const { return (isLeft ? LS_RIGHT : LS_LEFT); }

		// Data members
		LimbIndex index;
		LimbSign sign;
		bool isLeft;
	};

	// Limb type enumeration
	enum LimbType
	{
		LT_NONE = 0,
		LT_LEG,
		LT_ARM,
		LT_HEAD,
		LT_COUNT
	};
	const char* const LimbTypeName[LT_COUNT] = {
		"None",
		"Leg",
		"Arm",
		"Head"
	};
	const char LimbTypeChar[LT_COUNT] = {
		'N',
		'L',
		'A',
		'H'
	};
	inline bool limbTypeValid(LimbType limbType) { return (limbType > LT_NONE && limbType < LT_COUNT); }
	inline LimbType ensureLimbType(LimbType limbType) { return (limbTypeValid(limbType) ? limbType : LT_NONE); }
	inline const char* limbTypeName(LimbType limbType) { return (limbTypeValid(limbType) ? LimbTypeName[limbType] : LimbTypeName[LT_NONE]); }
	inline char limbTypeChar(LimbType limbType) { return (limbTypeValid(limbType) ? LimbTypeChar[limbType] : LimbTypeChar[LT_NONE]); }

	// Limb type stream operator
	inline std::ostream& operator<<(std::ostream& os, LimbType limbType) { return os << limbTypeName(limbType); }

	/**
	* @struct JointInfo
	* 
	* @brief Information describing a particular joint of the robot.
	**/
	struct JointInfo
	{
		// Constructor
		JointInfo(const std::string& jointName, LimbIndex limbIndex, LimbType limbType)
		 : jointName(jointName)
		 , limbIndex(ensureLimbIndexN(limbIndex))
		 , limbType(ensureLimbType(limbType))
		{}

		// Data members
		std::string jointName;
		LimbIndex limbIndex;
		LimbType limbType;
	};

	// Joint info stream operator
	inline std::ostream& operator<<(std::ostream& os, const JointInfo& info) { return os << info.limbIndex << info.limbType << "::" << info.jointName; }
}

#endif
// EOF