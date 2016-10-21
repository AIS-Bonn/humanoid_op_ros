// Walk and kick: Team communications variables
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
// Ensure header is only included once
#ifndef WAK_TC_VARS_H
#define WAK_TC_VARS_H

// Includes
#include <walk_and_kick/wak_common.h>
#include <walk_and_kick/wak_config.h>
#include <walk_and_kick/wak_utils.h>
#include <walk_and_kick/TeamCommsData.h>
#include <boost/shared_ptr.hpp>
#include <map>

// Defines
#define DEFAULT_TCFRESH_TIME  5.0f

// Walk and kick namespace
namespace walk_and_kick
{
	// Class declarations
	class SensorVars;
	class WAKMarkerMan;
	class TCRosInterface;
	class TCRobotVars;

	// Typedefs
	typedef boost::shared_ptr<const TCRobotVars> TCRobotVarsConstPtr;
	typedef std::map<unsigned int, TCRobotVarsConstPtr> TCRobotDataMap;

	/**
	* @class TCVars
	* 
	* @brief A class that encapsulates all of the team communication input data to the walk and kick node.
	**/
	class TCVars
	{
	public:
		// Constructor
		TCVars(const TCRosInterface& TCRI, WAKConfig* config, plot_msgs::PlotManagerFS* PM, WAKMarkerMan* MM) : config(config), TCRI(TCRI), PM(PM), MM(MM) { update(ros::Time(), NULL); }

		// Config parameters
		WAKConfig* const config;

		// Field dimensions
		const FieldDimensions field;

		// Update function
		bool update(const ros::Time& now, const SensorVars* SV); // Returns whether any fresh and valid team communications data is available

		// Robot team communications data (read only)
		TCRobotDataMap robotDataMap; // This is guaranteed to contain the robot data of exactly every robot for which we have a *fresh* data packet available, and no more. To use this data, just check the dataValid field of each entry first.
		float timeSinceData;

	private:
		// Internal modifiable robot data map
		typedef boost::shared_ptr<TCRobotVars> TCRobotVarsPtr;
		typedef std::map<unsigned int, TCRobotVarsPtr> TCRobotDataMapInternal;
		TCRobotDataMapInternal m_robotDataMap;

		// Helper functions
		bool dataIsFresh(const ros::Time& now, const TeamCommsData& data) const { return (!data.timestamp.isZero() && (now - data.timestamp).toSec() <= (config ? config->tcFreshTime() : DEFAULT_TCFRESH_TIME)); }

		// Team communications that are assumed to be updated regularly by the ROS interface
		const TCRosInterface& TCRI;

		// Plot manager
		plot_msgs::PlotManagerFS* PM;

		// Marker manager
		WAKMarkerMan* MM;
	};

	/**
	* @class TCRobotVars
	* 
	* @brief A class that encapsulates all of the team communications data received from one particular robot.
	**/
	class TCRobotVars
	{
	public:
		// Reason invalid enumeration
		enum ReasonInvalid
		{
			RI_VALID                = 0x0000,  // The robot data is valid
			RI_IS_PENALTY           = 0x0001,  // The robot is in penalty shootout mode
			RI_FIELD_MISMATCH       = 0x0002,  // The robot is playing on a different field size to us
			RI_INVALID_KICKOFF_TYPE = 0x0004,  // The claimed kickoff type is invalid
			RI_INVALID_BUTTON_STATE = 0x0008,  // The claimed button state is invalid
			RI_INVALID_GAME_CMD     = 0x0010,  // The claimed game command is invalid
			RI_INVALID_GAME_ROLE    = 0x0020,  // The claimed game role is invalid
			RI_INVALID_PLAY_STATE   = 0x0040,  // The claimed play state is invalid
			RI_SAME_ROBOT_NUMBER    = 0x0080,  // The robot has the same robot number as us
			RI_WRONG_DIRN_OF_PLAY   = 0x0100,  // The robot is playing on the opposite goal to us
			RI_OLD_DATA             = 0x0200,  // The robot data has timed out because it is too old
			RI_MISCELLANEOUS        = 0x0400,  // Invalid for a miscellaneous reason
			RI_COUNT                = 11       // ReasonInvalid = (1 << ReasonIndex)
		};
		static const bool reasonInvalidIndexValid(int reasonIndex) { return (reasonIndex >= 0 && reasonIndex < RI_COUNT); }
		static const std::string& reasonInvalidName(int reasonIndex) { if(reasonInvalidIndexValid(reasonIndex)) return ReasonInvalidName[reasonIndex]; else return ReasonInvalidName[RI_MISCELLANEOUS]; }
		static const std::string& reasonInvalidFlagName(ReasonInvalid reason) { if(ReasonInvalidMap.find(reason) != ReasonInvalidMap.end()) return ReasonInvalidMap.at(reason); else return ReasonInvalidMap.at(RI_MISCELLANEOUS); }
		static std::string reasonsInvalidString(int reasonInvalid, const std::string& separator = " | ");
	private:
		static const std::string ReasonInvalidName[RI_COUNT];
		static const std::map<ReasonInvalid, std::string> ReasonInvalidMap;
		static std::map<ReasonInvalid, std::string> reasonInvalidMap();

	public:
		// Constructor
		TCRobotVars(WAKConfig* config, const FieldDimensions& field, const std::string& robot, unsigned int robotUID) : config(config), field(field), robot(robot), robotUID(robotUID) { update(TeamCommsData(), NULL); dataValid = false; }
		TCRobotVars(WAKConfig* config, const FieldDimensions& field, const std::string& robot, unsigned int robotUID, const TeamCommsData& newData) : config(config), field(field), robot(robot), robotUID(robotUID) { update(newData, NULL); }

		// Config parameters
		WAKConfig* const config;

		// Field dimensions
		const FieldDimensions& field;

		// Robot name and UID
		const std::string robot;
		const unsigned int robotUID;

		// Update function
		void update(const TeamCommsData& newData, const SensorVars* SV);

		// Data valid flag (this should always be checked before using any of the data variables below)
		bool dataValid;
		int reasonInvalid;

		// Processed team communications data
		FieldModel::FieldType fieldType;
		KickoffType kickoffType;
		ButtonState buttonState;
		GameCommand gameCommand;
		GameRole gameRole;
		PlayState playState;

		// Raw team communications packet
		TeamCommsData data;
	};
}

#endif
// EOF