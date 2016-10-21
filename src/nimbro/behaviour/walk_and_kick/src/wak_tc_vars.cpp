// Walk and kick: Team communications variables
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/wak_tc_vars.h>
#include <walk_and_kick/wak_tc_ros.h>
#include <walk_and_kick/wak_sensor_vars.h>
#include <walk_and_kick/wak_vis.h>
#include <boost/make_shared.hpp>
#include <sstream>

// Namespaces
using namespace walk_and_kick;

//
// TCVars class
//

// Update function
bool TCVars::update(const ros::Time& now, const SensorVars* SV)
{
	// Retrieve the latest team communications
	const TCRobotListenerList& teamComms = TCRI.teamComms();

	// Update the data variables
	if(now.isZero())
	{
		// Don't do anything if the current time is zero
		m_robotDataMap.clear();
		robotDataMap.clear();
	}
	else
	{
		// Update the stored robot data based on the latest packets received by the team communications ROS interface
		for(TCRobotListenerList::const_iterator rtcIt = teamComms.begin(); rtcIt != teamComms.end(); ++rtcIt)
		{
			const TCRobotListenerConstPtr& rtc = *rtcIt;
			const TeamCommsData& rtcData = rtc->data();
			TCRobotDataMapInternal::iterator rdIt = m_robotDataMap.find(rtc->robotUID);
			if(rdIt != m_robotDataMap.end())
				rdIt->second->update(rtcData, SV);
			else if(dataIsFresh(now, rtcData))
			{
				TCRobotVarsPtr dataPtr = boost::make_shared<TCRobotVars>(config, field, rtc->robot, rtc->robotUID, rtcData);
				m_robotDataMap.insert(TCRobotDataMapInternal::value_type(rtc->robotUID, dataPtr));
				robotDataMap.insert(TCRobotDataMap::value_type(rtc->robotUID, dataPtr));
			}
		}

		// Erase any old robot data that is no longer fresh
		for(TCRobotDataMapInternal::iterator rdIt = m_robotDataMap.begin(); rdIt != m_robotDataMap.end();)
		{
			if(dataIsFresh(now, rdIt->second->data))
				++rdIt;
			else
			{
				robotDataMap.erase(rdIt->first);
				m_robotDataMap.erase(rdIt++); // Note: This works because the iterator is incremented before the element is erased and the interator is invalidated
			}
		}
	}

	// Decide whether we wish to plot data
	bool plotData = (config && PM && config->plotData());
	const unsigned int numRobotSlots = TCRI.nextRobotUID();

	// Reset the plot values for all robots
	if(plotData)
	{
		for(TCRobotListenerList::const_iterator rtcIt = teamComms.begin(); rtcIt != teamComms.end(); ++rtcIt)
		{
			const TCRobotListenerConstPtr& TCRL = *rtcIt;
			unsigned int offset = PM_COUNT + TCRL->robotUID;
			PM->plotScalar(TCRobotVars::RI_OLD_DATA, offset + 0*numRobotSlots);
			PM->plotScalar(INFINITY, offset + 1*numRobotSlots);
		}
	}

	// Compute the time since the last team communications data arrived and how many robot packets are fresh and valid
	timeSinceData = INFINITY;
	int numFresh = 0, numFreshAndValid = 0;
	for(TCRobotDataMapInternal::const_iterator rdIt = m_robotDataMap.begin(); rdIt != m_robotDataMap.end(); ++rdIt)
	{
		numFresh++;
		if(rdIt->second->dataValid)
			numFreshAndValid++;
		float timeSinceThisData = (now - rdIt->second->data.timestamp).toSec();
		if(timeSinceThisData < timeSinceData)
			timeSinceData = timeSinceThisData;
		if(plotData)
		{
			unsigned int offset = PM_COUNT + rdIt->second->robotUID;
			PM->plotScalar(rdIt->second->reasonInvalid, offset + 0*numRobotSlots);
			PM->plotScalar(timeSinceThisData, offset + 1*numRobotSlots);
		}
	}

	// Plotting
	if(plotData)
	{
		PM->plotScalar(numFresh, PM_TC_NUMFRESH);
		PM->plotScalar(numFreshAndValid, PM_TC_NUMFRESHVALID);
		PM->plotScalar(timeSinceData, PM_TC_TIMESINCEDATA);
	}

	// Return whether any fresh and valid team communications data is available
	return (numFreshAndValid > 0);
}

//
// TCRobotVars class
//

// Reason invalid names
const std::string TCRobotVars::ReasonInvalidName[RI_COUNT] = {
	"IsPenalty",
	"FieldMismatch",
	"InvalidKickoffType",
	"InvalidButtonState",
	"InvalidGameCmd",
	"InvalidGameRole",
	"InvalidPlayState",
	"SameRobotNumber",
	"WrongDirnOfPlay",
	"OldData",
	"Miscellaneous",
};

// Reason invalid name map
const std::map<TCRobotVars::ReasonInvalid, std::string> TCRobotVars::ReasonInvalidMap = reasonInvalidMap();

// Function that statically generates the reason invalid name map
std::map<TCRobotVars::ReasonInvalid, std::string>TCRobotVars::reasonInvalidMap()
{
	// Generate the required map
	std::map<ReasonInvalid, std::string> map;
	for(int i = 0; i < RI_COUNT; i++)
		map[(ReasonInvalid) (1 << i)] = ReasonInvalidName[i];
	return map;
}

// Generate reasons invalid string
std::string TCRobotVars::reasonsInvalidString(int reasonInvalid, const std::string& separator)
{
	// Combine all the reasons why some data is invalid into a string
	std::ostringstream ss;
	bool haveItem = false, haveMisc = false;
	for(int i = 0; i < RI_COUNT; i++, reasonInvalid >>= 1)
	{
		if((reasonInvalid & 1) != 0)
		{
			if(haveItem)
				ss << separator;
			ss << reasonInvalidName(i);
			haveMisc |= ((1 << i) == RI_MISCELLANEOUS);
			haveItem = true;
		}
	}
	if(reasonInvalid != 0 && !haveMisc)
	{
		if(haveItem)
			ss << separator;
		ss << reasonInvalidFlagName(RI_MISCELLANEOUS);
	}
	return ss.str();
}

// Update function
void TCRobotVars::update(const TeamCommsData& newData, const SensorVars* SV)
{
	// No need to update anything if this is the same packet as we last updated the class with
	if(newData.packetID == data.packetID && !data.timestamp.isZero()) return;

	// Transcribe the new packet of robot data
	data = newData;

	// Process team communications data
	if(data.fieldType >= FieldModel::UnknownField && data.fieldType < FieldModel::NumFieldTypes)
		fieldType = (FieldModel::FieldType) data.fieldType;
	else
		fieldType = FieldModel::UnknownField;
	if(kickoffTypeValid(data.kickoffType))
		kickoffType = (KickoffType) data.kickoffType;
	else
		kickoffType = KT_UNKNOWN;
	if(buttonStateValid(data.buttonState))
		buttonState = (ButtonState) data.buttonState;
	else
		buttonState = BTN_UNKNOWN;
	if(gameCommandValid(data.gameCommand))
		gameCommand = (GameCommand) data.gameCommand;
	else
		gameCommand = CMD_UNKNOWN;
	if(gameRoleValid(data.gameRole))
		gameRole = (GameRole) data.gameRole;
	else
		gameRole = ROLE_UNKNOWN;
	if(playStateValid(data.playState))
		playState = (PlayState) data.playState;
	else
		playState = PS_UNKNOWN;

	// Determine whether the configuration of the source robot matches ours, meaning that we can trust its data
	reasonInvalid = RI_VALID;
	if(fieldType != field.fieldType() || fieldType == FieldModel::UnknownField)
		reasonInvalid |= RI_FIELD_MISMATCH;
	if(!kickoffTypeValid(kickoffType))
		reasonInvalid |= RI_INVALID_KICKOFF_TYPE;
	if(!buttonStateValid(buttonState))
		reasonInvalid |= RI_INVALID_BUTTON_STATE;
	if(!gameCommandValid(gameCommand))
		reasonInvalid |= RI_INVALID_GAME_CMD;
	if(!gameRoleValid(gameRole))
		reasonInvalid |= RI_INVALID_GAME_ROLE;
	if(!playStateValid(playState))
		reasonInvalid |= RI_INVALID_PLAY_STATE;
	if(config)
	{
		if(data.isPenaltyShoot != 0)
			reasonInvalid |= RI_IS_PENALTY;
		if(data.robotNumber == config->robotNumber())
			reasonInvalid |= RI_SAME_ROBOT_NUMBER;
	}
	if(SV)
	{
		if((data.playOnYellow != 0) != SV->playOnYellow)
			reasonInvalid |= RI_WRONG_DIRN_OF_PLAY;
	}
	dataValid = (reasonInvalid == RI_VALID);
}
// EOF