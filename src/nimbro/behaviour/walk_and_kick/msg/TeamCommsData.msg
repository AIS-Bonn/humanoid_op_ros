# Robot team communications data
# If you change something here, then you have to update WalkAndKick::publishTeamComms() in walk_and_kick.cpp,
# and if the field requires special preprocessing (e.g. an enumerated value) then you also have to update
# the class TCVars::RobotData.

# Packet information
uint16 packetID
time timestamp
int8 robotNumber
bool silencing

# Configuration information
int8 fieldType
bool playOnYellow
bool playAsCyan
bool listenToGC
bool listenToTC
bool isPenaltyShoot
int8 kickoffType

# Play state information
int8 buttonState
int8 gameCommand
int8 gameRole
int8 playState
int8 gameState
int8 behState
bool fallen

# Game state information
float32 compassHeading
float32 robotPoseX
float32 robotPoseY
float32 robotPoseT
float32 robotPoseConf
bool robotPoseValid
float32 ballPoseX
float32 ballPoseY
float32 ballPoseConf
bool ballPoseValid
bool ballPoseStable
float32 obstClosestPoseX
float32 obstClosestPoseY
float32 obstClosestPoseConf
bool obstClosestPoseValid
float32 walkingTargetPoseX
float32 walkingTargetPoseY
bool walkingTargetPoseValid

# Status information
float32 timeSinceGCBase
float32 timeSinceGCExtra
float32 timeSinceTC
uint8 numFreshTC