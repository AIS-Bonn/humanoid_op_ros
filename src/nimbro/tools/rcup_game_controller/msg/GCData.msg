# Game states
uint8 STATE_INITIAL = 0
uint8 STATE_READY = 1
uint8 STATE_SET = 2
uint8 STATE_PLAYING = 3
uint8 STATE_FINISHED = 4

# Secondary game states
uint8 SECSTATE_NORMAL = 0
uint8 SECSTATE_PENALTYSHOOT = 1
uint8 SECSTATE_OVERTIME = 2
uint8 SECSTATE_TIMEOUT = 3

# Game controller data
uint32 seq                # Sequence ID
time  stampBase           # Local ROS time of when the base data was last updated
time  stampExtra          # Local ROS time of when the extra data was last updated
bool  extraOutOfDate      # Boolean flag whether the extra data is possibly out of date, due to no extra data having been received when or since the state/secondary state last changed
uint8 playersPerTeam      # Base: Number of players in the team
uint8 state               # Base: State of the game (see STATE_* above)
bool  firstHalf           # Base: Boolean flag whether this is the first half of a game
bool  ownKickoff          # Base: Boolean flag whether the kickoff, if applicable, is ours
bool  isDropBall          # Base: Boolean flag whether the kickoff, if applicable, is a drop ball
uint8 secondaryState      # Base: Secondary state of the game (see SECSTATE_* above)
bool  dropInDueToOwnTeam  # Base: Boolean flag whether the last ball out was due to us
int16 dropInTime          # Base: Time since the last ball out (<0 => No ball was out yet)
int16 secsRemaining       # Base: Time of play remaining (can be negative)
int16 secondaryTime       # Extra: Secondary time remaining (negative values should never occur, but do not assume this)
GCTeamInfo ownTeam        # Base/Extra: Information about our team
GCTeamInfo oppTeam        # Base/Extra: Information about the opponent team
GCRobotInfo ownRobot      # Base: Information about us
