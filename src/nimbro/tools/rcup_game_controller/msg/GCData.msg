##
#Constants defining game states
uint8 STATE_INITIAL = 0
uint8 STATE_READY = 1
uint8 STATE_SET = 2
uint8 STATE_PLAYING = 3
uint8 STATE_FINISHED = 4

##
#Constants defining secondary game states

uint8 STATE2_NORMAL = 0
uint8 STATE2_PENALTYSHOOT = 1
uint8 STATE2_OVERTIME = 2

##
#returns the number of players in each team
#
uint8 playerPerTeam

##
#returns the current state of the game
#
uint8 state

##
#returns the secondary State
#
uint8 secondaryState

##
#return 1 if in first half, 0 otherwise
#
uint8 firstHalf

time timeStamp
uint8 kickOffTeam
uint8 dropInTeam
uint8 dropInTime
uint32 remainingSeconds
GCTeamInfo[] teams
