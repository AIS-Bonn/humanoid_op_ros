# Team colours
uint8 TEAM_CYAN = 0
uint8 TEAM_MAGENTA = 1

# Team information
uint8 teamNumber       # Base: Team number
uint8 teamColour       # Base: Team colour
uint8 score            # Base: Goals scored
uint8 penaltyShot      # Extra: Penalty shot counter
uint16 singleShots     # Extra: Penalty shot success bit mask
GCRobotInfo[] players  # Base: Information about the robot players
