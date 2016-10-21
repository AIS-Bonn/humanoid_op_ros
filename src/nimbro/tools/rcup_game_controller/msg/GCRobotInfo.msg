# Penalty states
uint8 PENALTY_NONE = 0
uint8 PENALTY_BALL_MANIPULATION = 1
uint8 PENALTY_PHYSICAL_CONTACT = 2
uint8 PENALTY_ILLEGAL_ATTACK = 3
uint8 PENALTY_ILLEGAL_DEFENSE = 4
uint8 PENALTY_REQUEST_FOR_PICKUP = 5
uint8 PENALTY_REQUEST_FOR_SERVICE = 6
uint8 PENALTY_REQUEST_FOR_PICKUP_2_SERVICE = 7
uint8 PENALTY_SUBSTITUTE = 14
uint8 PENALTY_MANUAL = 15

# Robot information
uint8 penaltyState          # Base: Penalty state of the robot (see PENALTY_* above)
uint8 secsUntilUnpenalised  # Base: Time until the robot should be unpenalised
