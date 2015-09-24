# General
uint8 type          # Type of limb command (TYPE_SETPOINT_TIME, TYPE_SETPOINT_VEL, TYPE_SINE_WAVE)
string joint        # Name of joint to command
float32 effort      # Effort (torque) with which to move the joint, in the range [0.0,1.0]
uint8 timeref       # Time reference identifier (Valid if != 0, if 0 then this becomes a null command that can be used to separate motions with equal timerefs)

# For all types (exact definitions vary with type)
float32 period      # Time base of the motion (not used for TYPE_SETPOINT_VEL)
float32 position    # Angular position pertaining to motion

# For type 2 (setpoint with velocity)
float32 velocity    # Velocity with which to approach the given setpoint (i.e. Joint command(t) = initPos + sign(position - initPos) * velocity * t)

# For type 3 (sine wave)
float32 amplitude   # Amplitude of the sine wave
float32 phase       # Initial phase of the sine wave (i.e. Joint command(t) = position + amplitude * sin(2*pi*t/period + phase))
float32 cycles      # Number of cycles to execute (i.e. Total motion duration is cycles*period)
