uint8 TONE = 0
uint8 MUSIC = 1

uint8 soundType      # Play a tone (TONE) or music (MUSIC)
uint8 musicIndex     # Valid range 0-25
uint8 toneFreq       # Valid range 0-51 (0 is deepest pitch, 51 is highest pitch)
float32 toneDuration # Valid range 0.0 to 5.0 seconds