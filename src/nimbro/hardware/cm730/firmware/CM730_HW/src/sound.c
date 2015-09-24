/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : sound.c
* Author             : Robotis
* Version            : V0.1
* Date               : 2011/02/17
* Description        : Functions to control sound output and the buzzer
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Includes
#include "sound.h"
#include "stm32f10x_lib.h"

// Sound lengths
#define DIGIT_SOUND_LEN   0x0080
#define LEN_16            (1*DIGIT_SOUND_LEN)
#define LEN_8             (2*DIGIT_SOUND_LEN)
#define LEN_8H            (3*DIGIT_SOUND_LEN)
#define LEN_4             (4*DIGIT_SOUND_LEN)
#define LEN_4H            (6*DIGIT_SOUND_LEN)
#define LEN_2             (8*DIGIT_SOUND_LEN)

// Sound wave defines
#define S_OFF             0
#define S_CONTINUE        30
#define S_WAVE            32
#define S_WAVE_SIGN       64
#define S_WAVE_P          S_WAVE
#define S_WAVE_M          (S_WAVE+S_WAVE_SIGN)
#define S_NULL            0

// Sound table sizes
#define SIZE_DOREMITABLE  52
#define SIZE_SOUNDTABLE   29

// The sound to play is defined using a data variable and a play length.
// Set the data with setBuzzerData(), set the play length with setBuzzerPlayLength(), and use PlayBuzzer() to start the sound.
// If play length is 0xFF then the data is interpreted as the index in the music table of the sound to play.
// If play length is NOT 0xFF then the data is interpreted as the index of a tone in the doremi table, with the play length being multiplied by 100ms to get the play time.
// The very first index in a music array is a scaler for the tempo of the music (larger = slower).
// The subsequent u16 values in a music array are referred to as 'packets', and contain one sound command each.
// A guide to how the individual u16 packets are defined:
// <12-15>: Unused
// <11>:    On/Off
// <7-10>:  Sound length
// <6>:     S_WAVE sign
// <5>:     S_WAVE flag
// <0-4>:   Identifier of the sound

// Play functions
void PlayMusic(u8 musicIndex);
void PlayDoremi(u16 index, u16 playTime);

// Sound table for the notes in the doremi scale (entries are buzzer periods in us)
const u16 DoremiTable[SIZE_DOREMITABLE] = {
	9091, 8584, 8099, 7645, 7220, 6811, 6428, 6067, 5727, 5406,
	5102, 4816, 4545, 4292, 4049, 3822, 3610, 3405, 3214, 3033,
	2863, 2703, 2551, 2408, 2272, 2145, 2025, 1911, 1804, 1702,
	1607, 1517, 1431, 1351, 1275, 1204, 1136, 1072, 1012, 955,
	902, 850, 803, 758, 716, 675, 638, 602, 568, 536, 506, 478
};

// Sound table used to define the notes of a music/melody (entries are buzzer periods in us)
const u16 SoundTable[SIZE_SOUNDTABLE] = {
	0, 750, 4292, 4049, 3822, 3610, 3405, 3214, 3033, 2863, 2703,
	2551, 2408, 2272, 2145, 2025, 1911, 1804, 1702, 1607, 1517,
	1431, 1351, 1275, 1204, 1136, 900, 12500, 6250
};

// Enumerated indices for the SoundTable array
enum
{
	high1 = 1,
	la0_, si0, do1, do1_,
	le1, le1_, mi1, fa1,
	fa1_, sol1, sol1_, la1,
	la1_, si1, do2, do2_,
	le2, le2_, mi2, fa2,
	fa2_, sol2, sol2_, la2,
	high2, low0, low1
};

//
// Music and sounds
//

#define DELTA_M0  4
const u16 Music0[] = {
	8,
	DELTA_M0 + do1  + LEN_8,
	DELTA_M0 + le1  + LEN_8,
	DELTA_M0 + mi1  + LEN_8,
	DELTA_M0 + fa1  + LEN_8,
	DELTA_M0 + sol1 + LEN_8,
	DELTA_M0 + la1  + LEN_8,
	DELTA_M0 + si1  + LEN_8,
	DELTA_M0 + do2  + LEN_8,
	S_NULL
};

#define DELTA_M1  0
const u16 Music1[] = {
	8,
	DELTA_M1 + do2  + LEN_8,
	DELTA_M1 + si1  + LEN_8,
	DELTA_M1 + la1  + LEN_8,
	DELTA_M1 + sol1 + LEN_8,
	DELTA_M1 + fa1  + LEN_8,
	DELTA_M1 + mi1  + LEN_8,
	DELTA_M1 + le1  + LEN_8,
	DELTA_M1 + do1  + LEN_2,
	S_NULL
};

#define DELTA_M6  4
const u16 Music6[] = {
	15,
	DELTA_M6 + do1 + LEN_8,
	DELTA_M6 + fa1 + LEN_8,
	DELTA_M6 + la1 + LEN_8,
	DELTA_M6 + do2 + LEN_8H,
	S_OFF          + LEN_16,
	DELTA_M6 + la1 + LEN_8,
	DELTA_M6 + do2 + LEN_2,
	S_CONTINUE     + LEN_8,
	S_OFF          + LEN_8,
	DELTA_M6 + do2 + LEN_8,
	S_OFF          + LEN_8,
	DELTA_M6 + do2 + LEN_16,
	DELTA_M6 + do2 + LEN_16,
	DELTA_M6 + do2 + LEN_8,
	S_OFF          + LEN_8,
	DELTA_M6 + do2 + LEN_8,
	DELTA_M6 + fa2 + LEN_2,
	S_OFF          + LEN_4,
	S_NULL
};

#define DELTA_M7  12
const u16 Music7[] = {
	23,
	DELTA_M7 + do1  + LEN_8,
	S_OFF           + LEN_16,
	DELTA_M7 + do1  + LEN_16,
	DELTA_M7 + do1  + LEN_16,
	DELTA_M7 + do1  + LEN_8,
	DELTA_M7 + do1  + LEN_8,
	DELTA_M7 + sol1 + LEN_16,
	DELTA_M7 + mi1  + LEN_16,
	DELTA_M7 + sol1 + LEN_16,
	DELTA_M7 + mi1  + LEN_16,
	DELTA_M7 + do1  + LEN_2,
	S_OFF           + LEN_8,
	S_NULL
};

const u16 Sound0[] = {
	20,
	do2           + LEN_16,
	S_WAVE_P + 8  + LEN_8H,
	S_CONTINUE    + LEN_8,
	S_WAVE_M + 8  + LEN_8H,
	S_WAVE_P + 12 + LEN_16,
	S_WAVE_M + 12 + LEN_16,
	S_OFF         + LEN_8,
	S_NULL
};

const u16 Sound1[] = {
	20,
	mi1          + LEN_8,
	S_WAVE_P + 4 + LEN_8,
	S_WAVE_M + 2 + LEN_2,
	S_CONTINUE   + LEN_2,
	S_CONTINUE   + LEN_4,
	S_OFF        + LEN_8,
	S_NULL
};

const u16 Sound2[] = {
	16,
	sol2          + LEN_16,
	S_WAVE_P + 18 + LEN_16,
	S_WAVE_M + 18 + LEN_16,
	S_OFF         + LEN_16,
	S_WAVE_P + 18 + LEN_16,
	S_WAVE_M + 18 + LEN_16,
	S_OFF         + LEN_8,
	S_NULL
};

const u16 Sound3[] = {
	32,
	fa1          + LEN_8,
	S_WAVE_P + 1 + LEN_2,
	S_CONTINUE   + LEN_4,
	S_OFF        + LEN_8,
	S_NULL
};

const u16 Sound4[] = {
	20,
	sol2          + LEN_8,
	S_WAVE_P + 16 + LEN_16,
	S_WAVE_M + 1  + LEN_4,
	S_OFF         + LEN_8,
	S_NULL
};

const u16 Sound5[] = {
	16,
	sol2         + LEN_8,
	S_WAVE_P + 3 + LEN_4,
	S_WAVE_M + 3 + LEN_4,
	S_WAVE_P + 3 + LEN_4,
	S_WAVE_M + 3 + LEN_4,
	S_OFF        + LEN_8,
	S_NULL
};

const u16 Sound6[] = {
	20,
	do2           + LEN_16,
	S_WAVE_M + 12 + LEN_2,
	S_CONTINUE    + LEN_2,
	S_NULL
};

const u16 Sound7[] = {
	20,
	high2        + LEN_16,
	S_WAVE_M + 1 + LEN_2,
	S_WAVE_P + 1 + LEN_2,
	S_OFF        + LEN_8,
	S_NULL
};

const u16 Sound8[] = {
	16,
	high1        + LEN_16,
	S_WAVE_M + 1 + LEN_16,
	high2        + LEN_16,
	S_WAVE_M + 1 + LEN_16,
	S_WAVE_P + 1 + LEN_16,
	S_OFF        + LEN_8,
	S_NULL
};

const u16 Sound10[] = {
	16,
	high2         + LEN_8,
	S_WAVE_M + 10 + LEN_8,
	S_OFF         + LEN_8,
	S_NULL
};

const u16 Sound11[] = {
	16,
	high2         + LEN_16,
	S_WAVE_P + 10 + LEN_16,
	S_WAVE_M + 10 + LEN_16,
	S_OFF         + LEN_8,
	S_NULL
};

const u16 Sound12[] = {
	16,
	high2         + LEN_16,
	S_WAVE_P + 10 + LEN_16,
	S_WAVE_M + 10 + LEN_16,
	S_OFF         + LEN_16,
	S_WAVE_P + 10 + LEN_16,
	S_WAVE_M + 10 + LEN_16,
	S_OFF         + LEN_8,
	S_NULL
};

const u16 Sound13[] = {
	8,
	mi2          + LEN_16,
	S_WAVE_P + 5 + LEN_16,
	sol2         + LEN_4,
	mi2          + LEN_4,
	S_WAVE_P + 5 + LEN_16,
	sol2         + LEN_4,
	mi2          + LEN_4,
	S_OFF        + LEN_8,
	S_NULL
};

const u16 Sound15[] = {
	9,
	S_OFF + LEN_2,
	la1_  + LEN_4,
	S_OFF + LEN_16,
	la0_  + LEN_8H,
	la0_  + LEN_8H,
	la0_  + LEN_8H,
	S_OFF + LEN_2,
	S_NULL
};

const u16 Sound20[] = {
	16,
	low1          + LEN_16,
	S_WAVE_P + 50 + LEN_2,
	S_OFF         + LEN_8,
	S_NULL
};

const u16 Sound21[] = {
	4,
	high1         + LEN_16,
	S_WAVE_M + 20 + LEN_2,
	S_WAVE_P + 20 + LEN_2,
	S_WAVE_M + 40 + LEN_2,
	S_WAVE_P + 40 + LEN_2,
	S_OFF         + LEN_8,
	S_NULL
};

const u16 Sound22[] = {
	1,
	high2         + LEN_16,
	S_WAVE_M + 10 + LEN_2,
	S_WAVE_P + 10 + LEN_2,
	S_WAVE_M + 20 + LEN_2,
	S_WAVE_P + 20 + LEN_2,
	S_WAVE_M + 30 + LEN_2,
	S_WAVE_P + 30 + LEN_2,
	S_WAVE_M + 40 + LEN_2,
	S_WAVE_P + 40 + LEN_2,
	S_WAVE_M + 50 + LEN_2,
	S_WAVE_P + 50 + LEN_2,
	S_WAVE_M + 60 + LEN_2,
	S_WAVE_P + 60 + LEN_2,
	S_WAVE_M + 70 + LEN_2,
	S_WAVE_P + 70 + LEN_2,
	S_WAVE_M + 80 + LEN_2,
	S_WAVE_P + 80 + LEN_2,
	S_OFF         + LEN_8,
	S_NULL
};

const u16 Sound23[] = {
	8,
	high1        + LEN_16,
	S_WAVE_M + 8 + LEN_2,
	S_WAVE_P + 1 + LEN_2,
	S_OFF        + LEN_8,
	S_NULL
};

const u16 Sound24[] = {
	4,
	la2   + LEN_8H,
	S_OFF + LEN_16,
	la2   + LEN_8H,
	S_OFF + LEN_16,
	la2   + LEN_8H,
	S_OFF + LEN_16,
	la2   + LEN_8H,
	S_OFF + LEN_16,
	S_NULL
};

const u16 Sound25[] = {
	16,
	fa2           + LEN_8,
	S_WAVE_M + 18 + LEN_4,
	S_WAVE_M + 18 + LEN_4,
	S_WAVE_M + 18 + LEN_4,
	S_OFF         + LEN_8,
	S_NULL
};

const u16 Sound30[] = {
	8,
	high2        + LEN_16,
	high1        + LEN_16,
	S_WAVE_M + 8 + LEN_16,
	S_WAVE_P + 2 + LEN_4H,
	S_WAVE_P + 1 + LEN_4,
	S_OFF        + LEN_8,
	S_NULL
};

const u16 Sound31[] = {
	7,
	high1   + LEN_16,
	fa2 - 1 + LEN_8,
	S_OFF   + LEN_16,
	high1   + LEN_16,
	fa2 - 2 + LEN_8,
	S_OFF   + LEN_16,
	high1   + LEN_16,
	fa2 - 3 + LEN_8,
	S_OFF   + LEN_16,
	high1   + LEN_16,
	fa2 - 4 + LEN_8,
	S_OFF   + LEN_16,
	high1   + LEN_16,
	fa2 - 5 + LEN_8,
	S_OFF   + LEN_16,
	S_NULL
};

// Music table
#define MUSIC_N  26
const u16* MusicTable[MUSIC_N] = {Music0 , Music1 , Music6 , Music7 , Sound0 , Sound1 , Sound2 , Sound3 ,
                                  Sound4 , Sound5 , Sound6 , Sound7 , Sound8 , Sound10, Sound11, Sound12,
                                  Sound13, Sound15, Sound20, Sound21, Sound22, Sound23, Sound24, Sound25,
                                  Sound30, Sound31};

// Global variables
u16 Music_MusicIndex = 0;
u16 Music_TempoContainer = 1;
u16 Music_TempoCnt = 0;
u16 Music_ReadIndex = 0;
u16 Music_Play = 0;
u16 Music_CurrentPacket = 0;
u16 Music_CurrentSound = 0;
u16 Music_CurrentLen = 0;
u16 Music_WaveFlag = 0;
u16 Music_WaveBuffer = 0;
s16 Music_WaveStep = 0;
u16 Music_StartDelayFlag = 0;
u16 Doremi_Play = 0;
u16 Doremi_Index = 0;
u16 Doremi_TimCount = 0;
u8  gbBuzzerPlayLength = 0;
u8  gbBuzzerData = 0;

//
// ISRs
//

// Interrupt service routine for the buzzer
void __ISR_BUZZER_MANAGE(void)
{
	// Handle the playing of a doremi tone
	if(Doremi_Play)
	{
		if(Doremi_TimCount > 0)
		{
			Doremi_TimCount--;
			setBuzzer(DoremiTable[Doremi_Index]);
		}
		else
		{
			Doremi_Play = 0;
			setBuzzer(0);
		}
		return;
	}

	// Nothing more to do if we are not playing music at the moment
	if(!Music_Play) return;

	// If the current packet is not finished yet then keep going with it...
	if(Music_TempoCnt > 0)
	{
		Music_TempoCnt--;
		if(Music_WaveFlag == 1)
		{
			Music_WaveBuffer = (u16)((s16)Music_WaveBuffer + Music_WaveStep);
			setBuzzer(Music_WaveBuffer);
		}
	}
	else // OR: Process a new packet (single u16 entry in one of the music arrays)...
	{
		// If this is the start of a new music then retrieve the tempo container value (scales the lengths specified in the individual packets)
		if(Music_ReadIndex == 0)
			Music_TempoContainer = *(MusicTable[Music_MusicIndex] + Music_ReadIndex++);

		// Retrieve the next packet to process
		Music_CurrentPacket = *(MusicTable[Music_MusicIndex] + Music_ReadIndex); // 16-bit entry from the required music array
		Music_CurrentLen    = (Music_CurrentPacket >> 7) & 0x000F;               // Bits 7-10: Sound length
		Music_CurrentSound  = Music_CurrentPacket & 0x001F;                      // Bits 0-4:  Identifier of the sound

		// Stop the buzzer and the playback if the null terminator of the music has been reached
		if(Music_CurrentPacket == S_NULL)
		{
			Music_ReadIndex = 0;
			Music_Play = 0;
			setBuzzer(0);
			return;
		}

		// Handle wave packets
		if(Music_CurrentPacket & S_WAVE)
		{
			Music_WaveFlag = 1;
			if(Music_CurrentPacket & S_WAVE_SIGN)
				Music_WaveStep = -((s16) Music_CurrentSound);
			else
				Music_WaveStep = (s16) Music_CurrentSound;
			Music_TempoCnt = Music_CurrentLen * Music_TempoContainer;
			Music_ReadIndex++;
			return;
		}
		else
			Music_WaveFlag = 0;

		// Handle continue packets
		if(Music_CurrentSound == S_CONTINUE)
		{
			Music_TempoCnt = Music_CurrentLen * Music_TempoContainer;
			Music_ReadIndex++;
			return;
		}

		// Delay the start of a new packet by one iteration
		if(Music_StartDelayFlag == 1)
		{
			Music_StartDelayFlag = 0;
			Music_TempoCnt = Music_CurrentLen * Music_TempoContainer;
			setBuzzer(SoundTable[Music_CurrentSound]);
			Music_WaveBuffer = SoundTable[Music_CurrentSound];
			Music_ReadIndex++;
		}
		else
			Music_StartDelayFlag = 1;
	}
}

//
// Play functions
//

// Start the playback of a sound based on the currently set buzzer data and play length
void PlayBuzzer(void)
{
	// Play music or a doremi tone as required
	if(gbBuzzerPlayLength == 0xFF)
		PlayMusic(gbBuzzerData);
	else
		PlayDoremi(gbBuzzerData, gbBuzzerPlayLength);
}

// Play a certain pre-defined music from the music table
void PlayMusic(u8 musicIndex)
{
	// If already playing something then ignore the request
	if(Music_Play || Doremi_Play) return;

	// Initialise the playback of the music
	if(musicIndex < MUSIC_N)
	{
		Music_MusicIndex = musicIndex;
		Music_ReadIndex = 0;
		Music_Play = 1;
	}
}

// Play a doremi tone for a given duration (the units of playTime are in 100ms)
void PlayDoremi(u16 index, u16 playTime)
{
	// If currently playing music then ignore the request
	if(Music_Play) return;

	// Saturate the index by the size of the doremi table
	if(index >= SIZE_DOREMITABLE)
		index = SIZE_DOREMITABLE - 1;

	// Adjust the play time as required
	if(playTime > 50)
		playTime = 50;
	else if(playTime == 0)
		playTime = 3;

	// Initialise the playback of the doremi tone for the desired duration
	Doremi_Index = index;
	Doremi_TimCount = playTime * 25;
	Doremi_Play = 1;
}

//
// Buzzer functions
//

// Set the period of the buzzer
void setBuzzer(u16 periodUs)
{
	// Adjust the timer 4 reload and compare value (duty cycle of 50%)
	TIM_SetAutoreload(TIM4, periodUs);
	TIM_SetCompare4(TIM4, periodUs / 2);
}

// Get the state of the buzzer (non-zero if something is currently playing)
u8 getBuzzerState(void)
{
	// Return whether a sound is currently being played
	return (Music_Play || Doremi_Play);
}

// Set the current play length of the buzzer
void setBuzzerPlayLength(u8 length)
{
	gbBuzzerPlayLength = length;
}

// Retrieve the current buzzer play length
u8 getBuzzerPlayLength(void)
{
	return gbBuzzerPlayLength;
}

// Set the current data for the buzzer
void setBuzzerData(u8 data)
{
	gbBuzzerData = data;
}

// Retrieve the current data for the buzzer
u8 getBuzzerData(void)
{
	return gbBuzzerData;
}

// Stop any currently playing sound and turn off the buzzer
void setBuzzerOff(void)
{
	Music_Play = Doremi_Play = 0;
	setBuzzer(0);
}
/************************ (C) COPYRIGHT 2010 ROBOTIS *********END OF FILE******/
