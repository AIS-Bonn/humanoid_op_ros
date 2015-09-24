/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : led.c
* Author             : zerom
* Version            : V0.0.1
* Date               : 2010/08/23
* Description        : Contains the functions and defines for the LEDs
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Includes
#include "led.h"
#include "stm32f10x_lib.h"
#include "system_init.h"
#include "CM_DXL_COM.h"

// Defines
#define BLINK_COUNT  8    // Number of times to blink the LEDs on USB enable
#define BLINK_TIME   1037 // Half-period of LED blinking (482us * 1037 ~= 500ms, refer to ISR_TIMER2() for the 482us)

// Global variables
volatile byte gbLEDHeadR = 0;
volatile byte gbLEDHeadG = 0;
volatile byte gbLEDHeadB = 0;
volatile byte gbLEDEyeR = 0;
volatile byte gbLEDEyeG = 0;
volatile byte gbLEDEyeB = 0;
volatile byte gbLEDPwm = 0;
vu8  bLEDBlinkFlag = 0;
vu8  bLEDCounter = 0;
vu16 bLEDTimer = 0;

// Interrupt service routine to control the RGB LEDs
void __ISR_LED_RGB_TIMER(void)
{
	// Increment the LED PWM variable (wraps to 0 on reaching 32)
	gbLEDPwm = (gbLEDPwm + 1) & 0x1F;

	// Control LED5 depending on whether the USB is connected or not
	if(GPIO_ReadInputDataBit(PORT_USB_SLEEP, PIN_USB_SLEEP) != SET) // USB disconnected...
	{
		// If a non-zero count of desired blinks remains...
		if(bLEDCounter)
		{
			bLEDTimer--;
			if(bLEDTimer == 0) // If the blink time has just elapsed for the current blink...
			{
				bLEDCounter--; // One less blink still to do
				bLEDTimer = BLINK_TIME; // Reset the blink timer
				bLEDBlinkFlag = !bLEDBlinkFlag; // Toggle the state of the LED
				if(bLEDCounter == 0) bLEDBlinkFlag = 0; // Ensure that the final state of the LED is consistent, no matter the parity of bLEDCounter
			}
		}

		// Blink LED5 red and green
		if(bLEDBlinkFlag)
		{
			GPIO_SetBits(PORT_LED5_R, PIN_LED5_R);
			GPIO_ResetBits(PORT_LED5_G, PIN_LED5_G);
			GPIO_SetBits(PORT_LED5_B, PIN_LED5_B);
		}
		else
		{
			GPIO_ResetBits(PORT_LED5_R, PIN_LED5_R);
			GPIO_SetBits(PORT_LED5_G, PIN_LED5_G);
			GPIO_SetBits(PORT_LED5_B, PIN_LED5_B);
		}
	}
	else // USB connected...
	{
		// Reinitialise the blink count and time variables
		bLEDCounter = BLINK_COUNT;
		bLEDTimer = BLINK_TIME;

		// Pulse width modulate the RGB LED5
		if(gbLEDPwm >= gbLEDHeadR) GPIO_SetBits(PORT_LED5_R, PIN_LED5_R);
		else                     GPIO_ResetBits(PORT_LED5_R, PIN_LED5_R);
		if(gbLEDPwm >= gbLEDHeadG) GPIO_SetBits(PORT_LED5_G, PIN_LED5_G);
		else                     GPIO_ResetBits(PORT_LED5_G, PIN_LED5_G);
		if(gbLEDPwm >= gbLEDHeadB) GPIO_SetBits(PORT_LED5_B, PIN_LED5_B);
		else                     GPIO_ResetBits(PORT_LED5_B, PIN_LED5_B);
	}

	// Pulse width modulate the RGB LED6
	if(gbLEDPwm >= gbLEDEyeR) GPIO_SetBits(PORT_LED6_R, PIN_LED6_R);
	else                    GPIO_ResetBits(PORT_LED6_R, PIN_LED6_R);
	if(gbLEDPwm >= gbLEDEyeG) GPIO_SetBits(PORT_LED6_G, PIN_LED6_G);
	else                    GPIO_ResetBits(PORT_LED6_G, PIN_LED6_G);
	if(gbLEDPwm >= gbLEDEyeB) GPIO_SetBits(PORT_LED6_B, PIN_LED6_B);
	else                    GPIO_ResetBits(PORT_LED6_B, PIN_LED6_B);
}

// Retrieve the state of a particular LED
PowerState LED_GetState(u8 LED_PORT)
{
	// Return the state of the LED corresponding to the given port
	     if(LED_PORT == LED_MANAGE) return (GPIO_ReadOutputDataBit(PORT_LED_MANAGE, PIN_LED_MANAGE) == SET ? OFF : ON);
	else if(LED_PORT == LED_EDIT)   return (GPIO_ReadOutputDataBit(PORT_LED_EDIT  , PIN_LED_EDIT  ) == SET ? OFF : ON);
	else if(LED_PORT == LED_PLAY)   return (GPIO_ReadOutputDataBit(PORT_LED_PLAY  , PIN_LED_PLAY  ) == SET ? OFF : ON);
	else if(LED_PORT == LED_TX)     return (GPIO_ReadOutputDataBit(PORT_LED_TX    , PIN_LED_TX    ) == SET ? OFF : ON);
	else if(LED_PORT == LED_RX)     return (GPIO_ReadOutputDataBit(PORT_LED_RX    , PIN_LED_RX    ) == SET ? OFF : ON);
	else return OFF;
}

// Set the state of a particular LED or LEDs
void LED_SetState(u8 LED_PORT, PowerState NewState)
{
	// Either set or reset the corresponding GPIO pin... (Note: The LED pins are active low!)
	if(NewState == ON)
	{ 
		if(LED_PORT & LED_MANAGE) GPIO_ResetBits(PORT_LED_MANAGE, PIN_LED_MANAGE);
		if(LED_PORT & LED_EDIT  ) GPIO_ResetBits(PORT_LED_EDIT  , PIN_LED_EDIT  );
		if(LED_PORT & LED_PLAY  ) GPIO_ResetBits(PORT_LED_PLAY  , PIN_LED_PLAY  );
		if(LED_PORT & LED_TX    ) GPIO_ResetBits(PORT_LED_TX    , PIN_LED_TX    );
		if(LED_PORT & LED_RX    ) GPIO_ResetBits(PORT_LED_RX    , PIN_LED_RX    );
		GB_LED_PANEL = (GB_LED_PANEL | LED_PORT) & LED_ALL;
	}
	else
	{
		if(LED_PORT & LED_MANAGE) GPIO_SetBits(PORT_LED_MANAGE, PIN_LED_MANAGE);
		if(LED_PORT & LED_EDIT  ) GPIO_SetBits(PORT_LED_EDIT  , PIN_LED_EDIT  );
		if(LED_PORT & LED_PLAY  ) GPIO_SetBits(PORT_LED_PLAY  , PIN_LED_PLAY  );
		if(LED_PORT & LED_TX    ) GPIO_SetBits(PORT_LED_TX    , PIN_LED_TX    );
		if(LED_PORT & LED_RX    ) GPIO_SetBits(PORT_LED_RX    , PIN_LED_RX    );
		GB_LED_PANEL &= ~LED_PORT & LED_ALL;
	}
}

// Get the state of the RGB LED5
u8 LED5_GetState()
{
	// Declare variables
	u8 rgb = 0;

	// OR together the flags of the colour components that are currently on
	if(GPIO_ReadOutputDataBit(PORT_LED5_R, PIN_LED5_R) == SET) rgb |= LED_R;
	if(GPIO_ReadOutputDataBit(PORT_LED5_G, PIN_LED5_G) == SET) rgb |= LED_G;
	if(GPIO_ReadOutputDataBit(PORT_LED5_B, PIN_LED5_B) == SET) rgb |= LED_B;

	// Return the required OR-ed value
	return rgb;
}

// Set the state of the RGB LED5
void LED5_SetState(u8 RGB)
{
	// Set the individual colour component output pins as necessary
	if(RGB & LED_R) GPIO_ResetBits(PORT_LED5_R, PIN_LED5_R);
	else            GPIO_SetBits  (PORT_LED5_R, PIN_LED5_R);
	if(RGB & LED_G) GPIO_ResetBits(PORT_LED5_G, PIN_LED5_G);
	else            GPIO_SetBits  (PORT_LED5_G, PIN_LED5_G);
	if(RGB & LED_B) GPIO_ResetBits(PORT_LED5_B, PIN_LED5_B);
	else            GPIO_SetBits  (PORT_LED5_B, PIN_LED5_B);
}

// Set the colour of an RGB LED (RGB values should be specified 0-255, they are internally right-shifted to obtain 5-bit values)
void RGBLED_SetColour(u8 RGBLED_PORT, u8 R, u8 G, u8 B)
{
	// Set the colour of the appropriate RGB LED
	if(RGBLED_PORT == RGBLED5)      // Note: LED5 == HEAD
	{
		gbLEDHeadR = (R >> 3);
		gbLEDHeadG = (G >> 3);
		gbLEDHeadB = (B >> 3);
	}
	else if(RGBLED_PORT == RGBLED6) // Note: LED6 == EYE
	{
		gbLEDEyeR = (R >> 3);
		gbLEDEyeG = (G >> 3);
		gbLEDEyeB = (B >> 3);
	}
}
/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
