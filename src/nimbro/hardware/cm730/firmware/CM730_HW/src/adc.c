/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : adc.c
* Author             : zerom
* Version            : V0.1
* Date               : 2010/09/08
* Description        : Contains the functions and defines for ADC conversions
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Includes
#include "adc.h"
#include "stm32f10x_lib.h"
#include "CM_DXL_COM.h"

// Defines
#define RESCALE_BATTERY_VOLTAGE(adcbits)  (gbVoltageTable[(adcbits >> 4) & 0x00FF]) // adcbits is expected to be a 12-bit (unsigned) ADC measurement

// Voltage rescaling lookup table
// Note: The following lookup table implements the multiplication of an 8-bit integer by 0.98828. This is used to scale the measured
//       ADC_VOLTAGE conversion result (a measure of the battery voltage) so that 10 corresponds to 1.0V (i.e. 164 corresponds to 16.4V).
//       The STM32F103xx chip used on the CM730 is an LQFP64 package, which doesn't have a pin for the ADC reference voltage VREF+.
//       Instead, VREF+ is internally tied to VDDA, which is kept at VDD = 3.3V on the CM730. The VREF- reference voltage is tied to
//       VSSA (always), and is hence at a potential of 0.0V. Thus the 12-bit ADC converter has a range of 0-4095 for ADC input voltages
//       of 0-3.3V. On the CM730, the battery voltage is passed through a 3.3kOhm / 22kOhm voltage divider to generate the ADC_VOLTAGE
//       analogue signal. Hence ADC_VOLTAGE = (3.3/(3.3 + 22))*VBattery = 3.3*VBattery/25.3, and so the ADC value after conversion
//       (in bits) is (4096/3.3 bits per V)*(3.3*VBattery/25.3) = 4096*VBattery/25.3. We now wish to scale this bit value so that 10
//       corresponds to a VBattery of 1.0V => Need to scale by 10*25.3/4096 = (253/256)/16 = 0.98828/16. This is implemented by first
//       right-shifting by 4 bits (division by 16) and then passing the resulting 8-bit variable through the following voltage table.
//       Refer to the RESCALE_BATTERY_VOLTAGE() macro.
const u8 gbVoltageTable[256] = {
	0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41,
	42, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82,
	83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
	120, 121, 122, 123, 124, 125, 126, 127, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152,
	153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186,
	187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 211, 212, 213, 214, 215, 216, 217, 218, 219,
	220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252};

// ADC channels
const u8 ADC_Channel[ADC_NUMBER] =
{
	// ADC1
	ADC_Channel_10,
	ADC_Channel_11,
	ADC_Channel_12,
	ADC_Channel_13,
	ADC_Channel_0,
	ADC_Channel_1,
	ADC_Channel_2,
	ADC_Channel_3,

	// ADC2
	ADC_Channel_4,
	ADC_Channel_5,
	ADC_Channel_6,
	ADC_Channel_7,
	ADC_Channel_14,
	ADC_Channel_15,
	ADC_Channel_8,
	ADC_Channel_9
};

// File scope variables
vu8  ADC_Channel_Index = 0;
vu16 ADC_Value[ADC_NUMBER] = {0};

// Initialisation function for the ADC interrupt service routine
void initADCISR(void)
{
	// Declare variables
	u8 PORT;

	// Initialise the ADC values array
	for(PORT = 0; PORT < ADC_NUMBER; PORT++)
		ADC_Value[PORT] = 0;

	// Initialise the channel index variable
	ADC_Channel_Index = 0;

	// Set up the first pair of ADC conversions
	ADC_RegularChannelConfig(ADC1, ADC_Channel[ADC_Channel_Index    ], 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel[ADC_Channel_Index + 8], 1, ADC_SampleTime_239Cycles5);
}

// Get function for the latest ADC result for the given port
u16 getADC(u8 ADCPort)
{
	// Return the latest available ADC conversion value (12-bit value that is right-aligned in 16 bits)
	return ADC_Value[ADCPort];
}

// Retrieve the current battery voltage
u8 getBatteryVoltage(void)
{
	// Return the battery voltage, converting the raw ADC value so that 10 = 1.0V (i.e. 164 corresponds to 16.4V)
	return RESCALE_BATTERY_VOLTAGE(ADC_Value[ADC_VOLTAGE]);
}

// Interrupt service routine to retrieve the ADC results
void __ISR_ADC(void)
{
	// Note: The ADC peripherals are configured to run continuous conversions (see ADC_Configuration()),
	//       with the channel to convert at each point in time being defined as the channel that was last
	//       passed to ADC_RegularChannelConfig() for the corresponding ADC peripheral. It is assumed that
	//       this ISR is called at a rate slower than the continuous ADC conversion rate, meaning that the
	//       latest (and guaranteed to be existing) measurements of the current channel pair are always
	//       available to us already.

	// Declare variables
	u8 bCount;

	// Retrieve the latest conversion results of the current channel pair
	ADC_Value[ADC_Channel_Index    ] = ADC_GetConversionValue(ADC1);
	ADC_Value[ADC_Channel_Index + 8] = ADC_GetConversionValue(ADC2);

	// Increment the channel index (move on to the next channel pair)
	ADC_Channel_Index++;

	// If we have finished going through all the channel pairs...
	if(ADC_Channel_Index == 8)
	{
		// ...then return to the first channel pair again.
		ADC_Channel_Index = 0;

		// Update the control table with the conversion results of the latest sweep through the channels
		for(bCount = 0; bCount < 16; bCount++)
			GW_CONTROL_TABLE(P_ADC0_BATTERY + (bCount << 1)) = ADC_Value[bCount];
	}

	// Set up the ADC conversion for the new channel pair
	ADC_RegularChannelConfig(ADC1, ADC_Channel[ADC_Channel_Index    ], 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel[ADC_Channel_Index + 8], 1, ADC_SampleTime_239Cycles5);
}
/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
