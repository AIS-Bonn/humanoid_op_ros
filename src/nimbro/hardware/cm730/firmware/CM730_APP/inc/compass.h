/******************************************************************************
* File Name          : compass.h
* Author             : Max Schwarz <max.schwarz@uni-bonn.de>
*                    : Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
* Version            : V0.1
* Date               : 02/04/14
* Description        : HMC5883L 3-axis compass driver
*******************************************************************************/

// Ensure header is only included once
#ifndef COMPASS_H
#define COMPASS_H

// Compass configuration function
void ConfigureCompass(void);

// Interrupt service routine to handle the compass communications
void __ISR_COMPASS_I2C(void);

#endif /* COMPASS_H */
/******************************** END OF FILE *********************************/
