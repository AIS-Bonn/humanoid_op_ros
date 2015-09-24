/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : gyro_acc.h
* Author             : danceww
* Version            : V0.0.1
* Date               : 2011/01/15
* Description        : Contains the functions and defines for the gyro/acc sensors
* Comment            : This file has been heavily modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14),
*                      and is intended for use with CM730's that have the L3G4200D
*                      gyroscope chip and the LIS331DLH accelerometer chip.
*******************************************************************************/

// Ensure header is only included once
#ifndef GYRO_ACC_H
#define GYRO_ACC_H

// Includes
#include "stm32f10x_type.h"

// Configuration functions for the gyroscope and accelerometer
void ConfigureGyro();
void ConfigureAcc();

// Interrupt service routine to read the gyroscope and accelerometer data
void __ISR_GYRO_ACC_SPI();

// Externed variables
extern vu8 GyroAccAvgBufClear;

#endif /* GYRO_ACC_H */
/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
