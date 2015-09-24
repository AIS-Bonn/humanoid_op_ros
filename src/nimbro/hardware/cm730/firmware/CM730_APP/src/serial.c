/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : serial.c
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/26
* Description        : Functions relating to PC serial communications
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*                      This file has been commented out to ensure that it is not
*                      used in conjunction with the remaining firmware code.
*******************************************************************************/

// Includes
#include "serial.h"
#include "usart.h"

// // Transmit a string
// void TxDString(u8 *bData)
// {
// 	while(*bData)
// 		TxDData(USART_PC, *bData++);
// }
// 
// // Transmit an 8-bit integer as two hexadecimal digits (MSB first)
// void TxDHexU8(u8 bData)
// {
// 	u8 bTmp;
// 
// 	bTmp = ((bData >> 4) & 0x0F) + ((u8) '0');
// 	if(bTmp > '9') bTmp += 7;
// 	TxDData(USART_PC, bTmp);
// 
// 	bTmp = (bData & 0x0F) + ((u8) '0');
// 	if(bTmp > '9') bTmp += 7;
// 	TxDData(USART_PC, bTmp);
// }
// 
// // Transmit a 16-bit integer as four hexadecimal digits (MSB first)
// void TxDHexU16(u16 wData)
// {
// 	TxDHexU8((u8) (wData >> 8));
// 	TxDHexU8((u8) (wData & 0x00FF));
// }
// 
// // Transmit a 32-bit integer as eight hexadecimal digits (MSB first)
// void TxDHexU32(u32 lData)
// {
// 	TxDHexU16((u16) (lData >> 16));
// 	TxDHexU16((u16) (lData & 0x0000FFFF));
// }
// 
// // Transmit an 8-bit integer as up to three decimal digits padded with spaces (right-aligned, most significant first)
// void TxDDecU8(u8 bData)
// {
// 	u8 bDigit, bTmp;
// 	u8 bPrinted = 0;
// 
// 	for(bDigit = 100; bDigit > 0; bDigit /= 10)
// 	{
// 		bTmp = bData / bDigit;
// 		if((bTmp != 0) || bPrinted)
// 		{
// 			TxDData(USART_PC, bTmp + '0');
// 			bPrinted = 1;
// 		}
// 		else
// 		{
// 			if(bDigit > 1) TxDData(USART_PC, ' ');
// 			else TxDData(USART_PC, '0');
// 		}
// 		bData -= bTmp * bDigit;
// 	}
// }
// 
// // Transmit a 16-bit integer as up to five decimal digits padded with spaces (right-aligned, most significant first)
// void TxDDecU16(u16 wData)
// {
// 	u16 wDigit, wTmp;
// 	u8 bPrinted = 0;
// 
// 	for(wDigit = 10000; wDigit > 0; wDigit /= 10)
// 	{
// 		wTmp = wData / wDigit;
// 		if((wTmp != 0) || bPrinted)
// 		{
// 			TxDData(USART_PC, ((u8) wTmp) + '0');
// 			bPrinted = 1;
// 		}
// 		else
// 		{
// 			if(wDigit > 1) TxDData(USART_PC, ' ');
// 			else TxDData(USART_PC, '0');
// 		}
// 		wData -= wTmp * wDigit;
// 	}
// }
// 
// // Transmit a 32-bit integer as up to ten decimal digits padded with spaces (right-aligned, most significant first)
// void TxDDecU32(u32 lData)
// {
// 	u32 lDigit, lTmp;
// 	u8 bPrinted = 0;
// 
// 	for(lDigit = 1000000000; lDigit > 0; lDigit /= 10)
// 	{
// 		lTmp = lData / lDigit;
// 		if((lTmp != 0) || bPrinted)
// 		{
// 			TxDData(USART_PC, ((u8) lTmp) + '0');
// 			bPrinted = 1;
// 		}
// 		else
// 		{
// 			if(lDigit > 1) TxDData(USART_PC, ' ');
// 			else TxDData(USART_PC, '0');
// 		}
// 		lData -= lTmp * lDigit;
// 	}
// }
// 
// // Transmit an 8-bit integer as a sign and up to three decimal digits padded with spaces (right-aligned, most significant first)
// void TxDDecS8(s8 bData)
// {
// 	u8 bUData, bDigit, bTmp;
// 	u8 bMinus, bPrinted = 0;
// 
// 	bMinus = (bData & 0x80 != 0);
// 	if(bMinus) bUData = -bData;
// 	else       bUData =  bData;
// 
// 	for(bDigit = 100; bDigit > 0; bDigit /= 10)
// 	{
// 		bTmp = bUData / bDigit;
// 		if((bTmp != 0) || bPrinted)
// 		{
// 			if(!bPrinted)
// 			{
// 				if(bMinus) TxDData(USART_PC, '-');
// 				else TxDData(USART_PC, ' ');
// 			}
// 			TxDData(USART_PC, bTmp + '0');
// 			bPrinted = 1;
// 		}
// 		else
// 		{
// 			if(bDigit > 1) TxDData(USART_PC, ' ');
// 			else TxDData(USART_PC, '0');
// 		}
// 		bUData -= bTmp * bDigit;
// 	}
// }
// 
// // Transmit a 16-bit integer as a sign and up to five decimal digits padded with spaces (right-aligned, most significant first)
// void TxDDecS16(s16 wData)
// {
// 	u16 wUData, wDigit, wTmp;
// 	u8 bMinus, bPrinted = 0;
// 
// 	bMinus = (wData & 0x8000 != 0);
// 	if(bMinus) wUData = -wData;
// 	else       wUData =  wData;
// 
// 	for(wDigit = 10000; wDigit > 0; wDigit /= 10)
// 	{
// 		wTmp = wUData / wDigit;
// 		if((wTmp != 0) || bPrinted)
// 		{
// 			if(!bPrinted)
// 			{
// 				if(bMinus) TxDData(USART_PC, '-');
// 				else TxDData(USART_PC, ' ');
// 			}
// 			TxDData(USART_PC, ((u8) wTmp) + '0');
// 			bPrinted = 1;
// 		}
// 		else
// 		{
// 			if(wDigit > 1) TxDData(USART_PC, ' ');
// 			else TxDData(USART_PC, '0');
// 		}
// 		wUData -= wTmp * wDigit;
// 	}
// }
// 
// // Transmit a 32-bit integer as a sign and up to ten decimal digits padded with spaces (right-aligned, most significant first)
// void TxDDecS32(s32 lData)
// {
// 	u32 lUData, lDigit, lTmp;
// 	u8 bMinus, bPrinted = 0;
// 
// 	bMinus = (lData & 0x80000000 != 0);
// 	if(bMinus) lUData = -lData;
// 	else       lUData =  lData;
// 
// 	for(lDigit = 1000000000; lDigit > 0; lDigit /= 10)
// 	{
// 		lTmp = lUData / lDigit;
// 		if((lTmp != 0) || bPrinted)
// 		{
// 			if(!bPrinted)
// 			{
// 				if(bMinus) TxDData(USART_PC, '-');
// 				else TxDData(USART_PC, ' ');
// 			}
// 			TxDData(USART_PC, ((u8) lTmp) + '0');
// 			bPrinted = 1;
// 		}
// 		else
// 		{
// 			if(lDigit > 1) TxDData(USART_PC, ' ');
// 			else TxDData(USART_PC, '0');
// 		}
// 		lUData -= lTmp * lDigit;
// 	}
// }
/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
