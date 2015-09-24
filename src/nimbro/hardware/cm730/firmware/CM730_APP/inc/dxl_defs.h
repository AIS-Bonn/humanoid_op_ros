/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : dxl_defs.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/26
* Description        : Contains defines for the dynamixel communications
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Ensure header is included only once
#ifndef DXL_DEFS_H
#define DXL_DEFS_H

// Packet ID's
#define INVALID_ID           0x00
#define BROADCAST_ID         0xFE

// Packet instructions
#define INST_NONE            0x00
#define INST_PING            0x01
#define INST_READ            0x02
#define INST_WRITE           0x03
#define INST_REG_WRITE       0x04
#define INST_ACTION          0x05
#define INST_RESET           0x06
#define INST_DIGITAL_RESET   0x07
#define INST_SYSTEM_READ     0x0C
#define INST_SYSTEM_WRITE    0x0D
#define INST_REPEAT_BULK     0x0F
#define INST_SYNC_WRITE      0x83
#define INST_SYNC_REG_WRITE  0x84
#define INST_BULK_READ       0x92

// Status flag error bits
#define ERRBIT_VOLTAGE       0x01
#define ERRBIT_ANGLE_LIMIT   0x02
#define ERRBIT_OVERHEATING   0x04
#define ERRBIT_RANGE         0x08
#define ERRBIT_CHECKSUM      0x10
#define ERRBIT_OVERLOAD      0x20
#define ERRBIT_INSTRUCTION   0x40

#endif /* DXL_DEFS_H */
/************************ (C) COPYRIGHT 2010 ROBOTIS *********END OF FILE******/
