/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : CM_DXL_COM.h
* Author             : dudung
* Version            : V0.1
* Date               : 2010/11/03
* Description        : Implements the PC and Dynamixel communication architecture,
*                      and the general main function of the CM730
* Comment            : This file has been completely rewritten by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Ensure header is included only once
#ifndef CM_DXL_COM_H
#define CM_DXL_COM_H

// Includes
#include "common_type.h"

// Generic control table access macro
#define GB_CONTROL_TABLE(P)     (gbControlTable[(P)])
#define GW_CONTROL_TABLE(P)     (WORD_CAST(gbControlTable[(P)]))

// CM730 ROM register addresses (standard format for all Dynamixel bus devices)
#define P_MODEL_NUMBER_L        0  // Model number (L)
#define P_MODEL_NUMBER_H        1  // Model Number (H)
#define P_VERSION               2  // Firmware version
#define P_ID                    3  // Device ID on Dynamixel bus
#define P_BAUD_RATE             4  // Baud rate
#define P_RETURN_DELAY_TIME     5  // Return delay time
#define P_CW_ANGLE_LIMIT_L      6  // Clockwise angle limit (L)
#define P_CW_ANGLE_LIMIT_H      7  // Clockwise angle limit (H)
#define P_CCW_ANGLE_LIMIT_L     8  // Counterclockwise angle limit (L)
#define P_CCW_ANGLE_LIMIT_H     9  // Counterclockwise angle limit (H)
#define P_SYSTEM_DATA2          10 // MX106 => Drive mode, MX64 => Reserved
#define P_LIMIT_TEMPERATURE     11 // Internal limit temperature
#define P_VOLTAGE_LOWER_LIMIT   12 // Voltage lower limit
#define P_VOLTAGE_UPPER_LIMIT   13 // Voltage upper limit
#define P_MAX_TORQUE_L          14 // Maximum torque (L)
#define P_MAX_TORQUE_H          15 // Maximum torque (H)
#define P_RETURN_LEVEL          16 // Status return level
#define P_ALARM_LED             17 // Alarm LED (this register is used for the CM730 to signal error conditions, and not the ALARM_SHUTDOWN one)
#define P_ALARM_SHUTDOWN        18 // Alarm shutdown
#define P_OPERATING_MODE        19 // Custom
#define P_DOWN_CALIBRATION_L    20 // Custom
#define P_DOWN_CALIBRATION_H    21 // Custom
#define P_UP_CALIBRATION_L      22 // Custom
#define P_UP_CALIBRATION_H      23 // Custom
#define ROM_CONTROL_TABLE_LEN   24 // Number of ROM registers

// CM730 ROM register values
#define GW_MODEL_NUMBER         GW_CONTROL_TABLE(P_MODEL_NUMBER_L)
#define GB_VERSION              GB_CONTROL_TABLE(P_VERSION)
#define GB_ID                   GB_CONTROL_TABLE(P_ID)
#define GB_BAUD_RATE            GB_CONTROL_TABLE(P_BAUD_RATE)
#define GB_RETURN_DELAY_TIME    GB_CONTROL_TABLE(P_RETURN_DELAY_TIME)
#define GW_CW_ANGLE_LIMIT       GW_CONTROL_TABLE(P_CW_ANGLE_LIMIT_L)
#define GW_CCW_ANGLE_LIMIT      GW_CONTROL_TABLE(P_CCW_ANGLE_LIMIT_L)
#define GB_SYSTEM_DATA2         GB_CONTROL_TABLE(P_SYSTEM_DATA2)
#define GB_LIMIT_TEMPERATURE    GB_CONTROL_TABLE(P_LIMIT_TEMPERATURE)
#define GB_VOLTAGE_LOWER_LIMIT  GB_CONTROL_TABLE(P_VOLTAGE_LOWER_LIMIT)
#define GB_VOLTAGE_UPPER_LIMIT  GB_CONTROL_TABLE(P_VOLTAGE_UPPER_LIMIT)
#define GW_MAX_TORQUE           GW_CONTROL_TABLE(P_MAX_TORQUE_L)
#define GB_RETURN_LEVEL         GB_CONTROL_TABLE(P_RETURN_LEVEL)
#define GB_ALARM_LED            GB_CONTROL_TABLE(P_ALARM_LED)
#define GB_ALARM_SHUTDOWN       GB_CONTROL_TABLE(P_ALARM_SHUTDOWN)
#define GB_OPERATING_MODE       GB_CONTROL_TABLE(P_OPERATING_MODE)
#define GW_DOWN_CALIBRATION     GW_CONTROL_TABLE(P_DOWN_CALIBRATION_L)
#define GW_UP_CALIBRATION       GW_CONTROL_TABLE(P_UP_CALIBRATION_L)

// CM730 RAM register addresses (commented addresses reserve the memory location for the upper byte of a 16-bit value, e.g. P_MAG_X is a 16-bit value stored in registers L32/H33
#define P_DYNAMIXEL_POWER       24
#define P_LED_PANEL             25
#define P_RGBLED5               26
//                              27
#define P_RGBLED6               28
//                              29
#define P_BUTTON                30
#define P_BATTERY_VOLTAGE       31
#define P_GYRO_X                32
//                              33
#define P_GYRO_Y                34
//                              35
#define P_GYRO_Z                36
//                              37
#define P_ACC_X                 38
//                              39
#define P_ACC_Y                 40
//                              41
#define P_ACC_Z                 42
//                              43
#define P_MAG_X                 44
//                              45
#define P_MAG_Y                 46
//                              47
#define P_MAG_Z                 48
//                              49
#define P_ADC0_BATTERY          50
//                              51
#define P_ADC1_MIC1             52
//                              53
#define P_ADC2_MIC2             54
//                              55
#define P_ADC3                  56
//                              57
#define P_ADC4                  58
//                              59
#define P_ADC5                  60
//                              61
#define P_ADC6                  62
//                              63
#define P_ADC7                  64
//                              65
#define P_ADC8                  66
//                              67
#define P_ADC9                  68
//                              69
#define P_ADC10                 70
//                              71
#define P_ADC11                 72
//                              73
#define P_ADC12                 74
//                              75
#define P_ADC13                 76
//                              77
#define P_ADC14                 78
//                              79
#define P_ADC15                 80
//                              81
#define P_BUZZER_DATA           82
#define P_BUZZER_PLAY_LENGTH    83
#define P_ZIGBEE_ID             84
//                              85
#define P_TX_REMOCON_DATA       86
//                              87
#define P_RX_REMOCON_DATA       88
//                              89
#define P_RX_REMOCON_DATA_ARR   90
#define P_DXLRX_PACKET_CNT      91
//                              92
#define P_DXLRX_OVERFLOW_CNT    93
//                              94
#define P_DXLRX_BUFERROR_CNT    95
//                              96
#define P_DXLRX_CHKERROR_CNT    97
//                              98
#define P_DXLRX_ORE_CNT         99
//                              100
#define P_DXLTX_PACKET_CNT      101
//                              102
#define P_DXLTX_OVERFLOW_CNT    103
//                              104
#define P_DXLTX_BUFERROR_CNT    105
//                              106
#define P_PCRX_PACKET_CNT       107
//                              108
#define P_PCRX_OVERFLOW_CNT     109
//                              110
#define P_PCRX_BUFERROR_CNT     111
//                              112
#define P_PCRX_CHKERROR_CNT     113
//                              114
#define P_PCRX_ORE_CNT          115
//                              116
#define P_PCTX_PACKET_CNT       117
//                              118
#define P_PCTX_OVERFLOW_CNT     119
//                              120
#define P_PCTX_BUFERROR_CNT     121
//                              122
#define P_MISC0                 123
#define P_MISC1                 124
#define P_MISC2                 125
#define P_MISC3                 126
#define CONTROL_TABLE_LEN       127 // Total number of registers (ROM + RAM)

// CM730 RAM register values
#define GB_DYNAMIXEL_POWER      GB_CONTROL_TABLE(P_DYNAMIXEL_POWER)
#define GB_LED_PANEL            GB_CONTROL_TABLE(P_LED_PANEL)
#define GW_RGBLED5              GW_CONTROL_TABLE(P_RGBLED5)
#define GW_RGBLED6              GW_CONTROL_TABLE(P_RGBLED6)
#define GB_BUTTON               GB_CONTROL_TABLE(P_BUTTON)
#define GB_BATTERY_VOLTAGE      GB_CONTROL_TABLE(P_BATTERY_VOLTAGE)
#define GW_GYRO_X               GW_CONTROL_TABLE(P_GYRO_X)
#define GW_GYRO_Y               GW_CONTROL_TABLE(P_GYRO_Y)
#define GW_GYRO_Z               GW_CONTROL_TABLE(P_GYRO_Z)
#define GW_ACC_X                GW_CONTROL_TABLE(P_ACC_X)
#define GW_ACC_Y                GW_CONTROL_TABLE(P_ACC_Y)
#define GW_ACC_Z                GW_CONTROL_TABLE(P_ACC_Z)
#define GW_MAG_X                GW_CONTROL_TABLE(P_MAG_X)
#define GW_MAG_Y                GW_CONTROL_TABLE(P_MAG_Y)
#define GW_MAG_Z                GW_CONTROL_TABLE(P_MAG_Z)
#define GB_ADC0_BATTERY         GW_CONTROL_TABLE(P_ADC0_BATTERY)
#define GW_ADC1_MIC1            GW_CONTROL_TABLE(P_ADC1_MIC1)
#define GW_ADC2_MIC2            GW_CONTROL_TABLE(P_ADC2_MIC2)
#define GW_ADC3                 GW_CONTROL_TABLE(P_ADC3)
#define GW_ADC4                 GW_CONTROL_TABLE(P_ADC4)
#define GW_ADC5                 GW_CONTROL_TABLE(P_ADC5)
#define GW_ADC6                 GW_CONTROL_TABLE(P_ADC6)
#define GW_ADC7                 GW_CONTROL_TABLE(P_ADC7)
#define GW_ADC8                 GW_CONTROL_TABLE(P_ADC8)
#define GW_ADC9                 GW_CONTROL_TABLE(P_ADC9)
#define GW_ADC10                GW_CONTROL_TABLE(P_ADC10)
#define GW_ADC11                GW_CONTROL_TABLE(P_ADC11)
#define GW_ADC12                GW_CONTROL_TABLE(P_ADC12)
#define GW_ADC13                GW_CONTROL_TABLE(P_ADC13)
#define GW_ADC14                GW_CONTROL_TABLE(P_ADC14)
#define GW_ADC15                GW_CONTROL_TABLE(P_ADC15)
#define GB_BUZZER_DATA          GB_CONTROL_TABLE(P_BUZZER_DATA)
#define GB_BUZZER_PLAY_LENGTH   GB_CONTROL_TABLE(P_BUZZER_PLAY_LENGTH)
#define GW_ZIGBEE_ID            GW_CONTROL_TABLE(P_ZIGBEE_ID)
#define GW_TX_REMOCON_DATA      GW_CONTROL_TABLE(P_TX_REMOCON_DATA)
#define GW_RX_REMOCON_DATA      GW_CONTROL_TABLE(P_RX_REMOCON_DATA)
#define GB_RX_REMOCON_DATA_ARR  GB_CONTROL_TABLE(P_RX_REMOCON_DATA_ARR)
#define GW_DXLRX_PACKET_CNT     GW_CONTROL_TABLE(P_DXLRX_PACKET_CNT)
#define GW_DXLRX_OVERFLOW_CNT   GW_CONTROL_TABLE(P_DXLRX_OVERFLOW_CNT)
#define GW_DXLRX_BUFERROR_CNT   GW_CONTROL_TABLE(P_DXLRX_BUFERROR_CNT)
#define GW_DXLRX_CHKERROR_CNT   GW_CONTROL_TABLE(P_DXLRX_CHKERROR_CNT)
#define GW_DXLRX_ORE_CNT        GW_CONTROL_TABLE(P_DXLRX_ORE_CNT)
#define GW_DXLTX_PACKET_CNT     GW_CONTROL_TABLE(P_DXLTX_PACKET_CNT)
#define GW_DXLTX_OVERFLOW_CNT   GW_CONTROL_TABLE(P_DXLTX_OVERFLOW_CNT)
#define GW_DXLTX_BUFERROR_CNT   GW_CONTROL_TABLE(P_DXLTX_BUFERROR_CNT)
#define GW_PCRX_PACKET_CNT      GW_CONTROL_TABLE(P_PCRX_PACKET_CNT)
#define GW_PCRX_OVERFLOW_CNT    GW_CONTROL_TABLE(P_PCRX_OVERFLOW_CNT)
#define GW_PCRX_BUFERROR_CNT    GW_CONTROL_TABLE(P_PCRX_BUFERROR_CNT)
#define GW_PCRX_CHKERROR_CNT    GW_CONTROL_TABLE(P_PCRX_CHKERROR_CNT)
#define GW_PCRX_ORE_CNT         GW_CONTROL_TABLE(P_PCRX_ORE_CNT)
#define GW_PCTX_PACKET_CNT      GW_CONTROL_TABLE(P_PCTX_PACKET_CNT)
#define GW_PCTX_OVERFLOW_CNT    GW_CONTROL_TABLE(P_PCTX_OVERFLOW_CNT)
#define GW_PCTX_BUFERROR_CNT    GW_CONTROL_TABLE(P_PCTX_BUFERROR_CNT)
#define GW_MISC0                GW_CONTROL_TABLE(P_MISC0) // Note: Although word access is provided for the misc variables,
#define GB_MISC0                GB_CONTROL_TABLE(P_MISC0) //       for the purposes of INST_READ/INST_WRITE they are treated
#define GB_MISC1                GB_CONTROL_TABLE(P_MISC1) //       as byte variables (see gbCTDataSize[]).
#define GW_MISC2                GW_CONTROL_TABLE(P_MISC2)
#define GB_MISC2                GB_CONTROL_TABLE(P_MISC2)
#define GB_MISC3                GB_CONTROL_TABLE(P_MISC3)

// Control table sizes
#define RAM_CONTROL_TABLE_LEN   (CONTROL_TABLE_LEN - ROM_CONTROL_TABLE_LEN) // Number of RAM registers

// Error bit flags
#define NO_ERROR_BIT           0x00
#define VOLTAGE_ERROR_BIT      0x01
#define ANGLE_LIMIT_ERROR_BIT  0x02
#define OVERHEATING_ERROR_BIT  0x04
#define RANGE_ERROR_BIT        0x08
#define CHECKSUM_ERROR_BIT     0x10
#define OVERLOAD_ERROR_BIT     0x20
#define INSTRUCTION_ERROR_BIT  0x40

// Default ROM values
#define DEFAULT_BAUD_RATE      1    // Baudrate (bps) = 2000000/(DEFAULT_BAUD_RATE + 1) = 1Mbps

// Communications logging types
#define COMMS_LOG_NONE         0
#define COMMS_LOG_PCRX         1
#define COMMS_LOG_PCTX         2
#define COMMS_LOG_DXLRX        3
#define COMMS_LOG_DXLTX        4
#define COMMS_LOG_CUSTOM       5

// Configure communications logging
#define GB_LOG_CONFIG          GB_MISC1 // Configuration of the communications logging
#define GB_LOGA_PTR            GB_MISC2 // Address of the register last written to in log A
#define GB_LOGB_PTR            GB_MISC3 // Address of the register last written to in log B
#define COMMS_LOGA_SIZE        64
#define COMMS_LOGB_SIZE        64
#define P_LOGA                 127 // Register range 127-->190 (64 bytes)
#define P_LOGB                 191 // Register range 191-->254 (64 bytes)

// Configure control table
#define EXTRA_CONTROL_TABLE    // Uncomment this line for some extra control table room (also inhibits range error checking in reads)
#ifdef EXTRA_CONTROL_TABLE
#define CT_EXTRA               128             // The extra control table registers are initialised to zero
#define COMMS_LOGA             COMMS_LOG_NONE  // Select the type of communications logging for log A (if COMMS_LOG_NONE then COMMS_LOGA_WRITE() must NOT be used)
#define COMMS_LOGB             COMMS_LOG_NONE  // Select the type of communications logging for log B (if COMMS_LOG_NONE then COMMS_LOGB_WRITE() must NOT be used)
#else
#define CT_EXTRA               0
#define COMMS_LOGA             COMMS_LOG_NONE
#define COMMS_LOGB             COMMS_LOG_NONE
#endif

// Communications logging macros (accept 8-bit byte arguments)
#define COMMS_LOGA_WRITE(b)    {if((++GB_LOGA_PTR) == P_LOGA + COMMS_LOGA_SIZE){GB_LOGA_PTR = P_LOGA;} GB_CONTROL_TABLE(GB_LOGA_PTR) = (b);}
#define COMMS_LOGB_WRITE(b)    {if((++GB_LOGB_PTR) == P_LOGB + COMMS_LOGB_SIZE){GB_LOGB_PTR = P_LOGB;} GB_CONTROL_TABLE(GB_LOGB_PTR) = (b);}

// Global variables
extern vu8 gbControlTable[];
extern vu8 gbAlarmState;

// Functions
void Process(void);
void DXLServoConfig(void);
void DXLServoTorqueOff(void);
void InitControlTable(void);
void OnControlTableWrite(u8 address);

#endif /* CM_DXL_COM_H */
/************************ (C) COPYRIGHT 2010 ROBOTIS *********END OF FILE******/
