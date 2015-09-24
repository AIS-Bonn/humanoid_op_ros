/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : system_init.c
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/25
* Description        : Functions relating to system initialisation
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Includes
#include "system_init.h"
#include "stm32f10x_lib.h"
#include "CM_DXL_COM.h"
#include "gyro_acc.h"
#include "compass.h"
#include "zigbee.h"
#include "sound.h"
#include "usart.h"
#include "adc.h"

// Functions
void Buzzer_Configuration(void);
void Timer_Configuration(void);
void SysTick_Configuration(void);
void RCC_Configuration(void);
void ADC_Configuration(void);
void SPI_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);

// Baudrates
u32 Baudrate_PC  = 2000000/((u32) (DEFAULT_BAUD_RATE + 1));
u32 Baudrate_DXL = 2000000/((u32) (DEFAULT_BAUD_RATE + 1));
u32 Baudrate_ZIG = 57600;

// Configure the entire system
void System_Configuration(void)
{
	// Disable interrupts
	GLOBAL_INTERRUPT_DISABLE();

	// Configure the system clocks
	RCC_Configuration(); // RCC = Reset and Clock Control

	// Configure the interrupts
	NVIC_Configuration(); // NVIC = Nested Vectored Interrupt Controller

	// Configure the GPIO ports
	GPIO_Configuration(); // GPIO = General-Purpose I/O

	// Unlock the Flash program/erase controller
	FLASH_Unlock();

	// Configure USART (Universal Synchronous Asynchronous Receiver Transmitter)
	USART_Configuration(USART_PC, Baudrate_PC);   // Note: Usually USART_Configuration() writes into the Baudrate_* variables, but in this special initial case we know they already have the value we need, so we just feed them in.
	USART_Configuration(USART_DXL, Baudrate_DXL);
	zgb_initialize(0);                            // Note: Internally calls USART_Configuration(USART_ZIG, 57600), which just makes sure even if !ALLOW_ZIGBEE that all the interrupts are disabled etc...

	// Initialise the USART port buffers and variables
	USARTInit(USART_ZIG);
	USARTInit(USART_DXL);
	USARTInit(USART_PC);

	// Disable automatic PC to DXL packet forwarding, and disable DXL Rx local buffering
	disableDXLForwarding();
	disableDXLBuffering();

	// Configure ADC (Analog-to-Digital Converter)
	ADC_Configuration();

	// Configure Cortex System Timer (SysTick)
	SysTick_Configuration();

	// Configure timers
	Timer_Configuration();

	// Configure SPI (Serial Peripheral Interface)
	GPIO_SetBits(PORT_SIG_GYRO_CS, PIN_SIG_GYRO_CS); // Deactivate chip select line of gyroscope
	GPIO_SetBits(PORT_SIG_ACC_CS, PIN_SIG_ACC_CS);   // Deactivate chip select line of accelerometer
	SPI_Configuration();

	// Configure the buzzer to run off a timer
	Buzzer_Configuration();
	setBuzzerOff();

	// Initialisation of GPIO pin states
	GPIO_ResetBits(PORT_ENABLE_TX, PIN_ENABLE_TX); // Disable USART1 TX
	GPIO_SetBits(PORT_ENABLE_RX, PIN_ENABLE_RX);   // Enable  USART1 RX

	// Configure the gyroscope, accelerometer and magnetometer
	ConfigureGyro();
	ConfigureAcc();
	ConfigureCompass();

	// Enable the Zigbee chip
#if ALLOW_ZIGBEE || IS_CM740 // The CM740 powers its compass over the Zigbee 3.3V power supply, so we wish to force it on
	Zigbee_SetState(ON);
#else
	Zigbee_SetState(OFF);
#endif

	// Initialise the values in the control table
	InitControlTable();
#if COMMS_LOGA != COMMS_LOG_NONE
	GB_LOGA_PTR = P_LOGA - 1;
#endif
#if COMMS_LOGB != COMMS_LOG_NONE
	GB_LOGB_PTR = P_LOGB - 1;
#endif
	GB_LOG_CONFIG = (COMMS_LOGA << 4) | COMMS_LOGB;

	// Enable interrupts
	GLOBAL_INTERRUPT_ENABLE();
}

// Configure the buzzer
void Buzzer_Configuration(void)
{
	// Declare and initialise variables
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);

	// Timer base configuration for the buzzer
	TIM_DeInit(TIM4);
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // 72 = 72MHz/1MHz where 1MHz is the resulting timer counter frequency
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 2000; // 2000 * 1MHz = 2kHz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	// Timer PWM configuration for the buzzer
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period / 2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Disable);

	// Enable the timer and the PWM output
	TIM_Cmd(TIM4, ENABLE);
	TIM_CtrlPWMOutputs(TIM4, ENABLE);
}

// Configure the timers
void Timer_Configuration(void)
{
	// Declare and initialise variables
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);

	// Timer 2 base configuration
	TIM_DeInit(TIM2);
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// Timer 2 prescaler configuration
	TIM_PrescalerConfig(TIM2, TIM2_PRESCALER, TIM_PSCReloadMode_Immediate);

	// Timer 2 output compare timing mode configuration
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	// Configure the capture compare register values for timer 2 (sets the frequency of the associated interrupts)
// 	TIM_OCInitStructure.TIM_Pulse = TIM2_CCR1_VAL;
// 	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
// 	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);
// 	TIM_OCInitStructure.TIM_Pulse = TIM2_CCR2_VAL;
// 	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
// 	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);
// 	TIM_OCInitStructure.TIM_Pulse = TIM2_CCR3_VAL;
// 	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
// 	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);
	TIM_OCInitStructure.TIM_Pulse = TIM2_CCR4_VAL;
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);

	// Enable the capture compare interrupts for timer 2
	TIM_ITConfig(TIM2, /*TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 |*/ TIM_IT_CC4 , ENABLE);

	// Enable timer 2
	TIM_Cmd(TIM2, ENABLE);
}

// Configure the Cortex System Timer
void SysTick_Configuration(void)
{
	  // Set up SysTick to elapse every 1ms (Cortex System Timer frequency = HCLK/8 = 72MHz/8 = 9MHz = 9000*1kHz)
	  SysTick_SetReload(9000);

	  // Enable SysTick interrupt
	  SysTick_ITConfig(ENABLE);
}

// Configure the system clocks
void RCC_Configuration(void)
{
	// Declare variables
	ErrorStatus HSEStartUpStatus;

	// RCC system reset(for debug purposes)
	RCC_DeInit();

	// Enable the High Speed External (HSE) clock
	RCC_HSEConfig(RCC_HSE_ON);

	// Wait until the HSE is ready
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	// Configure the Phase-Locked Loop (PLL) to drive the system clock
	if(HSEStartUpStatus == SUCCESS)
	{
		// Enable prefetch buffer
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		// Flash 2 wait state
		FLASH_SetLatency(FLASH_Latency_2);

		// HCLK = SYSCLK
		RCC_HCLKConfig(RCC_SYSCLK_Div1); 

		// PCLK2 = HCLK
		RCC_PCLK2Config(RCC_HCLK_Div1); 

		// PCLK1 = HCLK/2
		RCC_PCLK1Config(RCC_HCLK_Div2);

		// PLLCLK = 8MHz * 9 = 72 MHz (maximum clock frequency possible)
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		// Enable PLL
		RCC_PLLCmd(ENABLE);

		// Wait until PLL is ready
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}

		// Select PLL as the system clock source
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		// Wait until PLL is actually being used as the system clock source
		while(RCC_GetSYSCLKSource() != 0x08) {}
	} 
 
	// Enable the peripheral clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8 |
	                       RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
	                       RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5 |
	                       RCC_APB1Periph_USART3 | RCC_APB1Periph_UART5 | RCC_APB1Periph_SPI2 |
	                       RCC_APB1Periph_BKP | RCC_APB1Periph_PWR, ENABLE);

	// Enable access to the Real-Time Clock (RTC) and backup registers
	PWR_BackupAccessCmd(ENABLE);
}

// Configure USART
void USART_Configuration(u8 PORT, u32 baudrate)
{
	// Declare and initialise variables
	USART_InitTypeDef USART_InitStructure;
	USART_StructInit(&USART_InitStructure);
	
	// Populate the USART initialisation structure
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	// Configure the required USART port/connection
	if(PORT == USART_DXL)
	{
		// Save the baudrate in the appropriate variable
		Baudrate_DXL = baudrate;

		// Initialise the USART1 peripheral
		USART_DeInit(USART1);
		USART_Init(USART1, &USART_InitStructure);

		// Configure the USART1 interrupts we need
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  // Sets RXNEIE (enables RXNE, ORE interrupts)
		USART_ITConfig(USART1, USART_IT_TC, DISABLE);   // Resets TCIE (disables TC interrupt, this interrupt is enabled dynamically when it is needed)

		// Disable all other USART1 interrupts
		USART_ITConfig(USART1, USART_IT_TXE, DISABLE);  // Resets TXEIE
		USART_ITConfig(USART1, USART_IT_CTS, DISABLE);  // Resets CTSIE
		USART_ITConfig(USART1, USART_IT_IDLE, DISABLE); // Resets IDLEIE
		USART_ITConfig(USART1, USART_IT_PE, DISABLE);   // Resets PEIE
		USART_ITConfig(USART1, USART_IT_LBD, DISABLE);  // Resets LBDIE
		USART_ITConfig(USART1, USART_IT_ERR, DISABLE);  // Resets EIE (noise error, overrun error, frame error)

		// Enable the USART1 peripheral
		USART_Cmd(USART1, ENABLE);
	}
	else if(PORT == USART_ZIG)
	{
		// Save the baudrate in the appropriate variable
		Baudrate_ZIG = baudrate;

		// Initialise the UART5 peripheral
		USART_DeInit(UART5);
		USART_Init(UART5, &USART_InitStructure);

		// Enable the UART5 RX interrupt
#if ALLOW_ZIGBEE
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
#else
		USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);
#endif

		// Disable all other UART5 interrupts
		USART_ITConfig(UART5, USART_IT_TXE, DISABLE);  // Resets TXEIE
		USART_ITConfig(UART5, USART_IT_TC, DISABLE);   // Resets TCIE
		USART_ITConfig(UART5, USART_IT_IDLE, DISABLE); // Resets IDLEIE
		USART_ITConfig(UART5, USART_IT_PE, DISABLE);   // Resets PEIE
		USART_ITConfig(UART5, USART_IT_LBD, DISABLE);  // Resets LBDIE
		USART_ITConfig(UART5, USART_IT_ERR, DISABLE);  // Resets EIE (noise error, overrun error, frame error)

		// Enable the UART5 peripheral
#if ALLOW_ZIGBEE
		USART_Cmd(UART5, ENABLE);
#endif
	}
	else if(PORT == USART_PC)
	{
		// Save the baudrate in the appropriate variable
		Baudrate_PC = baudrate;

		// Initialise the USART3 peripheral
		USART_DeInit(USART3);
		USART_Init(USART3, &USART_InitStructure);

		// Configure the USART3 interrupts we need
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);  // Sets RXNEIE (enables RXNE, ORE interrupts)
		USART_ITConfig(USART3, USART_IT_TC, DISABLE);   // Resets TCIE (disables TC interrupt, this interrupt is enabled dynamically when it is needed)

		// Disable all other USART3 interrupts
		USART_ITConfig(USART3, USART_IT_TXE, DISABLE);  // Resets TXEIE
		USART_ITConfig(USART3, USART_IT_CTS, DISABLE);  // Resets CTSIE
		USART_ITConfig(USART3, USART_IT_IDLE, DISABLE); // Resets IDLEIE
		USART_ITConfig(USART3, USART_IT_PE, DISABLE);   // Resets PEIE
		USART_ITConfig(USART3, USART_IT_LBD, DISABLE);  // Resets LBDIE
		USART_ITConfig(USART3, USART_IT_ERR, DISABLE);  // Resets EIE (noise error, overrun error, frame error)

		// Enable the USART3 peripheral
		USART_Cmd(USART3, ENABLE);
	}
}

// Retrieve the baudrate of the given USART port
u32 USART_GetBaudrate(u8 PORT)
{
	// Return the required baudrate
	if(PORT == USART_DXL)
		return Baudrate_DXL;
	else if(PORT == USART_ZIG)
		return Baudrate_ZIG;
	else if(PORT == USART_PC)
		return Baudrate_PC;

	// Unknown port => Return zero
	return 0;
}

// Configure the Analog-to-Digital converters
void ADC_Configuration(void)
{
	// Declare and initialise variables
	ADC_InitTypeDef ADC_InitStructure;
	ADC_StructInit(&ADC_InitStructure);

	// Populate the ADC initialisation structure
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;

	// Initialise the ADC peripherals
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Init(ADC2, &ADC_InitStructure);

	// Initialise the ISR function for the ADC (called from the TIM2 interrupt)
	initADCISR();

	// Disable the ADC interrupts
	ADC_ITConfig(ADC1, ADC_IT_EOC, DISABLE);
	ADC_ITConfig(ADC2, ADC_IT_EOC, DISABLE);

	// Enable the ADC peripherals
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);

	// Reset the ADC calibration registers
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_ResetCalibration(ADC2);
	while(ADC_GetResetCalibrationStatus(ADC2));

	// Calibrate the ADC peripherals
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC2);
	while(ADC_GetCalibrationStatus(ADC2));

	// Start the continuous ADC conversions
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	ADC_SoftwareStartConvCmd(ADC2, ENABLE);
}

// Configure the SPI peripherals
void SPI_Configuration(void)
{
	// Declare and initialise variables
	SPI_InitTypeDef SPI_InitStructure;
	SPI_StructInit(&SPI_InitStructure);

	// Populate the SPI initialisation structure
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	// Initialise the SPI2 peripheral
	SPI_Init(SPI2, &SPI_InitStructure);

	// Enable the SPI2 peripheral
	SPI_Cmd(SPI2, ENABLE);
}

// Configure the various GPIO ports
void GPIO_Configuration(void)
{
	// Declare and initialise variables
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	//
	// Port A
	//

	// LED6 pins
	GPIO_InitStructure.GPIO_Pin = PIN_LED6_R | PIN_LED6_G | PIN_LED6_B;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// USB sleep pin (for detecting USB connection state)
	GPIO_InitStructure.GPIO_Pin = PIN_USB_SLEEP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// PC USART RX pin
	GPIO_InitStructure.GPIO_Pin = PIN_CPU_RXD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// PC USART TX pin
	GPIO_InitStructure.GPIO_Pin = PIN_CPU_TXD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// ADC pins
	GPIO_InitStructure.GPIO_Pin = PIN_ADC4 | PIN_ADC5 | PIN_ADC6 | PIN_ADC7 | PIN_ADC8 | PIN_ADC9 | PIN_ADC10 | PIN_ADC11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Button (SW) pins
	GPIO_InitStructure.GPIO_Pin = PIN_SW_MODE | PIN_SW_START;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//
	// Port B
	//

	// Output pins
	GPIO_InitStructure.GPIO_Pin = PIN_ENABLE_TX | PIN_ENABLE_RX | PIN_ENABLE_DXLPWR | PIN_BOOT1 | PIN_LED3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Enable Zigbee pin
#if !DEBUG_USE_JTAG
	GPIO_InitStructure.GPIO_Pin = PIN_ENABLE_ZIGBEE;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

	// USART RX pins
	GPIO_InitStructure.GPIO_Pin = PIN_DXL_RXD | PIN_PC_RXD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Communications pins
	GPIO_InitStructure.GPIO_Pin = PIN_DXL_TXD | PIN_PC_TXD | PIN_SIG_SCK | PIN_SIG_MOSI | PIN_SIG_MISO | PIN_SIG_BUZZER;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// ADC pins
	GPIO_InitStructure.GPIO_Pin = PIN_ADC14 | PIN_ADC15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//
	// Port C
	//

	// Output pins
	GPIO_InitStructure.GPIO_Pin = PIN_LED4 | PIN_LED5_R | PIN_LED5_G | PIN_LED5_B | PIN_SIG_ACC_CS | PIN_SIG_GYRO_CS | PIN_LED_TX | PIN_LED_RX | PIN_LED2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Zigbee TX pin
	GPIO_InitStructure.GPIO_Pin = PIN_ZIGBEE_TXD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// ADC pins
	GPIO_InitStructure.GPIO_Pin = PIN_ADC0 | PIN_ADC1 | PIN_ADC2 | PIN_ADC3 | PIN_ADC12 | PIN_ADC13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//
	// Port D
	//

	// Zigbee RX pin
	GPIO_InitStructure.GPIO_Pin = PIN_ZIGBEE_RXD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// Configure USART1 pin remapping
	GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

	// Configure JTAG pin remapping
#if DEBUG_USE_JTAG
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE); // Disable the NJTRST remap of the TXD_ENABLE pin ('PIN_ENABLE_TX') for the case that JTAG is enabled
#else
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
#endif
}

// Configure the Nested Vectored Interrupt Controller (NVIC)
void NVIC_Configuration(void)
{
	// Declare variables
	NVIC_InitTypeDef NVIC_InitStructure;  

	// Set the vector table base location
#ifdef VECT_TAB_RAM
	NVIC_SetVectorTable(NVIC_VectTab_RAM  , 0x0000); // Located at 0x20000000
#else // VECT_TAB_FLASH
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000); // Located at 0x08003000
#endif

	// Configure the NVIC priority grouping (priority values are then of the format XX.XX[0000])
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // => Group Priority 0-3 (2 bits), SubPriority 0-3 (2 bits), a lower number corresponds to a higher priority (0 is the highest priority), only the Group Priority (i.e. not the SubPriority) determines whether an interrupt interrupts another, SubPriority then determines order of processing if multiple interrupts are pending

	//
	// Interrupt Priority 0
	//

	// Configure the USART3 interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel; // Priority 00.00[0000]
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Configure the USART1 interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel; // Priority 00.01[0000]
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

#if ALLOW_ZIGBEE
	// Configure the UART5 interrupt
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQChannel;  // Priority 00.10[0000]
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

	//
	// Interrupt Priority 1
	//

	// Configure the DXL Rx handler interrupt
	NVIC_InitStructure.NVIC_IRQChannel = IRQ_DXL_RXHANDLER; // Priority 01.00[0000]
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Configure the PC Rx handler interrupt
	NVIC_InitStructure.NVIC_IRQChannel = IRQ_PC_RXHANDLER;  // Priority 01.01[0000]
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Configure the DXL Tx handler interrupt
	NVIC_InitStructure.NVIC_IRQChannel = IRQ_DXL_TXHANDLER; // Priority 01.10[0000]
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Configure the PC Tx handler interrupt
	NVIC_InitStructure.NVIC_IRQChannel = IRQ_PC_TXHANDLER;  // Priority 01.11[0000]
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//
	// Interrupt Priority 2
	//

	// Configure the TIM2 interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;   // Priority 10.00[0000]
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
