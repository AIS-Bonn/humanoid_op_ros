/******************************************************************************
* File Name          : compass.c
* Author             : Max Schwarz <max.schwarz@uni-bonn.de>
*                    : Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
* Version            : V0.1
* Date               : 02/04/14
* Description        : HMC5883L 3-axis compass driver
*******************************************************************************/

// Includes
#include "compass.h"
#include <stm32f10x_gpio.h>
#include <system_init.h>
#include <CM_DXL_COM.h>

// I2C Bit Banging:
// As we cannot electrically access the dedicated I2C pins of the microcontroller
// on the CM730 board, we need to simulate our own I2C communications using
// standard GPIO pins.
//
// Connections:
// Two GPIO pins are used, SDA and SCL. Refer to the definitions of PIN_COMPASS_SDA
// and PIN_COMPASS_SCL in system_init.h.
// 
// General idea:
// Everything is implemented in finite state machines, as we have to work
// in very small steps without blocking. We have a low-level I2C state machine
// and a high level "script" machine, which contains compass initialisation
// and data acquisition.

// Low level FSM states
enum I2CState
{
	I2C_IDLE,           // [STATE] SCL = SDA = 1 (idle bus)
	I2C_READY,          // [STATE] SCL = SDA = 0 (bus ready for activity)
	I2C_START,          // [REQUEST] Start: I2C_IDLE -> I2C_READY
	I2C_START_2,
	I2C_START_3,
	I2C_WRITE,          // [REQUEST] Write byte: I2C_WRITE -> I2C_WRITE_GOT_[N]ACK (uses g_i2c_to_write = data, g_i2c_bit = 7)
	I2C_WRITE_2,
	I2C_WRITE_3,
	I2C_WRITE_ACK_1,
	I2C_WRITE_ACK_2,
	I2C_WRITE_ACK_3,
	I2C_WRITE_GOT_ACK,  // [STATE] Got ACK after write
	I2C_WRITE_GOT_NACK, // [STATE] Got NACK after write
	I2C_READ,           // [REQUEST] Read byte: I2C_READ -> I2C_READ_ENDED (uses g_i2c_read = 0, g_i2c_bit = 7)
	I2C_READ_2,
	I2C_READ_3,
	I2C_READ_4,
	I2C_READ_ACK_1,
	I2C_READ_ACK_2,
	I2C_READ_ENDED,     // [STATE] Finished reading a byte
	I2C_STOP,           // [REQUEST] Stop: I2C_READY -> I2C_IDLE
	I2C_STOP_2,
	I2C_STOP_3
};

// High level FSM command codes
enum CommandType
{
	CMD_START,         // Send a start condition
	CMD_WRITE,         // Write a byte <data>
	CMD_READ,          // Read a byte into read_data[<data>]
	CMD_STOP,          // Send a stop condition
	CMD_WAIT,          // Wait for <data> steps
	CMD_LOOP           // Jump to command number <data> (zero-indexed)
};

// High level FSM command struct
struct Command
{
	enum CommandType type;
	u8 data;
};

// Command script to configure and read the HMC5883L compass values
const struct Command g_script[] = {
	{CMD_START, 0},
	{CMD_WRITE, 0x3C}, // Device I2C write address
	{CMD_WRITE, 0x00}, // Config Register A
	{CMD_WRITE, 0x38}, // <7> = 0 / <6-5> = 01 = Average 2 samples / <4-2> = 110 = 75Hz measurement rate / <1-0> = 00 = Normal measurement configuration
	{CMD_STOP , 0},

	{CMD_START, 0},
	{CMD_WRITE, 0x3C}, // Device I2C write address
	{CMD_WRITE, 0x01}, // Config Register B
	{CMD_WRITE, 0x40}, // <7-5> = 010 = 820 LSb/gauss = +-2.50 gauss full scale range, +-1.90 gauss recommended measurement range / <4-0> = 00000
	{CMD_STOP , 0},

	{CMD_START, 0},
	{CMD_WRITE, 0x3C}, // Device I2C write address
	{CMD_WRITE, 0x02}, // Mode register
	{CMD_WRITE, 0x00}, // <7-2> = 000000 / <1-0> = 00 = Continuous measurement mode
	{CMD_STOP , 0},

	{CMD_WAIT , 100},  // Wait for a number of cycles (results in a loop rate of approximately 41Hz)

	{CMD_START, 0},
	{CMD_WRITE, 0x3C}, // Device I2C write address
	{CMD_WRITE, 0x03}, // Select the address of the first output register to read
	{CMD_STOP , 0},

	{CMD_START, 0},
	{CMD_WRITE, 0x3D}, // Device I2C read address
	{CMD_READ , 0},    // Read X MSB
	{CMD_READ , 1},    // Read X LSB
	{CMD_READ , 2},    // Read Z MSB
	{CMD_READ , 3},    // Read Z LSB
	{CMD_READ , 4},    // Read Y MSB
	{CMD_READ , 5},    // Read Y LSB
	{CMD_STOP , 0},

	{CMD_LOOP , 0}     // Jump back to writing all of the config registers again (unexplainably irrevocably necessary!) and update the control table with the last read data
};

// Global variables
volatile enum I2CState g_i2c_state = I2C_IDLE;
vu8 g_i2c_to_write = 0;
vu8 g_i2c_bit = 7;
vu8 g_i2c_read = 0;
vu8 current_entry = 0;
vu8 wait_counter = 0;
vu8 read_data[6] = {0};
vu8 g_i2c_sda_cmd = 1;
vu8 g_i2c_scl_cmd = 1;

// Functions
static void i2c_init(void);
static void i2c_step(void);
static inline void i2c_start(void);
static inline void i2c_stop(void);
static inline void i2c_write(u8 value);
static inline void i2c_read(void);
static inline void i2c_scl(u8 on);
static inline void i2c_sda(u8 on);
static inline u8   i2c_sda_read(void);
static inline void ProcessData();

//
// Configuration
//

// Configuration of the compass
void ConfigureCompass(void)
{
	// Initialise the I2C communications
	i2c_init();
}

// Initialise the I2C communications
static void i2c_init(void)
{
	// Declare variables
	GPIO_InitTypeDef init;

	// SDA pin configuration
	init.GPIO_Pin = PIN_COMPASS_SDA;
	init.GPIO_Mode = GPIO_Mode_Out_OD;
	init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_COMPASS_SDA, &init);
	
	// SCL pin configuration
	init.GPIO_Pin = PIN_COMPASS_SCL;
	init.GPIO_Mode = GPIO_Mode_Out_OD;
	init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_COMPASS_SCL, &init);

	// Initialise the states of the SDA and SCL pins
	i2c_sda(1);
	i2c_scl(1);
}

//
// Low level FSM
//

// Perform one step of the I2C FSM
static void i2c_step(void)
{
	switch(g_i2c_state)
	{
		// Idle states
		case I2C_IDLE: // Idle bus with SCL and SDA high
			break;
		case I2C_READY: // Bus ready for activity with SCL and SDA low
			break;

		// Start states
		case I2C_START:
			i2c_init(); // Sets SDA and SCL to 1
			g_i2c_state = I2C_START_2;
			break;
		case I2C_START_2:
			i2c_sda(0); // SDA falling while SCL 1 => Start bit
			g_i2c_state = I2C_START_3;
			break;
		case I2C_START_3:
			i2c_scl(0); // Now SDA and SCL are both 0, ready for bus activity
			g_i2c_state = I2C_READY;
			break;
		
		// Write states
		case I2C_WRITE:
			i2c_sda(g_i2c_to_write & (1 << g_i2c_bit)); // SDA reflects data bit
			g_i2c_state = I2C_WRITE_2;
			break;
		case I2C_WRITE_2:
			i2c_scl(1); // Toggle SCL high
			g_i2c_state = I2C_WRITE_3;
			break;
		case I2C_WRITE_3:
			i2c_scl(0); // Toggle SCL low
			if(g_i2c_bit == 0)
				g_i2c_state = I2C_WRITE_ACK_1;
			else
			{
				g_i2c_bit--;
				g_i2c_state = I2C_WRITE;
			}
			break;
		case I2C_WRITE_ACK_1:
			i2c_sda(1); // Release SDA to allow the slave to ACK or NACK
			g_i2c_state = I2C_WRITE_ACK_2;
			break;
		case I2C_WRITE_ACK_2:
			i2c_scl(1); // Toggle SCL high
			g_i2c_state = I2C_WRITE_ACK_3;
			break;
		case I2C_WRITE_ACK_3:
			if(i2c_sda_read()) // Read the acknowledgement bit
				g_i2c_state = I2C_WRITE_GOT_NACK;
			else
				g_i2c_state = I2C_WRITE_GOT_ACK;
			i2c_scl(0); // Toggle SCL low
			break;
		case I2C_WRITE_GOT_ACK:
			break;
		case I2C_WRITE_GOT_NACK:
			break;
		
		// Read states
		case I2C_READ:
			i2c_sda(1); // Release SDA to allow the slave to transmit data
			g_i2c_state = I2C_READ_2;
			break;
		case I2C_READ_2:
			i2c_scl(1); // Toggle SCL high
			g_i2c_state = I2C_READ_3;
			break;
		case I2C_READ_3:
			if(i2c_sda_read())
				g_i2c_read |= (1 << g_i2c_bit);
			i2c_scl(0); // Toggle SCL low
			if(g_i2c_bit == 0)
				g_i2c_state = I2C_READ_4;
			else
			{
				g_i2c_bit--;
				g_i2c_state = I2C_READ_2;
			}
			break;
		case I2C_READ_4:
			i2c_sda(0); // Reassert control over SDA again and pull it low for an ACK
			g_i2c_state = I2C_READ_ACK_1;
			break;
		case I2C_READ_ACK_1:
			i2c_scl(1); // Toggle SCL high
			g_i2c_state = I2C_READ_ACK_2;
			break;
		case I2C_READ_ACK_2:
			i2c_scl(0); // Toggle SCL low
			g_i2c_state = I2C_READ_ENDED;
			break;
		case I2C_READ_ENDED:
			break;
		
		// Stop states
		case I2C_STOP:
			i2c_sda(0);
			g_i2c_state = I2C_STOP_2;
			break;
		case I2C_STOP_2:
			i2c_scl(1);
			g_i2c_state = I2C_STOP_3;
			break;
		case I2C_STOP_3:
			i2c_sda(1);
			g_i2c_state = I2C_IDLE;
			break;
		
		// Default state
		default:
			break;
	}
}

// Start the I2C
static inline void i2c_start(void)
{
	g_i2c_state = I2C_START;
}

// Stop the I2C
static inline void i2c_stop(void)
{
	g_i2c_state = I2C_STOP;
}

// Initiate the write of a byte over I2C
static inline void i2c_write(u8 value)
{
	g_i2c_to_write = value;
	g_i2c_bit = 7;
	g_i2c_state = I2C_WRITE;
}

// Initiate the read of a byte over I2C
static inline void i2c_read(void)
{
	g_i2c_read = 0;
	g_i2c_bit = 7;
	g_i2c_state = I2C_READ;
}

// Set the state of the SCL pin
static inline void i2c_scl(u8 on)
{
	if(on) GPIO_SetBits(PORT_COMPASS_SCL, PIN_COMPASS_SCL);
	else GPIO_ResetBits(PORT_COMPASS_SCL, PIN_COMPASS_SCL);
	g_i2c_scl_cmd = (on ? 1 : 0);
}

// Set the state of the SDA pin
static inline void i2c_sda(u8 on)
{
	if(on) GPIO_SetBits(PORT_COMPASS_SDA, PIN_COMPASS_SDA);
	else GPIO_ResetBits(PORT_COMPASS_SDA, PIN_COMPASS_SDA);
	g_i2c_sda_cmd = (on ? 1 : 0);
}

// Read the state of the SDA pin
static inline u8 i2c_sda_read(void)
{
	return (GPIO_ReadInputDataBit(PORT_COMPASS_SDA, PIN_COMPASS_SDA) == Bit_SET);
}

// Read the state of the SCL pin
static inline u8 i2c_scl_read(void)
{
	return (GPIO_ReadInputDataBit(PORT_COMPASS_SCL, PIN_COMPASS_SCL) == Bit_SET);
}

//
// High level FSM
//

// Interrupt service routine to handle the compass communications
void __ISR_COMPASS_I2C(void) // Called at 24896Hz
{
	// I2C clock stretching
	if(i2c_scl_read() != g_i2c_scl_cmd)
		return;

	// Step the low level FSM
	i2c_step();

	// Retrieve the current high level FSM command
	const struct Command* cmd = &g_script[current_entry];

	// Perform the required action for the command
	switch(cmd->type)
	{
		// Signal a start condition
		case CMD_START:
			if(g_i2c_state == I2C_IDLE)
				i2c_start();
			else if(g_i2c_state == I2C_READY)
				current_entry++;
			break;
		
		// Write a byte of data
		case CMD_WRITE:
			switch(g_i2c_state)
			{
				case I2C_READY:
					i2c_write(cmd->data);
					break;
				case I2C_WRITE_GOT_ACK:
					g_i2c_state = I2C_READY;
					current_entry++;
					break;
				case I2C_WRITE_GOT_NACK:
					g_i2c_state = I2C_READY;
					current_entry++;
					break;
				default:
					break;
			}
			break;
		
		// Read a byte of data
		case CMD_READ:
			switch(g_i2c_state)
			{
				case I2C_READY:
					i2c_read();
					break;
				case I2C_READ_ENDED:
					read_data[cmd->data] = g_i2c_read;
					g_i2c_state = I2C_READY;
					current_entry++;
					break;
				default:
					break;
			}
			break;
		
		// Signal a stop condition
		case CMD_STOP:
			if(g_i2c_state == I2C_READY)
				i2c_stop();
			else if(g_i2c_state == I2C_IDLE)
				current_entry++;
			break;
		
		// Process the last read data and loop back to a particular command (zero-based index in g_script)
		case CMD_LOOP:
			ProcessData();
			current_entry = cmd->data;
			break;
		
		// Wait some time
		case CMD_WAIT:
			if(wait_counter == 0)
				wait_counter = cmd->data;
			else if(wait_counter == 1)
			{
				wait_counter = 0;
				current_entry++;
			}
			else
				wait_counter--;
			break;
		
		// Should never happen
		default:
			break;
	}
}

// Process the data bytes received from the compass
static inline void ProcessData()
{
	// Declare variables
	s16 RawMagX, RawMagY, RawMagZ;

	// Retrieve the last-received raw compass data
	// Note: The read data is X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB in that order.
	//       Each returned MSB/LSB pair is a 12-bit signed value (-2048->2047) returned in 16-bit 2's complement form.
	//       If there is an internal error or overflow in the chip the returned value for the affected MSB/LSB pair is -4096.
	RawMagX = ((s16) read_data[0] << 8) | read_data[1];
	RawMagY = ((s16) read_data[4] << 8) | read_data[5];
	RawMagZ = ((s16) read_data[2] << 8) | read_data[3];

	// Write the compass data into the control table, checking for errors
	// Note: It is assumed that the compass is mounted so that its internal x, y and z axes
	//       line up with the robot's trunk coordinate frame.
	if((RawMagX != -4096) && (RawMagY != -4096) && (RawMagZ != -4096))
	{
		GW_MAG_X = RawMagX;
		GW_MAG_Y = RawMagY;
		GW_MAG_Z = RawMagZ;
	}
}
/******************************** END OF FILE *********************************/
