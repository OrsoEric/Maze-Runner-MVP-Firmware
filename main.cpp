/****************************************************************************
**	ORANGEBOT PROJECT
*****************************************************************************
**	MODULE TESTING
*****************************************************************************
**	Author: 			Orso Eric
**	Creation Date:
**	Last Edit Date:
**	Revision:			1
**	Version:			0.1 ALPHA
****************************************************************************/

/****************************************************************************
**	HYSTORY VERSION
*****************************************************************************
**	R1 V0.1ALPHA
**		>
****************************************************************************/

/****************************************************************************
**	DESCRIPTION
*****************************************************************************
**	This project is meant to test the AT324 Module
**	>Test LCD display
**	>Test USART0 (57Kbs) with loopback
****************************************************************************/

/****************************************************************************
**	USED PIN
**	TIPS: compile this field before writing any code: it really helps!
*****************************************************************************
**
****************************************************************************/

/****************************************************************************
**	USED PHERIPERALS
**	TIPS: compile this field before writing any code: it really helps!
*****************************************************************************
**	Continuous rotation servo
**	1.3mS - 1.7mS with ded time 1.5mS
**
**	
****************************************************************************/

/****************************************************************************
**	KNOWN BUG
*****************************************************************************
**	>
****************************************************************************/

/****************************************************************************
**	TODO
*****************************************************************************
**
****************************************************************************/

/****************************************************************************
**	ENVROIMENT VARIABILE
****************************************************************************/

/****************************************************************************
**	INCLUDE
**	TIPS: ".h" should not include other ".h", it lower the leggibility of the code
**	TIPS: ".h" must not contain anything that generate code, write only declaration or prototype
**	this help the leggibility and the debug phase
**	TIPS: type from the stdint.h have a well defined width and signedness, use them
**	( uint8_t = unsigned 8 bit, int8_t = signed 8 bit, uint32_t = unsiged 32 bit, ecc... )
****************************************************************************/

#include "global.h"

/****************************************************************************
**	DEFINE
****************************************************************************/

//define for the mail loop
#define EVER (;;)

/****************************************************************************
**	MACRO
****************************************************************************/

/****************************************************************************
**	STRUCTURE
****************************************************************************/

/****************************************************************************
**	PROTOTYPE: FUNCTION
**	TIPS: use "extern" in function prototype, it's not necessary, but any other
**	prototype need it, it help the legibility of the code
****************************************************************************/

/****************************************************************************
**	PROTOTYPE: GLOBAL VARIABILE
****************************************************************************/

/****************************************************************************
**	GLOBAL VARIABILE:
**	TIPS: "const" variable will be loaded in the flash memory, saving the ram,
**	use it for string that will not be modified
**	TIPS: if you want a ISR to manipulate a global variable, than that variable
**	**must** be declared "volatile" so that the c compiler will not wipe out the
**	variable by optimizing the code, use that variable as less as possible
**	because it will not be optimized
**	TIPS: "volatile int" variable may give problem, don't use it (uP is 8 bits, while
**	int is 16 bits, it's implemented by concatenating 2 byte, the volatile statement
**	disable the optimization on that variable, and mess up that code)
****************************************************************************/

	//-----------------------------------------------------------------------
	//	FLAGS
	//-----------------------------------------------------------------------

//Volatile flags used by ISRs
volatile Isr_flags flags;
//Status variable for the servos, keep track of which servo to do next
volatile U8 servo_cnt	= N_SERVOS;

	//-----------------------------------------------------------------------
	//	SERVOS VARS
	//-----------------------------------------------------------------------

//One flag per servo. '1' if the servo is keeping up with target_pos
U8 servo_lock			= 0x00;
//Current position of the servos. Used by the driver, user should not write here
U16 servo_delay[ N_SERVOS ];
//Target position. The user write here, the driver will do it's best to reach it
S8 servo_target_pos[ N_SERVOS ];
//The servo will rotate at this speed [unit/second]
U8 servo_target_speed[ N_SERVOS ];
//servo position offset for true 0 position
//offsets are accounted for in separately from the position, it does not eat into the dynamic
S8 servo_off[ N_SERVOS ];

	//-----------------------------------------------------------------------
	//	TRAJECTORY VARS
	//-----------------------------------------------------------------------

//Timebase for trajectory generation. 1 time unit = 1/50Hz.
U16 servo_global_time 	= 0;
//Current motion plan
//Trajectories trajectory = MOVE_IDLE;
	///----------------------------------------------------------------------
	///	BUFFERS
	///----------------------------------------------------------------------
	//	Buffers structure and data vectors

//Safe circular buffer for UART input data
volatile At_buf8_safe uart_rx_buf;
//Safe circular buffer for uart tx data
At_buf8 uart_tx_buf;
//allocate the working vector for the buffer
U8 v0[ UART_RX_BUF_SIZE ];
//allocate the working vector for the buffer
U8 v1[ UART_TX_BUF_SIZE ];

	///--------------------------------------------------------------------------
	///	PARSER COMMANDS
	///--------------------------------------------------------------------------

//Command dictionary. Command IDs 0 and 255 are forbidden
U8 uart_cmd[] =
{
	//Ping: No action. Effect is to reset the connection timeout
	UART_CMD_PING		, 'P', '\0',
	//Sign: Ask for board signature
	UART_CMD_SIGN		, 'F', '\0',									
	//Set Speed: R right engine L left engine 
	UART_CMD_SETVEL	, 'V', 'R', '%', 'd', 'L', '%', 'd', '\0', 		
	//Dictionary terminator
	'\0'
};
//Board Signature
U8 *board_sign = (U8 *)"MazeRunner_05032";
//communication timeout counter
U8 uart_timeout_cnt = 0;

bool f_timeout_detected = false;

/****************************************************************************
**	MAIN
****************************************************************************/

int main( void )
{
	///----------------------------------------------------------------------
	///	LOCAL VARS
	///----------------------------------------------------------------------
		///TEMP vars
	U8 u8t, t;
	S8 s8t;
	U16 u16t;
	//Index to the LCD display (serial terminal)
	//U8 lcd_index;
	//prescaler
	U8 pre;

		///Parser
	//This structure hold a link to the command dictionary, the parser status variables and a link to the partial packet structure
	Parser *parser = (Parser *)NULL;
	//parser exe return a Parser_packet structure if a command is successfully decoded. User must manually destroy it when done
	Parser_packet *packet = (Parser_packet *)NULL;

    ///----------------------------------------------------------------------
	///	VARS INIT
	///----------------------------------------------------------------------

		///UART RX BUFFER INIT
		//I init the rx and tx buffers
	//attach vector to buffer
	AT_BUF_ATTACH( uart_rx_buf, v0, UART_RX_BUF_SIZE);
	//attach vector to buffer
	AT_BUF_ATTACH( uart_tx_buf, v1, UART_TX_BUF_SIZE);

	//lcd_index 	= 0;		//Index to the LCD display (serial terminal)
	pre 		= 0;		//counter prescaler

	//Clear global servo time
	//Initialize servo position to zero (offset is accounted for during calculations, i must not add it here)
	for (u8t = 0;u8t < N_SERVOS;u8t++)
	{
		servo_delay[u8t] 		= K0;	//Servo true position
		servo_target_pos[u8t] 	= +0;	//Servo target position (user)
		servo_target_speed[u8t]	= SERVO_DEFAULT_SPEED;	//Servo target speed (default)
	}

	//servo_target_speed[ SERVO_WHEEL_RIGHT ] = 


	///----------------------------------------------------------------------
	///	DEVS INIT
	///----------------------------------------------------------------------

	//Init all pins, init all devices
	global_init();

	lcd_print_str( LCD_POS( 0, 0), (U8 *)"Maze Runner");

	///Parser Init
	//Create a new parser. uart_cmd is the dictionary for this parser (i can have multiple indipendent parsers)
	parser = born_parser( uart_cmd );
	//If: parser creation failed
	if (parser == NULL)
	{
		//Signal Error
		AT_BUF_PUSH( uart_tx_buf, 'E' );
	}

	///----------------------------------------------------------------------
	///	MAIN LOOP
	///----------------------------------------------------------------------

	//Main Loop
	for EVER
	{
		///----------------------------------------------------------------------
		/// LCD Display Driver
		///----------------------------------------------------------------------
		//	Sync the display content with the user structure

		//If: LCD update flag
		if (flags.lcd_update == 1)
		{
			//Clear flag
			flags.lcd_update = 0;
			//Driver that sync the user structure with the LCD.
			//This paradigm solve lots of timing problems of the direct call version.
			//You can print a million time a second, and the driver still won't bug out
			lcd_update();
		}	//End If: LCD update flag

		//-----------------------------------------------------------------------
		//	START MOTOR SCAN (5.8uS max all servo functions)
		//-----------------------------------------------------------------------
		//	Flag rised by [Timer 0]
		//	>activity pin (led signal uC use, oscilloscope allow to measure function times)
		//	>clear servo status var
		//	>calculate first delay, pull down first line
		//	>setup first delay, enable [Timer 1]
		//	>Timer 1 ISR will handle the update of the servos and disable it self when done

		//If: Start Servo Scan (50Hz)
		if (flags.servo_scan == 1)
		{
			//clear flag
			flags.servo_scan = 0;

			///----------------------------------------------------------------------
			///	Generate System Ticks
			///----------------------------------------------------------------------
			//	Waste of resources to do it in ISR
			
			//If prescaler reset
			if (pre == 0)
			{
				//Generate slow tick
				flags.tick_slow = 1;
			}
			//Divide frequency by 50
			pre = AT_TOP_INC( pre, 50 );

			///----------------------------------------------------------------------
			///	Communication Timeout
			///----------------------------------------------------------------------

			if (uart_timeout_cnt < UART_TIMEOUT)
			{
				uart_timeout_cnt++;
				f_timeout_detected = false;
			}
			else
			{
				//do nothing. I'm already in timeout
			}
			
			//If in timeout
			if ((f_timeout_detected == false) && (uart_timeout_cnt >= UART_TIMEOUT))
			{
				//Just once
				f_timeout_detected = true;
				//Initialize servo position to zero (offset is accounted for during calculations, i must not add it here)
				for (u8t = 0;u8t < N_SERVOS;u8t++)
				{
					servo_target_pos[u8t] 	= +0;	//Servo target position (user)
				}
				//Clear row
				lcd_print_str(LCD_POS(1, 0),(U8 *)"                ");
			}

			///----------------------------------------------------------------------
			///	Generate PPM Signals
			///----------------------------------------------------------------------

			//Startup the servo scan
			//clear status var
			servo_cnt 	= 0;
			//calculate delay
			u16t		= servo_calc_delay( 0 );
			//Store delay on T1
			OCR1A 		= u16t;
			//start T1
			START_TIMER1();
			//pull up first line
			SET_BIT( SERVO_PORT, 0 +SERVO_PIN_OFFSET );
			//Advance global time by one tick
			servo_global_time++;

			lcd_print_u16( LCD_POS(0, 11), servo_global_time );

			//pre_traj = AT_TOP_INC( pre_traj, MOVE_STEP_TIME );
		}	//End If: motor scan flag

		///----------------------------------------------------------------------
		///	System Ticks
		///----------------------------------------------------------------------

		//If: Slow Tick
		if (flags.tick_slow == 1)
		{
			//Clear flag
			flags.tick_slow = 0;
			
			//DEBUG: send a data trough the UART
			AT_BUF_PUSH( uart_tx_buf, 'Z' );
			//Toggle leds
			TOGGLE_BIT( PORTB, PB6 );
			TOGGLE_BIT( PORTB, PB7 );
			//Move motor
			/*
			if (servo_target_pos[ 0 ]>0)
			{
				servo_target_pos[ 0 ] = -5;
				servo_target_pos[ 1 ] = -5;
				servo_target_pos[ 2 ] = -20;
				servo_target_pos[ 3 ] = -20;
			}
			else
			{
				servo_target_pos[ 0 ] = 5;
				servo_target_pos[ 1 ] = 5;
				servo_target_pos[ 2 ] = 20;
				servo_target_pos[ 3 ] = 20;
			}
			*/
			
		}	//Endif: slow tick

		///----------------------------------------------------------------------
		/// UART RX
		///----------------------------------------------------------------------
		//	Handle RX from RS232
		//	Loopback

		//if: uart rx buffer is not empty
		if ( AT_BUF_NUMELEM( uart_rx_buf ) > 0)
		{
				///Get data
			//Get the byte from the RX buffer (ISR put it there)
			u8t = AT_BUF_PEEK( uart_rx_buf );
			AT_BUF_KICK_SAFER( uart_rx_buf );
				///Loopback
			//Push into tx buffer
			AT_BUF_PUSH( uart_tx_buf, u8t );
			
			///Command parser
			//feed the input RX byte to the parser
			packet = parser_exe( parser, u8t );
			//if: the parser_exe is doing t's stuffs
			if (packet == NULL)
			{
				//do nothing
			}
			//if: parser exe has fully decoded a command
			else
			{
				//I decoded a valid packet. reset the timeout counter
				uart_timeout_cnt = 0;
					///UART_CMD_PING
				//if: ping. Only used to reset the com timeout counter
				if (packet -> id == UART_CMD_PING)
				{
					lcd_print_char( LCD_POS(1,14), 'P' );
					//do nothing
				}
					///UART_CMD_SIGN
				//If: Master is asking for signature
				else if (packet -> id == UART_CMD_SIGN)
				{
					//If: I have a valid board signature string
					if (board_sign != NULL)
					{
						t = 0;
						while ((t < MAX_SIGN_LEN) && (board_sign[t] != '\0'))
						{
							u8t = board_sign[t];
							t++;
							AT_BUF_PUSH( uart_tx_buf, u8t );
						}
						AT_BUF_PUSH( uart_tx_buf, '\0' );
					}	//End If: I have a valid board signature string
				}	//End If: Master is asking for signature
					///UART_CMD_SETVEL
				else if (packet -> id == UART_CMD_SETVEL)
				{
					//fetch x. Stored in arg 0 of packet
					s8t = packet_get_s8( packet, 0 );
					lcd_print_s8( LCD_POS(1,0), s8t );
					servo_target_pos[ SERVO_WHEEL_RIGHT ] = s8t;
					//fetch y. Stored in arg 1 of packet
					s8t = packet_get_s8( packet, 1 );
					lcd_print_s8( LCD_POS(1,8), s8t );
					servo_target_pos[ SERVO_WHEEL_LEFT ] = s8t;
				}
				//if: parser decoded a command that i am not handling
				else
				{
					AT_BUF_PUSH( uart_tx_buf, '?' );
					AT_BUF_PUSH( uart_tx_buf, '\0' );
				}
				//Manually dispose of the packet structure
				bury_packet( packet );
				packet = (Parser_packet *)NULL;
			} //End parser

		}	//end if: uart rx buffer is not empty

		///----------------------------------------------------------------------
		/// UART TX
		///----------------------------------------------------------------------

		//if: the Uart0 HW buffer is empty and the UART tx buffer is not empty
		if ( (UART0_TX_READY()) && (AT_BUF_NUMELEM( uart_tx_buf ) > 0) )
		{
			//Get the byte to be filtered out
			u8t = AT_BUF_PEEK( uart_tx_buf );
			AT_BUF_KICK( uart_tx_buf );
			//Write on UART tx buffer.
			UDR0 = u8t;

			//lcd_print_char( LCD_POS( 1, 14), u8t );
		}	//End If: uart tx
	}	//end for: for EVER

	return 0;
}	//end main

/****************************************************************************
** FUNCTIONS:
****************************************************************************/

/****************************************************************************
**	SERVO CALC POS
*****************************************************************************
**	This function will calculate the delay that move servo[index] closer to the target position
**		SLEW RATE LIMITER: The function will only move at the speed set by the user
**		OFFSET CORRECTION:	In this function i account for mechanical misalignment
**	Formula:
**	xpos = xtarget if ABS(xtarget -xpos) < xspeed/50
**	delay[OCR] = K0 + K1*xoff + K1*xpos
****************************************************************************/

U16 servo_calc_delay( U8 index )
{
	///--------------------------------------------------------------------------
	///	STATIC VARIABILE
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	LOCAL VARIABILE
	///--------------------------------------------------------------------------

	U16 delay;

	S16 slew_rate;

	S16 s16t;


	U16 ret;

	///--------------------------------------------------------------------------
	///	CHECK AND INITIALIZATIONS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------
	//	>Calculate target delay
	//	>Calculate maximum delay change target_delay-old delay (slew rate)
	//		>based on user defined target_speed
	//		>limited by servo max slew rate (max rotation speed normalized to my units)
	//	>calculate new delay (apply delay change)
	//	>save delay on servo_delay
	//	>return new delay
	///Calculate target OCR of servo [index]
	//Calculate position dependent coefficient
	s16t 	= K1 *servo_off[ index ] +K1 *servo_target_pos[ index ];
	//apply sign correction
	if (IS_BIT_ONE( SERVO_DIR, index ))
	{
		s16t = -s16t;
	}
	delay	= K0 + s16t;
	///Speed Limiter
	//calculate maximum allowed motion. I need to convert from [unit/second] -> [unit/20mS]
	slew_rate = K1*servo_target_speed[ index ] /50;
	//If: The user wants to exceed the servo limits
	if (slew_rate > SERVO_MAX_SLEW_RATE)
	{
		//Clip the Slew rate
		slew_rate = SERVO_MAX_SLEW_RATE;
	}
	//calculate slew rate required to meet user input
	s16t = delay - servo_delay[ index ];
	if (s16t > +slew_rate)
	{
		//I'm NOT locked: the servo is moving at max speed
		CLEAR_BIT( servo_lock, index );
		ret = servo_delay[ index ] +slew_rate;
	}
	else if (s16t < -slew_rate)
	{
		//I'm NOT locked: the servo is moving at max speed
		CLEAR_BIT( servo_lock, index );
		ret = servo_delay[ index ] -slew_rate;
	}
	else
	{
		//I'm locked: The motor is not moving at max speed
		SET_BIT( servo_lock, index );
		ret = delay;
	}
	//Write back result and return the delay
	servo_delay[ index ] = ret;

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	return ret;
}	//end function: servo_calc_delay

