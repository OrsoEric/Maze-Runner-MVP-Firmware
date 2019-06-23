#ifndef GLOBAL_H
	//header environment variable, is used to detect multiple inclusion
	//of the same header, and can be used in the c file to detect the
	//included library
	#define GLOBAL_H

	/****************************************************************************
	**	ENVROIMENT VARIABILE
	****************************************************************************/

	#define F_CPU 20000000

	//Enable to test reading on the ADC module
	//#define TEST_ADC

	/****************************************************************************
	**	GLOBAL INCLUDE
	**	TIPS: you can put here the library common to all source file
	****************************************************************************/

	//type definition using the bit width and signedness
	#include <stdint.h>
	//define the ISR routune, ISR vector, and the sei() cli() function
	#include <avr/interrupt.h>
	//name all the register and bit
	#include <avr/io.h>
	//hard delay
	#include <util/delay.h>
	//General purpose macros
	#include "at_utils.h"
	//Library to control a 16x2 LCD display
	#include "at_lcd.h"
	//Basic string manipulation
	#include "at_string.h"
	//Parsing with arguments
	#include "at_parser.h"
	//AT Mega specific MACROS
	#include "at_mega_port.h"

	/****************************************************************************
	**	DEFINE
	****************************************************************************/

		///----------------------------------------------------------------------
		///	BUFFERS
		///----------------------------------------------------------------------

	#define UART_RX_BUF_SIZE	16
	#define UART_TX_BUF_SIZE	8

		///----------------------------------------------------------------------
		///	ADC
		///----------------------------------------------------------------------

		///----------------------------------------------------------------------
		///	SERVOS
		///----------------------------------------------------------------------
		
	//Total number of servos
	#define N_SERVOS			4
	//Special code to disable a servo
	#define SERVO_OFF			-128
	//Neutral OCR
	#define	K0					30000
	//Linear OCR (40°-400uS-63) (90°-900uS-141.7)
	#define K1					142
	//Port where the servos are placed
	#define SERVO_PORT			PORTC
	//offset at which servo pins starts
	#define SERVO_PIN_OFFSET	0
	//Servo maximum angular velocity [Angular Units/Ticks], at 5V i have 190mS per 60°, 180° are 256 Units, 1 tick is 20mS. Every Tick i can move ~ Units
	#define SERVO_MAX_SPEED		20
	//Convert to OCR K1*
	#define SERVO_MAX_SLEW_RATE	631
	//Default speed of the servo motors
	#define SERVO_DEFAULT_SPEED 40

		//-----------------------------------------------------------------------
		//	MOTOR ALIASES
		//-----------------------------------------------------------------------
		
	//enum servos names (positions and functions in the robot)
	enum
	{
		SERVO_WHEEL_RIGHT		= 0,	//
		SERVO_WHEEL_LEFT		= 1,	//
		SERVO_CAMERA_PAN		= 2,	//
		SERVO_CAMERA_TILT		= 3,	//
	};

	//enum servo direction. forward/inverted
	enum
	{
		SERVO_DIR_WHEEL_RIGHT		= (U8)0xff,	//
		SERVO_DIR_WHEEL_LEFT		= (U8)0xff,	//
		SERVO_DIR_CAMERA_PAN		= (U8)0xff,	//
		SERVO_DIR_CAMERA_TILT		= (U8)0xff,	//
	};

		//-----------------------------------------------------------------------
		//	MOTOR CORRECTIONS
		//-----------------------------------------------------------------------

	//servo direction correction mask, allow for the logical direction of a joint to be reversed in sign
	//each bit is associated with a servo, if the bit is '1', the direction is reversed
	#define SERVO_DIR	\
		(	\
			(SERVO_DIR_WHEEL_RIGHT && MASK(SERVO_WHEEL_RIGHT)) |\
			(SERVO_DIR_WHEEL_LEFT && MASK(SERVO_WHEEL_LEFT)) |\
			(SERVO_DIR_CAMERA_PAN && MASK(SERVO_CAMERA_PAN)) |\
			(SERVO_DIR_CAMERA_TILT && MASK(SERVO_CAMERA_TILT))\
		)
		
		///--------------------------------------------------------------------------
		///	COMMANDS
		///--------------------------------------------------------------------------
			
	//Communication timeout (tick@50Hz)
	#define UART_TIMEOUT		50
	//max length of board signature string
	#define MAX_SIGN_LEN		16
	//Ping command
	#define UART_CMD_PING		1
	//Sign command. Board is expected to answer with the signature string on UART
	#define UART_CMD_SIGN		2
	//Set desired forward and sideway velocity
	#define UART_CMD_SETVEL		10

	/****************************************************************************
	**	MACRO
	****************************************************************************/

		///----------------------------------------------------------------------
		///
		///----------------------------------------------------------------------

		//-----------------------------------------------------------------------
		//	TIMER 1 MACROS
		//-----------------------------------------------------------------------
		//	Timer 1 is used to wait precises delay to pull down the servo lines
		//	I have to start it up, stop it, clear the content and setup the delay

	//connect the clock at N=1
	#define START_TIMER1()	\
		SET_BIT( TCCR1B, CS10 )

	//disconnect the clock (if N==1)
	#define STOP_TIMER1()	\
		CLEAR_BIT( TCCR1B, CS10 )

		//-----------------------------------------------------------------------
		//	SERVOS MACROS
		//-----------------------------------------------------------------------
		//	Those macros are wrappers to access servos variables

	//access servo pos as matrix
	#define SERVO_POS( col, index )	\
		servo_pos[ N_SERVOS*(col) + (index) ]

	//access servo pos as matrix
	#define SERVO_TIME( col, index )	\
		servo_time[ N_SERVOS*(col) + (index) ]

	//offset
	#define SERVO_OFFSET( index )	\
		servo_off[ (index) ]


	/****************************************************************************
	**	TYPEDEF
	****************************************************************************/

	//Global flags raised by ISR functions
	typedef struct _Isr_flags Isr_flags;

	/****************************************************************************
	**	STRUCTURE
	****************************************************************************/

	//Global flags raised by ISR functions
	struct _Isr_flags
	{
		//First byte
		U8 lcd_update	: 1;	//Execute LCD update routine 9765[Hz]
		U8 servo_scan	: 1;	//start the motor scan
		U8 tick_slow	: 1;	//Fast tick for LCD counter
		U8 				: 5;	//unused bits
	};

	/****************************************************************************
	**	PROTOTYPE: INITIALISATION
	****************************************************************************/

	//port configuration and call the peripherals initialization
	extern void global_init( void );

	//9765 Hz Time Base
	extern void timer0_init( void );
	//servo lines handler
	extern void timer1_init( void );
	//UART communication
	extern void usart0_init( void );
	//Init the ADC module
	extern void adc_init( void );

	/****************************************************************************
	**	PROTOTYPE: FUNCTION
	****************************************************************************/

		///----------------------------------------------------------------------
		///	SERVO
		///----------------------------------------------------------------------

	//very complex function, it is called by the ISR to calculate the delay
	//delay is calculated from global time, initial and ending time and position of the servo
	extern U16 servo_calc_delay( U8 index );

	/****************************************************************************
	**	PROTOTYPE: GLOBAL VARIABILE
	****************************************************************************/

		///----------------------------------------------------------------------
		///	STATUS FLAGS
		///----------------------------------------------------------------------

	//Volatile flags used by ISRs
	extern volatile	Isr_flags flags;
	//Status variable for the servos, keep track of which servo to do next
	extern volatile	U8 servo_cnt;
	//each servo has 1 bit, it's rised when the interpolator is done
	extern volatile	U8 servo_idle;

		///----------------------------------------------------------------------
		///	BUFFERS
		///----------------------------------------------------------------------

	//Safe circular buffer for RS485 input data
	extern volatile At_buf8_safe uart_rx_buf;
	//Safe circular buffer for RS485 output data
	extern At_buf8 uart_tx_buf;

		///----------------------------------------------------------------------
		///	SERVO
		///----------------------------------------------------------------------

	//servo direction correction mask, allow for the logical direction of a joint to be reversed in sign
	//each bit is associated with a servo, if the bit is '1', the direction is reversed
	extern U8 servo_dir;
	//One flag per servo '1' if the servo is keeping up with target_pos
	extern U8 servo_lock;
	//servo position offset for true 0 position
	//offsets are accounted for in separately from the position, it does not eat into the dynamic
	extern S8 servo_off[ N_SERVOS ];
	//Current position of the servos. Used by the driver, user should not write here
	extern U16 servo_delay[ N_SERVOS ];
	//Target position. The user write here, the driver will do it's best to reach it
	extern S8 servo_target_pos[ N_SERVOS ];
	//The servo will rotate at this speed [unit/second]
	extern U8 servo_target_speed[ N_SERVOS ];

#else
	#warning "multiple inclusion of the header file global.h"
#endif
