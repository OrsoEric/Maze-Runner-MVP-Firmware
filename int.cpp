/****************************************************************************
**	INCLUDE
****************************************************************************/

#include "global.h"

/****************************************************************************
** INTERRUPT SERVICE ROUTINE
*****************************************************************************
**	Interrupt Vector (sorted by priority):
**	>	RESET_vect				: issued by any reset
**	>	INT0_vect				: issued by external pin
**	>	INT1_vect				: **
**	>	INT2_vect				: **
**	>	TIMER2_COMP_vect		: issued when TCNT2 is equal to OCR2
**	>	TIMER2_OVF_vect			: issued when TCNT2 reach the maximum
**	>	TIMER1_CAPT_vect		: issued when an event is detected on ICP1 pin
**	>	TIMER1_COMPA_vect		: issued when TCNT1 is equal to OCR1A
**	>	TIMER1_COMPB_vect		: issued when TCNT1 is equal to OCR1B
**	>	TIMER1_OVF_vect			: issued when TCNT1 reach maximum
**	>	TIMER0_COMP_vect		: issued when TCNT0 is equal to OCR0
**	>	TIMER0_OVF_vect			: issued when TCNT0 reach maximum
**	>	SPI_STC_vect			: issued when a serial trasmission is completed
**	>	USART_RXC_vect			: issued when the usart reception buffer is not empty
**	>	USART_UDRE_vect			: issued when the usart trasmission buffer is empty
**	>	USART_TXC_vect			: issued when the usart module have completed a trasmission
**	>	ADC_vect				: issued when the ADC module have completed a conversion
**	>	EE_RDY_vect				: issued by the eeprom module
**	>	ANA_COMP_vect			: issued when the analog comparator has detected an event
**	>	TWI_vect				: issued by the twi module
**	>	SPM_RDY_vect			: ???
**
**	example:
**	ISR( TIMER0_COMP_vect )
**	{
**		//call sei() if you want this ISR to allow SLOW nested ISR call
**		//put your own code here
**		//you mustn't use return anywhere in the ISR
**	}
**
**	TIPS: the "all interrupt enable flag" is automatically cleared at the
**	start of any ISR and rised up again at it's bottom to avoid slow nested ISR call,
**	if you need nested ISR call then call sei() at the beginning of the ISR
**	TIPS: if two interrupt are issued at the same time, the one with lower address will
**	be executed first (lower address mean higher priority, this interrupt vector table is fixed)
**	TIPS: INT0,1,2 are issued even if the related pin is configurated as output, so
**	you can use them to generate software interrupt
**	TIPS: if you want a ISR to manipulate a global variable, than that variable
**	**must** be declared "volatile" so that the c compiler will not wipe out the
**	variable by optimizing the code, use that variable as less as possible
**	because it will not be optimized
**	TIPS: since nested ISR are suppressed, ISR code should be as short as possible (to not
**	block other incoming ISR), to do this, a good idea can be to rise a flag inside
**	the low priority ISR and read and clear that flag inside the main loop, in this way the ISR
**	last the least possible time
****************************************************************************/

/****************************************************************************
**	TIMER 0 OVERFLOW
*****************************************************************************
**	9765 Hz Time base
**	Generate 9765Hz tick for LCD display
**	Generate 50Hz tick for servo PPM generation
**	Generate system 10Hz tick
****************************************************************************/

ISR( TIMER0_COMPA_vect )
{
	///----------------------------------------------------------------------
	///	STATIC VARS
	///----------------------------------------------------------------------

	static U8 cnt8 = 0;

	///----------------------------------------------------------------------
	///	LOCAL VARS
	///----------------------------------------------------------------------

	///----------------------------------------------------------------------
	///	INIT
	///----------------------------------------------------------------------

	///----------------------------------------------------------------------
	///	BODY
	///----------------------------------------------------------------------

		///	9765 Hz Tick
	//Update LCD display
	flags.lcd_update = 1;
	
		///	50.08 Hz Tick
	//if: counter has reset
	if (cnt8 == 0)
	{	
		//Generate PPM signal
		flags.servo_scan = 1;
		//Start a conversion
		//SET_BIT( ADCSRA, ADSC );
	}
	//50Hz Prescaler
	cnt8 = AT_TOP_INC( cnt8, 195 );

	///----------------------------------------------------------------------
	///	RETURN
	///----------------------------------------------------------------------
	//	No return in ISR
}	//end ISR: TIMER0_COMPA_vect

/****************************************************************************
**	TIMER1_COMPA_vect
*****************************************************************************
**	This function is a controlled delay used to generate the servos signals
****************************************************************************/

ISR( TIMER1_COMPA_vect )
{
	//***********************************************************************
	//	STATIC VARIABLES
	//***********************************************************************

	//***********************************************************************
	//	LOCAL VARIABLES
	//***********************************************************************

	//temp status var
	U8 cnt;
	//calculation of delay
	U16 delay;

	//***********************************************************************
	//	BODY
	//***********************************************************************
	//	>Activity pin (led signal uC use, oscilloscope allow to measure function times)
	//	>Fetch status var
	//	>If valid: 
	//		>pull down previous line
	//		>update current motor index
	//	>If valid:
	//		>pull up line
	//		>calculate delay
	//		>setup delay on Timer 1
	//	>If not valid
	//		>stop Timer 1
	//	>write back status var

	//fetch status var
	cnt = servo_cnt;
	//If i still have a pending servo, pull it down
	if (cnt < N_SERVOS)
	{
		//pull down line
		CLEAR_BIT( SERVO_PORT, cnt +SERVO_PIN_OFFSET );
		//next servo
		cnt++;
	}
	//If i still have servos to do, set the next one up
	if (cnt < N_SERVOS)
	{
		//pull up line
		SET_BIT( SERVO_PORT, cnt +SERVO_PIN_OFFSET );
		//calculate delay (position and times are global variables, it's an interpolator)
		delay 			= servo_calc_delay( cnt );
		//Store delay on T1
		OCR1A 			= delay;
	}
	//If: i did the last servo
	else
	{
		//stop the timer, I'm done
		STOP_TIMER1();
	}
	//write back status var
	servo_cnt = cnt;

	//***********************************************************************
	//	RETURN
	//***********************************************************************
}	//End ISR: TIMER1_COMPA_vect

/****************************************************************************
**	USART0 RX COMPLETE
*****************************************************************************
**	PC RX data
****************************************************************************/

ISR( USART0_RX_vect )
{
	///----------------------------------------------------------------------
	///	STATIC VARS
	///----------------------------------------------------------------------

	///----------------------------------------------------------------------
	///	LOCAL VARS
	///----------------------------------------------------------------------

	U8 rxd;

	///----------------------------------------------------------------------
	///	INIT
	///----------------------------------------------------------------------

	//Fetch data from UART rx buffer
	rxd = UDR0;

	///----------------------------------------------------------------------
	///	BODY
	///----------------------------------------------------------------------

	//Push byte into RS485 buffer for processing
	AT_BUF_PUSH_SAFER( uart_rx_buf, rxd );

	///----------------------------------------------------------------------
	///	RETURN
	///----------------------------------------------------------------------
	//	No return in ISR
}	//end ISR: USART0_RX_vect

/****************************************************************************
**	ADC_vect
*****************************************************************************
**	I will read channel 0, channel 1 and the uC temperature
****************************************************************************/

ISR( ADC_vect )
{
	///----------------------------------------------------------------------
	///	STATIC VARS
	///----------------------------------------------------------------------

	///----------------------------------------------------------------------
	///	LOCAL VARS
	///----------------------------------------------------------------------

	U16 adc_data;

	///----------------------------------------------------------------------
	///	INIT
	///----------------------------------------------------------------------

	//fetch ADC reading
	adc_data = ADC;

	lcd_print_u8( LCD_POS( 1, 0), (U8)((U16)adc_data/4) );

	///----------------------------------------------------------------------
	///	BODY
	///----------------------------------------------------------------------

	///----------------------------------------------------------------------
	///	RETURN
	///----------------------------------------------------------------------
	//	No return in ISR
}	//end ISR: ADC_vect


/****************************************************************************
**	TIMER0_COMPA_vect
*****************************************************************************
**
****************************************************************************/

//ISR( TIMER0_COMPA_vect )

/****************************************************************************
**	INT0_vect
*****************************************************************************
**
****************************************************************************/

//ISR( INT0_vect )

/****************************************************************************
**	INT10_vect
*****************************************************************************
**
****************************************************************************/

//ISR( INT1_vect )
