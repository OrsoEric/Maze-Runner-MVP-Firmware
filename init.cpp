/****************************************************************************
**	INCLUDE
****************************************************************************/

#include "global.h"

/****************************************************************************
** GLOBAL INITIALISATION
****************************************************************************/

void global_init( void )
{
	///----------------------------------------------------------------------
	///	PORT I/O SETTING:
	///----------------------------------------------------------------------
	///	'I' = Input (HI-Z)
	///	'R' = Input (Rpu)
	///	'L' = Output (low)
	///	'H' = Output (Hi)
	///	the bit are in the order: LSB, ... , MSB
	///	TIPS: unused pin should be configured as 'R' to avoid power losses
	///	by spurious transition on the disconnected pin
	///	TIPS: I/O is the primary function of any pin, if you engage a peripheral
	///	that use a pin, that configuration will override the pin's I/O configuration
	///	TIPS: if you want to use oc0,1a,1b,2 waveform pins, you must
	///	configure the related I/O pin as output
	///----------------------------------------------------------------------

	//PA0			: LCD_D4
	//PA1			: LCD_D5
	//PA2			: LCD_D6
	//PA3			: LCD_D7
	//PA4			: LCD_RS
	//PA5			: LCD_EN
	//PA6			:
	//PA7			:
	PORT_A_CONFIG('L','L','L','L','H','H','R','R');

	//PB0			:
	//PB1			:
	//PB2			:
	//PB3			:
	//PB4			:
	//PB5			:
	//PB6			: LED0
	//PB7			: LED1
	PORT_B_CONFIG('R','R','R','R','R','R','H','H');

	//PC0			: SRV0 Servomotor
	//PC1			: SRV1 Servomotor
	//PC2			: SRV2 Servomotor
	//PC3			: SRV3 Servomotor
	//PC4			:
	//PC5			:
	//PC6			:
	//PC7			:
	PORT_C_CONFIG('L','L','L','L','R','R','R','R');

	//PD0			: RPI_TXO AT_RXI
	//PD1			: RPI_RXI AT_TXO
	//PD2			:
	//PD3			:
	//PD4			:
	//PD5			:
	//PD6			:
	//PD7			:
	PORT_D_CONFIG('R','H','R','R','R','R','R','R');

	///----------------------------------------------------------------------
	///	DEVICES INIT
	///----------------------------------------------------------------------

	//Init the serial port 1. Rx INT active
	usart0_init();
	//init Timer0. Handle user timing flags
	timer0_init();
	//Precise delay for the servos
	timer1_init();
	//Init the ADC module
	//adc_init();
		///LCD Init
	//Turn OFF Delay
	//This is meant to allow LCD display to safely power down and reset
	//If it's too short, the LCD will bug out when quickly doing ON -> OFF -> ON 
	//_delay_ms( 250.0 );
	//Power Up the LCD Display
	//CLEAR_BIT( PORTC, PC5 );
	//Turn ON Delay
	//This is meant to give the LCD Display time to safely power Up
	//If it's too short, The LCD will bug out when doing OFF -> ON -> COMMANDS
	_delay_ms( 250.0 );
	//Initialize the display and the driver: Send the sequences that configure the display
	lcd_init();

	///----------------------------------------------------------------------
	///	ENABLE ALL INTERRUPT:
	///	TIPS: sei() should be called after all the pheriperals have been configured,
	///	so that ISR will not mess up other initialization.
	/// TIPS: the "all interrupt enable flag" is automatically shut down at the
	///	start of any ISR and engaged again at it's bottom to avoid slow nested ISR call,
	///	if you need nested ISR call then call sei() at the beginning of the ISR
	///	TIPS: the function to disable all the interrupt is "cli()"
	///----------------------------------------------------------------------

	sei();

	return;
}	//end function: global_init

/****************************************************************************
** PHERIPERALS INITIALISATION
****************************************************************************/

void usart0_init( void )
{
	///----------------------------------------------------------------------
	//	STATIC VARIABILE
	///----------------------------------------------------------------------
	
	///----------------------------------------------------------------------
	//	LOCAL VARIABILE
	///----------------------------------------------------------------------

	uint8_t ucsr0a_temp = 0x00;
	uint8_t ucsr0b_temp = 0x00;
	uint8_t ucsr0c_temp = 0x00;
	
	///----------------------------------------------------------------------
	//	REGISTER CONFIGURATION
	///----------------------------------------------------------------------

	//Double the USART0 speed
	//SET_BIT( ucsr0a_temp, U2X0 );

	//Multi-processor mode
	//SET_BIT( ucsr0a_temp, MPCM0 );

	//Rx interrupt enable
	SET_BIT( ucsr0b_temp, RXCIE0 );

	//Tx interrupt enable
	//SET_BIT( ucsr0b_temp, TXCIE0 );

	//Tx buffer empty interrupt enable
	//SET_BIT( ucsr0b_temp, UDRIE0 );

	//Enable receiver
	SET_BIT( ucsr0b_temp, RXEN0 );

	//Enable Transmitter
	SET_BIT( ucsr0b_temp, TXEN0 );

	//Word size
	//	2	1	0	|	Mode
	//-------------------------------
	//	0	0	0	|	5 bits
	//	0	0	1	|	6 bits
	//	0	1	0	|	7 bits
	//	0	1	1	|	8 bits
	//	1	0	0	|	Reserved
	//	1	0	1	|	Reserved
	//	1	1	0	|	Reserved
	//	1	1	1	|	9 bits
	//SET_BIT( ucsr0b_temp, UCSZ02 );
	SET_BIT( ucsr0c_temp, UCSZ01 );
	SET_BIT( ucsr0c_temp, UCSZ00 );

	//Operation mode
	//	1	0	|	Mode
	//-------------------------------
	//	0	0	|	Asynchronous USART
	//	0	1	|	Synchronous USART
	//	1	0	|	Reserved
	//	1	1	|	Master SPI
	//SET_BIT( ucsr0c_temp, UMSEL01 );
	//SET_BIT( ucsr0c_temp, UMSEL00 );

	//Parity Check
	//	1	0	|	Mode
	//-------------------------------
	//	0	0	|	Disabled
	//	0	1	|	Reserved
	//	1	0	|	Even Parity
	//	1	1	|	Odd Parity
	//SET_BIT( ucsr0c_temp, UPM01 );
	//SET_BIT( ucsr0c_temp, UPM00 );

	//Two stop bits enabled
	//SET_BIT( ucsr0c_temp, USBS0 );

	//XCK0 clock polarity (synchronous mode only)
	//SET_BIT( ucsr0c_temp, UCPOL0 );

	///----------------------------------------------------------------------
	//	REGISTER WRITE BACK
	///----------------------------------------------------------------------

	UCSR0A = ucsr0a_temp;
	UCSR0B = ucsr0b_temp;
	UCSR0C = ucsr0c_temp;
	
	UBRR0H = 0;
	//UBRR0L = 8;
	UBRR0L = 21;

	return;
}	//end function: usart0_initialisation


/****************************************************************************
**	TIMER0 INITIALISATION
*****************************************************************************
**	This timer generate a ~100uS time base
**	Normal mode
**	Fclk = 20e6
**	F = Fclk / N / (OCR+1)
**	OCR = Fclk * T / N - 1
**	N = 64, OCR =  30 -> F = 10080.6 [Hz]
****************************************************************************/

void timer0_init( void )
{
	///----------------------------------------------------------------------
	///	STATIC VARIABILE
	///----------------------------------------------------------------------

	///----------------------------------------------------------------------
	///	LOCAL VARIABILE
	///----------------------------------------------------------------------

	//temp control register variable
	uint8_t tccr0a_temp = 0x00;
	uint8_t tccr0b_temp = 0x00;
	uint8_t timsk0_temp = 0x00;

	///----------------------------------------------------------------------
	///	CONTROL REGISTER SETTINGS
	///----------------------------------------------------------------------

	//	1	0	|	OC0A
	//-------------------------------
	//	0	0	|	Disconnected
	//	0	1	|	Toggle on CM
	//	1	0	|	Clear on CM
	//	1	1	|	Set on CM

	//SET_BIT( tccr0a_temp, COM0A0 );
	//SET_BIT( tccr0a_temp, COM0A1 );

	//	1	0	|	OC0B
	//-------------------------------
	//	0	0	|	Disconnected
	//	0	1	|	Toggle on CM
	//	1	0	|	Clear on CM
	//	1	1	|	Set on CM

	//SET_BIT( tccr0a_temp, COM0B0 );
	//SET_BIT( tccr0a_temp, COM0B1 );

	//Compare Out Mode
	//	2	1	0	|	Mode
	//-------------------------------
	//	0	0	0	|	Normal (TOP = 0xff)
	//	0	0	1	|	PWM Phase Correct (TOP = 0xff)
	//	0	1	0	|	CTC (TOP = OCRA)
	//	0	1	1	|	Fast PWM (TOP = 0xff)
	//	1	0	0	|	Reserved
	//	1	0	1	|	PWM Phase Correct (TOP = OCRA)
	//	1	1	0	|	Reserved
	//	1	1	1	|	Fast PWM (TOP = OCRA)
		
	//SET_BIT( tccr0a_temp, WGM00 );
	SET_BIT( tccr0a_temp, WGM01 );
	//SET_BIT( tccr0b_temp, WGM02 );

	///Clock Source
	///	CS02	|	CS01	|	CS00	|| 
	///-------------------------------------------
	///	0		|	0		|	0		|| no clock
	///	0		|	0		|	1		|| clk / 1
	///	0		|	1		|	0		|| clk / 8
	///	0		|	1		|	1		|| clk / 64
	///	1		|	0		|	0		|| clk / 256
	///	1		|	0		|	1		|| clk / 1024
	///	1		|	1		|	0		|| T0 pin, falling edge
	///	1		|	1		|	1		|| T0 pin, rising edge

	//SET_BIT( tccr0b_temp, CS02 );
	SET_BIT( tccr0b_temp, CS01 );
	SET_BIT( tccr0b_temp, CS00 );

	//Output compare 0a interrupt
	//This interrupt is launched when TCNT0 is equal to OCR0A
	SET_BIT( timsk0_temp, OCIE0A );
	
	//Output compare 0a interrupt
	//This interrupt is launched when TCNT0 is equal to OCR0B
	//SET_BIT( timsk0_temp, OCIE0B );
	
	//overflow interrupt enable
	//is launched when TCNT0 goes in overflow
	//SET_BIT( timsk0_temp, TOIE0 );

	///----------------------------------------------------------------------
	///	CONTROL REGISTER WRITEBACK
	///----------------------------------------------------------------------

	TCCR0A = tccr0a_temp;
	TCCR0B = tccr0b_temp;
	TIMSK0 = timsk0_temp;

	OCR0A = 30;
	OCR0B = 0xff;
			
	return;
}	//end function: timer0_initialisation

/****************************************************************************
** TIMER1 INITIALISATION
*****************************************************************************
**	8x16bit PPM Channels
**	I need to handle up to 8 special PWM channels for the servomotors
**	I use this this timer as an interrupt
**	countdown to pull down the channels at the right moment
****************************************************************************/

void timer1_init( void )
{
	//***********************************************************************
	//	STATIC VARIABILE
	//***********************************************************************

	//***********************************************************************
	//	LOCAL VARIABILE
	//***********************************************************************

	uint8_t tccr1a_temp = 0x00;
	uint8_t tccr1b_temp = 0x00;
	uint8_t tccr1c_temp = 0x00;
	uint8_t timsk1_temp = 0x00;

	//***********************************************************************
	//	REGISTER CONFIGURATION
	//***********************************************************************

	//OC1A pin behavior
	//	1	0	|	OC1A
	//-------------------------------
	//	0	0	|	Disconnected
	//	0	1	|	Toggle on CM if WGM1(3:0) = 15, Disconnected otherwise
	//	1	0	|	Clear on CM, Set at Bottom (non inverting mode)
	//	1	1	|	Set on CM, Clear at Bottom (inverting mode)
	//SET_BIT( tccr1a_temp, COM1A1 );
	//SET_BIT( tccr1a_temp, COM1A0 );

	//OC1B pin behavior
	//	1	0	|	OC1B
	//-------------------------------
	//	0	0	|	Disconnected
	//	0	1	|	Disconnected
	//	1	0	|	Clear on CM, Set at Bottom (non inverting mode)
	//	1	1	|	Set on CM, Clear at Bottom (inverting mode)
	//SET_BIT( tccr1a_temp, COM1B1 );
	//SET_BIT( tccr1a_temp, COM1B0 );

	//CTC mode with TOP in OCR1A
	//SET_BIT( tccr1b_temp, WGM13 );
	SET_BIT( tccr1b_temp, WGM12 );
	//SET_BIT( tccr1a_temp, WGM11 );
	//SET_BIT( tccr1a_temp, WGM10 );

	//Input capture noise canceler enabled
	//SET_BIT( tccr1b_temp, ICNC1 );

	//Input capture edge select
	//SET_BIT( tccr1b_temp, ICES1 );

	//Clock and Prescaler selection
	//	2	1	0	|	Mode
	//-------------------------------
	//	0	0	0	|	Disconnected
	//	0	0	1	|	Clk/1
	//	0	1	0	|	Clk/8
	//	0	1	1	|	Clk/64
	//	1	0	0	|	Clk/256
	//	1	0	1	|	Clk/1024
	//	1	1	0	|	T1 pin falling edge
	//	1	1	1	|	T1 pin rising edge
	//SET_BIT( tccr1b_temp, CS12 );
	//SET_BIT( tccr1b_temp, CS11 );
	//SET_BIT( tccr1b_temp, CS10 );

	//Input Time Capture Interrupt Enable
	//SET_BIT( timsk1_temp, ICIE1 );

	//Compare Match 1A Interrupt Enable
	SET_BIT( timsk1_temp, OCIE1A );

	//Compare Match 1B Interrupt Enable
	//SET_BIT( timsk1_temp, OCIE1B );

	//Timer Overflow Interrupt Enable 1
	//SET_BIT( timsk1_temp, TOIE1 );

	//***********************************************************************
	//	REGISTER WRITE-BACK
	//***********************************************************************

	OCR1A 	= 0xffff;

	OCR1B	= 0;

	ICR1 	= 0;

	TCCR1A = tccr1a_temp;
	TCCR1B = tccr1b_temp;
	TCCR1C = tccr1c_temp;
	TIMSK1 = timsk1_temp;

	return;
}	//end function: timer1_init

/****************************************************************************
** ADC INITIALISATION
*****************************************************************************
**	I will use the ADC in single capture mode, triggered by the timing ISR
**	I will use the ADC voltage as supply, in future i can use the internal 1,1V
**	reference for calibration purpose
**
**	I will read:
**	CHANNEL	|	FUNCTION	|	Sample Time
**	---------------------------------------
**	ADC0	|	CH0			|
**	ADC1	|	CH1			|
**	ADC8	|	UcTemp		|
**
**	The SAR frequency has to be between 50KHz and 200KHz to achieve the
**	maximum resolution, the prescaler will be:
**	Fclk	|	Nadc	|	Fsar
**	---------------------------------
**	20e6	|	128		|	156.250 [Hz]
**	8e6		|	64		|	125.000 [Hz]
**
****************************************************************************/

void adc_init( void )
{
	///----------------------------------------------------------------------
	///	STATIC VARIABILE
	///----------------------------------------------------------------------

	///----------------------------------------------------------------------
	///	LOCAL VARIABILE
	///----------------------------------------------------------------------

	uint8_t admux_temp		= 0x00;
	uint8_t adcsra_temp		= 0x00;
	uint8_t adcsrb_temp		= 0x00;
	uint8_t	didr0_temp		= 0x00;

	///----------------------------------------------------------------------
	///	VOLTAGE REFERENCE SETTINGS
	///----------------------------------------------------------------------

	///	REFS1	|	REFS0	||	
	///	-------------------------------
	///	0		|	0		||	Aref, internal reference disabled
	///	0		|	1		||	Aref connected to AVcc (capacitor on Aref)
	///	1		|	0		||	Reserved
	///	1		|	1		||	Aref = internal 1,1 voltage reference (capacitor on Aref)

	//SET_BIT( admux_temp, REFS1 );
	SET_BIT( admux_temp, REFS0 );

	///----------------------------------------------------------------------
	///	ADC LEFT ADJUST ENABLE
	///----------------------------------------------------------------------
	///Note: The ADC register is not updated until ADCH is read, so read ADCL FIRST
	///If this bit is written zero, the representation will be:
	///	
	///			|	B7	|	B6	|	B5	|	B4	|	B3	|	B2	|	B1	|	B0	|
	///	--------------------------------------------------------------------------
	///	ADCH	|	-	|	-	|	-	|	-	|	-	|	-	|	A9	|	A8	|
	///	ADCL	|	A7	|	A6	|	A5	|	A4	|	A3	|	A2	|	A1	|	A0	|
	///
	///If this bit is written one, the representation will be:
	///	
	///			|	B7	|	B6	|	B5	|	B4	|	B3	|	B2	|	B1	|	B0	|
	///	--------------------------------------------------------------------------
	///	ADCH	|	A9	|	A8	|	A7	|	A6	|	A5	|	A4	|	A3	|	A2	|
	///	ADCL	|	A1	|	A0	|	-	|	-	|	-	|	-	|	-	|	-	|
	///

	//SET_BIT( admux_temp, ADLAR );

	///----------------------------------------------------------------------
	///	ANALOG SIGNAL MULTIPLEXER SETTINGS
	///----------------------------------------------------------------------
	///	MUX3	|	MUX2	|	MUX1	|	MUX0	||	Source
	///	--------------------------------------------------------------------------------
	///	0		|	0		|	0		|	0		||	ADC0
	///	0		|	0		|	0		|	1		||	ADC1
	///	0		|	0		|	1		|	0		||	ADC2
	///	0		|	0		|	1		|	1		||	ADC3
	///	0		|	1		|	0		|	0		||	ADC4
	///	0		|	1		|	0		|	1		||	ADC5
	///	0		|	1		|	1		|	0		||	ADC6
	///	0		|	1		|	1		|	1		||	ADC7
	///	1		|	0		|	0		|	0		||	ADC8 (Temperature Sensor)
	///	1		|	0		|	0		|	1		||	Reserved
	///	1		|	0		|	1		|	0		||	Reserved
	///	1		|	0		|	1		|	1		||	Reserved
	///	1		|	1		|	0		|	0		||	Reserved
	///	1		|	1		|	0		|	1		||	Reserved
	///	1		|	1		|	1		|	0		||	1,1V
	///	1		|	1		|	1		|	1		||	0V
	
	//SET_BIT( admux_temp, MUX3 );
	//SET_BIT( admux_temp, MUX2 );
	//SET_BIT( admux_temp, MUX1 );
	//SET_BIT( admux_temp, MUX0 );

	///----------------------------------------------------------------------
	///	ADC ENABLE
	///----------------------------------------------------------------------

	SET_BIT( adcsra_temp, ADEN );

	///----------------------------------------------------------------------
	///	ADC CONVERSION START 
	///----------------------------------------------------------------------
	/// writing this bit to one will start a conversion, the bit is cleared
	///	by hardware when the conversion is complete
	
	//SET_BIT( adcsra_temp, ADSC );

	///----------------------------------------------------------------------
	///	ADC AUTOTRIGGER ENABLE
	///----------------------------------------------------------------------
	///	The adc will automatically start a new conversion on a positive edge of
	///	the selected trigger source

	//SET_BIT( adcsra_temp, ADATE );

	///----------------------------------------------------------------------
	///	ADC AUTOTRIGGER SOURCE SETTINGS
	///----------------------------------------------------------------------
	///
	///	ADTS2	|	ADTS1	|	ADTS0	||	TRIGGER SOURCE
	///	-----------------------------------------------------
	///	0		|	0		|	0		||	Free Running
	///	0		|	0		|	1		||	Analog Comparator
	///	0		|	1		|	0		||	External Interrupt 0
	///	0		|	1		|	1		||	Timer0 Compare Match A
	///	1		|	0		|	0		||	Timer0 Overflow
	///	1		|	0		|	1		||	Timer1 Compare Match B
	///	1		|	1		|	0		||	Timer1 Overflow
	///	1		|	1		|	1		||	Timer1 Capture Event

	//SET_BIT( adcsrb_temp, ADTS2 );
	//SET_BIT( adcsrb_temp, ADTS1 );
	//SET_BIT( adcsrb_temp, ADTS0 );

	///----------------------------------------------------------------------
	///	ADC INTERRUPT ENABLE
	///----------------------------------------------------------------------
	///	The ADC ISR will be called upon a completed conversion if this bit is
	///	written to one

	SET_BIT( adcsra_temp, ADIE );

	///----------------------------------------------------------------------
	///	ADC SAR PRESCALER SETTINGS
	///----------------------------------------------------------------------
	///	Select the prescaler from the system clock and the SAR clock, the maximum
	///	resolution of the ADC is achieved between 50KHz and 200KHz of SAR frequency
	///
	///	ADPS2	|	ADPS1	|	ADPS0	||	PRESCALER
	///	-----------------------------------------------
	///	0		|	0		|	0		||	2
	///	0		|	0		|	1		||	2
	///	0		|	1		|	0		||	4
	///	0		|	1		|	1		||	8
	///	1		|	0		|	0		||	16
	///	1		|	0		|	1		||	32
	///	1		|	1		|	0		||	64
	///	1		|	1		|	1		||	128

	SET_BIT( adcsra_temp, ADPS2 );
	SET_BIT( adcsra_temp, ADPS1 );
	SET_BIT( adcsra_temp, ADPS0 );

	///----------------------------------------------------------------------
	///	ANALOG COMPARATOR MULTIPLEXER ENABLE
	///----------------------------------------------------------------------

	//SET_BIT( adcsrb_temp, ACME );

	///----------------------------------------------------------------------
	///	DIGITAL INPUT DISABLE
	///----------------------------------------------------------------------
	///	Setting a bit of this register to one will disable the digital
	///	input circuitry for that pin, this will reduce power consumption too
	
	//SET_BIT( didr0_temp, ADC5D );
	//SET_BIT( didr0_temp, ADC4D );
	//SET_BIT( didr0_temp, ADC3D );
	//SET_BIT( didr0_temp, ADC2D );
	SET_BIT( didr0_temp, ADC1D );
	SET_BIT( didr0_temp, ADC0D );

	///----------------------------------------------------------------------
	///	REGISTER TEMP VARIABILE WRITE BACK
	///----------------------------------------------------------------------

	ADMUX		= admux_temp;
	ADCSRA		= adcsra_temp;
	ADCSRB		= adcsrb_temp;
	DIDR0		= didr0_temp;

	return;
}	//end function: adc_init
