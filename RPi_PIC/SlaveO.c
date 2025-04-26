
/* Parts of this code were modified from
 *  http://www.d.umn.edu/~cprince/PubRes/Hardware/SPI/
 * examples
 *
 * Fully interrupt driven SPI slave ADC for RPi via the daq_gert linux module
 * 8722
 * Port E is the main led diag port
 * PORT H is the LCD port
 * 25k22
 * Pins C0,C1 are the diag LED pins.
 * SPI 2 has been config'd as the slave with chip select.
 * DIP8 Pins for MCP3002 header
 * Pin 21   RB0	SPI Chip-Select	Pin 1
 * Pin 22   RB1	SPI Clock	Pin 7
 * Pin 23   RB2	SPI Data In	Pin 5
 * Pin 24   RB3	SPI Data Out	Pin 6
 * Pin 8    Vss			Pin 4
 * Pin 20   Vdd			Pin 8
 * Pin 2    RA0	ANA0		Pin 2
 * Pin 3    RA1	ANA1		Pin 3
 * The I/O and clock pins IDC connector pins
 * have been interconnected in the standard way for a PIC18F8722 chip EET Board
 *
 * Version
 *              1.0 stable version for daq_gert P25K22 4_TAD
 *		0.91 update exchange protocol
 *		0.9 add 45K80 commands and ports
 *		0.8 Add zero command for cleaner transfers and allow for no LCD code	
 *		0.7 minor software cleanups.
 *		0.06 P25K22 Set PIC speed to 64mhz and use have ADC use FOSC_64,12_TAD
 *		P8722 have ADC use FOSC_32,12_TAD
 *		0.05 Fixed the P25K22 version to work correctly.
 *		0.04 The testing hardware is mainly a pic18f8722 with a
 *		LCD display and PORTE bit leds.
 *		define the CPU type below.
 *
 *		The WatchDog and timer0 are used to check link status
 *		and to reset the chip if hung or confused.
 *
 * nsaspook@sma2.rain..com    Sept 2016
 */


#define P25K22

/*
 * bit 7 high for commands sent from the MASTER
 * bit 6 0 send lower or 1 send upper byte ADC result first
 * bits 3..0 port address
 *
 * bit 7 low  for config data sent in CMD_DUMMY per uC type
 * bits 6 config bit code always 1
 * bit	5 0=ADC ref VDD, 1=ADC rec FVR=2.048
 * bit  4 0=10bit adc, 1=12bit adc
 * bits 3..0 number of ADC channels
 * 
 */

#define	TIMEROFFSET	26474           // timer0 16bit counter value for 1 second to overflow
#define SLAVE_ACTIVE	10		// Activity counter level

/* PIC Slave commands */
#define CMD_ZERO        0b00000000
#define CMD_ADC_GO	0b10000000
#define CMD_PORT_GO	0b10100000	// send data LO_NIBBLE to port buffer
#define CMD_CHAR_GO	0b10110000	// send data LO_NIBBLE to TX buffer
#define CMD_ADC_DATA	0b11000000
#define CMD_PORT_DATA	0b11010000	// send data HI_NIBBLE to port buffer ->PORT and return input PORT data in received SPI data byte
#define CMD_CHAR_DATA	0b11100000	// send data HI_NIBBLE to TX buffer and return RX buffer in received SPI data byte
#define CMD_XXXX	0b11110000	//
#define CMD_CHAR_RX	0b00010000	// Get RX buffer
#define CMD_DUMMY_CFG	0b01000000	// stuff config data in SPI buffer
#define CMD_DEAD        0b11111111      // This is usually a bad response

#ifdef P8722
#define CMD_DUMMY       0b01001100	/* 12 channels VDD */
#define NUM_AI_CHAN     12
#define SPI_BUF		SSP2BUF		
#endif
#ifdef P25K22
#define CMD_DUMMY	0b01101110	/* 14 channels 2.048 but only 13 are ADC */
#define NUM_AI_CHAN     14
#define SPI_BUF		SSP2BUF
#endif
#ifdef P45K80
#define CMD_DUMMY	0b00111010	/* 10 channels 2.048 but only 9 are ADC,bit 6 set for rs232 data waiting */
#define NUM_AI_CHAN     10
#define UART_TX_MASK	0b10000000
#define UART_RX_MASK	0b01000000
#define SPI_BUF		SSPBUF
#endif

#define	HI_NIBBLE	0xf0
#define	LO_NIBBLE	0x0f
#define	ADC_SWAP_MASK	0b01000000
#define UART_DUMMY_MASK	0b01000000

/* LCD defines */
#define LCD_L           4                       // lines
#define LCD_W           20			// chars per line
#define LCD_STR         22			// char string for LCD messages
#define LCDW_SIZE       21			// add term char
#define MESG_W          250			// message string buffer
#define	LL1		0x00                    // LCD line addresses
#define	LL2		0x40
#define LL3		0x14
#define	LL4		0x54
#define	VC_MAX		3
#define VS_SLOTS	12                      // storage array size
#define	VC0		0			// LCD Virtual Screens
#define	VC1		4
#define	VC2		8
#define VS0		0			// Virtual screen select
#define VS1		1
#define VS2		2
#define	DS0		0			// LCD line index
#define	DS1		1
#define	DS2		2
#define	DS3		3
#define	DS4		4
#define	DS5		5
/* DIO defines */
#define LOW		0        // digital output state levels, sink
#define	HIGH		1        // digital output state levels, source
#define	SON		LOW       		//
#define SOFF		HIGH			//
#define	S_ON            LOW       		// low select/on for chip/led
#define S_OFF           HIGH			// high deselect/off chip/led
#define	R_ON            HIGH       		// control relay states, relay is on when output gate is high, uln2803,omron relays need the CPU at 5.5vdc to drive
#define R_OFF           LOW			// control relay states
#define R_ALL_OFF       0x00
#define R_ALL_ON	0xff
#define NO		LOW
#define YES		HIGH

#ifdef P8722
#define DLED0		LATEbits.LATE0
#define DLED1		LATEbits.LATE1
#define DLED2		LATEbits.LATE2
#define DLED3		LATEbits.LATE3
#define DLED4		LATEbits.LATE4
#define DLED5		LATEbits.LATE5
#define DLED6		LATEbits.LATE6
#define DLED7		LATEbits.LATE7
#endif
#ifdef P25K22
#define DLED0		LATCbits.LATC0
#define DLED1		LATCbits.LATC1
#define DLED2		LATCbits.LATC1
#define DLED3		LATCbits.LATC1
#define DLED4		LATCbits.LATC1
#define DLED5		LATCbits.LATC1
#define DLED6		LATCbits.LATC1
#define DLED7		LATCbits.LATC1
#endif
#ifdef P45K80
#define DLED0		LATDbits.LATD0
#define DLED1		LATDbits.LATD1
#define DLED2		LATDbits.LATD2
#define DLED3		LATDbits.LATD3
#define DLED4		LATDbits.LATD4
#define DLED5		LATDbits.LATD5
#define DLED6		LATAbits.LATA6
#define DLED7		LATAbits.LATA7
#endif

#include <xc.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "nslaven.X/mcc_generated_files/mcc.h"

struct lcdb {
	int8_t b[LCDW_SIZE];
};

struct spi_link_type { // internal state table
	uint8_t SPI_DATA : 1;
	uint8_t ADC_DATA : 1;
	uint8_t PORT_DATA : 1;
	uint8_t CHAR_DATA : 1;
	uint8_t REMOTE_LINK : 1;
	uint8_t REMOTE_DATA_DONE : 1;
	uint8_t LOW_BITS : 1;
};

struct spi_stat_type {
	volatile uint32_t adc_count, adc_error_count,
	port_count, port_error_count,
	char_count, char_error_count,
	slave_int_count, last_slave_int_count,
	comm_count;
	volatile uint8_t comm_ok;
};

struct serial_bounce_buffer_type {
	uint8_t data[2];
	uint32_t place;
};

volatile struct spi_link_type spi_comm = {false, false, false, false, false, false, false};
volatile struct spi_stat_type spi_stat = {0}, report_stat = {0};

const char *build_date = __DATE__, *build_time = __TIME__;
const char Version[] = " Version 1.0 PIC Slave ";
volatile uint8_t data_in2, adc_buffer_ptr = 0,
	adc_channel = 0;

volatile uint8_t dsi = 0; // LCD virtual console number
volatile uint16_t adc_buffer[64] = {0}, adc_data_in = 0;

int8_t bootstr2[MESG_W + 1];
uint8_t lcd18 = 200;
struct lcdb ds[VS_SLOTS];


void InterruptHandlerHigh(void);

//High priority interrupt vector
#pragma code InterruptVectorHigh = 0x08

void InterruptVectorHigh(void)
{

}

//----------------------------------------------------------------------------
// High priority interrupt routine

#pragma code

#pragma tmpdata ISRHtmpdata
#pragma interrupt InterruptHandlerHigh nosave=section (".tmpdata")

void InterruptHandlerHigh(void)
{
	static uint8_t channel = 0, link, upper, command, port_tmp, char_txtmp, char_rxtmp, cmd_dummy = CMD_DUMMY, b_dummy;
	static union Timers timer;

	DLED1 = HIGH;

#if defined(P8722) || defined(P25K22)
		/* we only get this when the master  wants data, the slave never generates one */
		if (PIR3bits.SSP2IF) { // SPI port #2 SLAVE receiver
			PIR3bits.SSP2IF = LOW;
#endif		
			spi_stat.slave_int_count++;
			data_in2 = SPI_BUF;
			command = data_in2 & HI_NIBBLE;

			if (command == CMD_ADC_GO) { // Found a GO for a conversion command
				spi_comm.ADC_DATA = false;
				if (data_in2 & ADC_SWAP_MASK) {
					upper = true;
				} else {
					upper = false;
				}
				channel = data_in2 & LO_NIBBLE;
#ifdef P25K22
				if (channel >= 5) channel += 6; // skip missing channels
				if (channel == 12 || channel > 19) channel = 0; // invalid so set to 0
#endif
			
				if (!ADCON0bits.GO) {
#if defined(P8722) || defined(P25K22)					
					ADCON0 = ((channel << 2) & 0b00111100) | (ADCON0 & 0b11000011);
#endif

					adc_buffer[channel] = 0xffff; // fill with bits
					ADCON0bits.GO = HIGH; // start a conversion
				} else {
					ADCON0bits.GO = LOW; // stop a conversion
					SPI_BUF = CMD_DUMMY; // Tell master  we are here
				}
				spi_comm.REMOTE_LINK = true;
				link = true;
				DLED0 = HIGH;
				/* reset link data timer if we are talking */
				timer.lt = TIMEROFFSET; // Copy timer value into union
				TMR0H = timer.bt[HIGH]; // Write high byte to Timer0
				TMR0L = timer.bt[LOW]; // Write low byte to Timer0
				INTCONbits.TMR0IF = LOW; //clear possible interrupt flag	
			}
			if (data_in2 == CMD_DUMMY_CFG) {
				SPI_BUF = CMD_DUMMY; // Tell master  we are here
			}

			if ((data_in2 == CMD_ZERO) && spi_comm.ADC_DATA) { // don't sent unless we have valid data
				spi_stat.last_slave_int_count = spi_stat.slave_int_count;
				if (upper) {
					SPI_BUF = ADRESH;
				} else {
					SPI_BUF = ADRESL; // stuff with lower 8 bits
				}
			}
			if (data_in2 == CMD_ADC_DATA) {
				if (spi_comm.ADC_DATA) {
					if (upper) {
						SPI_BUF = ADRESL; // stuff with lower 8 bits
					} else {
						SPI_BUF = ADRESH;
					}
					spi_stat.last_slave_int_count = spi_stat.slave_int_count;
				} else {
					SPI_BUF = CMD_DUMMY;
				}
			}
			if (command == CMD_CHAR_RX) {
				SPI_BUF = char_rxtmp; // Send current RX buffer contents
				cmd_dummy = CMD_DUMMY; // clear rx bit
			}
		}

		if (INTCONbits.TMR0IF) { // check timer0 irq 1 second timer int handler
			INTCONbits.TMR0IF = LOW; //clear interrupt flag
			//check for TMR0 overflow
			LATBbits.LATB7 = !LATBbits.LATB7;

			timer.lt = TIMEROFFSET; // Copy timer value into union
			TMR0H = timer.bt[HIGH]; // Write high byte to Timer0
			TMR0L = timer.bt[LOW]; // Write low byte to Timer0
			DLED0 = LOW;
		}

		if (PIR1bits.ADIF) { // ADC conversion complete flag
			DLED0 = LOW;
			PIR1bits.ADIF = LOW;
			spi_stat.adc_count++; // just keep count
			adc_buffer[channel] = (uint16_t) ADRES; // data is ready but must be written to the SPI buffer before a master command is received 
			if (upper) { /* same as CMD_ZERO */
				SPI_BUF = ADRESH;
			} else {
				SPI_BUF = ADRESL; // stuff with lower 8 bits
			}
			spi_comm.ADC_DATA = true; // so the transmit buffer will not be overwritten, WCOL set
			DLED0 = HIGH;
		}
		DLED1 = LOW;
	}
#pragma	tmpdata

	void wdtdelay(unsigned long delay)
	{
		static uint32_t dcount;
		for (dcount = 0; dcount <= delay; dcount++) { // delay a bit
			Nop();
			ClrWdt(); // reset the WDT timer
		};
	}



	void config_pic(void)
	{
#ifdef P25K22
		OSCCON = 0x70; // internal osc 16mhz, CONFIG OPTION 4XPLL for 64MHZ
		OSCTUNE = 0xC0; // 4x pll
		TRISC = 0b11111100; // [0..1] outputs for DIAG leds [2..7] for analog
		LATC = 0x00; // all LEDS on
		TRISAbits.TRISA6 = 0; // CPU clock out

		TRISBbits.TRISB1 = 1; // SSP2 pins clock in SLAVE
		TRISBbits.TRISB2 = 1; // SDI
		TRISBbits.TRISB3 = 0; // SDO
		TRISBbits.TRISB0 = 1; // SS2

		/* ADC channels setup */
		TRISAbits.TRISA0 = HIGH; // an0
		TRISAbits.TRISA1 = HIGH; // an1
		TRISAbits.TRISA2 = HIGH; // an2
		TRISAbits.TRISA3 = HIGH; // an3
		TRISAbits.TRISA5 = HIGH; // an4
		TRISBbits.TRISB4 = HIGH; // an11
		TRISBbits.TRISB0 = HIGH; // an12 SS2, don't use for analog
		TRISBbits.TRISB5 = HIGH; // an13
		TRISCbits.TRISC2 = HIGH; // an14
		TRISCbits.TRISC3 = HIGH; // an15
		TRISCbits.TRISC4 = HIGH; // an16
		TRISCbits.TRISC5 = HIGH; // an17
		TRISCbits.TRISC6 = HIGH; // an17
		TRISCbits.TRISC7 = HIGH; // an18

		TRISBbits.TRISB4 = 1; // QEI encoder inputs
		TRISBbits.TRISB5 = 1;
		TRISBbits.TRISB6 = LOW; /* outputs */
		TRISBbits.TRISB7 = LOW;

		ANSELA = 0b00101111; // analog bit enables
		ANSELB = 0b00110000; // analog bit enables
		ANSELC = 0b11111100; // analog bit enables
		VREFCON0 = 0b11100000; // ADC voltage ref 2.048 volts
		OpenADC(ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_4_TAD, ADC_CH0 & ADC_INT_ON, ADC_REF_FVR_BUF & ADC_REF_VDD_VSS); // open ADC channel
#endif

		PIE1bits.ADIE = HIGH; // the ADC interrupt enable bit
		IPR1bits.ADIP = HIGH; // ADC use high pri

#if defined(P8722) || defined(P25K22)
		OpenSPI2(SLV_SSON, MODE_11, SMPMID); // Must be SMPMID in slave mode
		SPI_BUF = CMD_DUMMY_CFG;
#endif

		/* System activity timer, can reset the processor */
		OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);
		WriteTimer0(TIMEROFFSET); //      start timer0 at 1 second ticks

#ifdef P25K22
		/* clear SPI module possible flag and enable interrupts*/
		PIR3bits.SSP2IF = LOW;
		PIE3bits.SSP2IE = HIGH;
		/* Enable interrupt priority */
		RCONbits.IPEN = 1;
		/* Enable all high priority interrupts */
		INTCONbits.GIEH = 1;
		/* clear any SSP error bits */
		SSP2CON1bits.WCOL = SSP2CON1bits.SSPOV = LOW;
#endif

	}

	void main(void) /* SPI Master/Slave loopback */
	{
		int16_t i, j, k = 0, num_ai_chan = 0;
		uint8_t stuff;

		config_pic(); // setup the slave for work

		while (1) { // just loop and output results on DIAG LCD for 8722
				if (SSP2CON1bits.WCOL || SSP2CON1bits.SSPOV) { // check for overruns/collisions
					SSP2CON1bits.WCOL = SSP2CON1bits.SSPOV = 0;
					spi_stat.adc_error_count = spi_stat.adc_count - spi_stat.adc_error_count;
				}


				for (i = 0; i < 1; i++) {
					for (j = 0; j < 1; j++) {
						_asm clrwdt _endasm // reset the WDT timer
					}
				}

			}

		}
