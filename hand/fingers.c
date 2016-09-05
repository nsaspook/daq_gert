
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
 * nsaspook@sma2.rain..com    Mar 2015
 */

#define P25K22

// PIC18F25K22 Configuration Bit Settings

// 'C' source line config statements

#include <p18f25k22.h>

// CONFIG1H
#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block)
#pragma config PLLCFG = ON      // 4X PLL Enable (Oscillator multiplied by 4)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (Power up timer disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = ON       // Watchdog Timer Enable bits (WDT is always enabled. SWDTEN bit has no effect)
#pragma config WDTPS = 1024     // Watchdog Timer Postscale Select bits (1:1024)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are configured as digital I/O on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output and ready status are not delayed by the oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
#pragma config P2BMX = PORTB5   // ECCP2 B output mux bit (P2B is on RB5)
#pragma config MCLRE = EXTMCLR  // MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = ON       // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode enabled)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#include <spi.h>
#include <timers.h>
#include <adc.h>
#include <delays.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ctmu.h>
#include <GenericTypeDefs.h>

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

//	CTMU section
unsigned int touch_base_calc(unsigned char);
void touch_channel(unsigned char);
unsigned int ctmu_touch(unsigned char, unsigned char);
int ctmu_setup(unsigned char, unsigned char);

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

#define CMD_DUMMY	0b01101110	/* 14 channels 2.048 but only 13 are ADC */
#define NUM_AI_CHAN     14
#define SPI_BUF		SSP2BUF

#define	HI_NIBBLE	0xf0
#define	LO_NIBBLE	0x0f
#define	ADC_SWAP_MASK	0b01000000
#define UART_DUMMY_MASK	0b01000000

#define	TIMERCHARGE_BASE_X10		65523		// 5.5 uA time, large plate ~150us
#define	TIMERCHARGE_BASE_1		64000		// .55 uA time, large plate max sens ~700us
#define	TIMERCHARGE_BASE_2		61543		// .55 uA time, large plate low sens ~1000us
#define	TIMERCHARGE_BASE_3		65000		// .55 uA time, small plate max sens ~200us
#define	TIMERCHARGE_BASE_4		62543		// .55 uA time, small plate low sens ~750us
#define	TIMERDISCHARGE			41000		// discharge and max touch data update period 1.8ms

#define TRIP 32 //Difference between pressed
//and un-pressed switch
#define HYST 8 //amount to change
//from pressed to un-pressed
#define PRESSED 1
#define UNPRESSED 0
#define	CHOP_BITS	1

/* LCD defines */
#define LCD_L           4                       // lines
#define LCD_W           20			// chars per line
#define LCD_STR         22                 // char string for LCD messages
#define LCDW_SIZE       21              // add term char
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
#define LOW		(unsigned char)0        // digital output state levels, sink
#define	HIGH		(unsigned char)1        // digital output state levels, source
#define	ON		LOW       		//
#define OFF		HIGH			//
#define	S_ON            LOW       		// low select/on for chip/led
#define S_OFF           HIGH			// high deselect/off chip/led
#define	R_ON            HIGH       		// control relay states, relay is on when output gate is high, uln2803,omron relays need the CPU at 5.5vdc to drive
#define R_OFF           LOW			// control relay states
#define R_ALL_OFF       0x00
#define R_ALL_ON	0xff
#define NO		LOW
#define YES		HIGH

#define DLED0		LATAbits.LATA7

#ifdef INTTYPES
#include <stdint.h>
#else
#define INTTYPES
/*unsigned types*/
typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long uint32_t;
typedef unsigned long long uint64_t;
/*signed types*/
typedef signed char int8_t;
typedef signed int int16_t;
typedef signed long int32_t;
typedef signed long long int64_t;
#endif

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

volatile struct spi_link_type spi_comm = {FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE};
volatile struct spi_stat_type spi_stat = {0}, report_stat = {0};

const rom int8_t *build_date = __DATE__, *build_time = __TIME__;
const rom char Version[] = " Version 0.9 PIC Slave ";
volatile uint8_t data_in2, adc_buffer_ptr = 0,
	adc_channel = 0;

volatile uint8_t dsi = 0, ctmu_button, PEAK_READS = 2; // LCD virtual console number
volatile uint16_t adc_buffer[64] = {0}, adc_data_in = 0;
#pragma udata gpr13
far int8_t bootstr2[MESG_W + 1];
uint8_t lcd18 = 200;
#pragma udata gpr2
far struct lcdb ds[VS_SLOTS];
#pragma udata gpr9
volatile unsigned char CTMU_ADC_UPDATED = FALSE, TIME_CHARGE = FALSE, CTMU_WORKING = FALSE;
volatile unsigned int touch_base[16], switchState = UNPRESSED, charge_time[16]; //storage for reading parameters

void InterruptHandlerHigh(void);

//High priority interrupt vector
#pragma code InterruptVectorHigh = 0x08

void InterruptVectorHigh(void)
{
	_asm
		goto InterruptHandlerHigh //jump to interrupt routine
		_endasm
}

//----------------------------------------------------------------------------
// High priority interrupt routine

#pragma code

#pragma interrupt InterruptHandlerHigh

void InterruptHandlerHigh(void)
{
	static uint8_t channel = 0, link, upper, command, port_tmp, char_txtmp, char_rxtmp, cmd_dummy = CMD_DUMMY, b_dummy;
	static union Timers timer;
	static unsigned char i = 0;
	static unsigned int touch_peak = 1024; // max CTMU voltage


	DLED0 = HIGH;

	if (PIR1bits.TX1IF) { // check timer1 irq 
		if (!CTMUCONHbits.IDISSEN) { // charge cycle timer1 int, because not shorting the CTMU voltage.
			CTMUCONLbits.EDG1STAT = 0; // Stop charging touch circuit
			TIME_CHARGE = FALSE; // clear charging flag
			CTMU_WORKING = TRUE; // set working flag, doing touch ADC conversion
			// configure ADC for next reading
			channel = (TMR3H >> 4)&0x0f; // ADC channel is TMR3H [4..7] bits
			ADCON0bits.CHS = channel; // Select ADC channel, TMR3H[4..7]
			ADCON0bits.ADON = 1; // Turn on ADC
			ADCON0bits.GO = 1; // and begin A/D conv, will set adc int flag when done.
		} else { // discharge cycle timer0 int, because CTMU voltage is shorted 
			CTMUCONHbits.IDISSEN = 0; // end drain of touch circuit
			TIME_CHARGE = TRUE; // set charging flag
			CTMU_WORKING = TRUE; // set working flag, doing 
			WriteTimer0(charge_time[channel]); // set timer to charge rate time
			CTMUCONLbits.EDG1STAT = 1; // Begin charging the touch circuit
		}
		// clr  TMR1 int flag
		PIR1bits.TX1IF = 0; //clear interrupt flag
	}

	if (PIR1bits.ADIF) { // check ADC irq
		PIR1bits.ADIF = 0; // clear ADC int flag
		if (ADRES < touch_peak) touch_peak = ADRES; // find peak value
		if (i++ >= PEAK_READS) {
			timer.lt = touch_peak; // Get the value from the A/D
			timer.lt = (timer.lt >> CHOP_BITS)&0x03ff; // toss lower bit noise and mask
			if ((timer.lt) < (touch_base[channel] - TRIP)) { // see if we have a pressed button
				switchState = PRESSED;
			} else if ((timer.lt) > (touch_base[channel] - TRIP + HYST)) {
				switchState = UNPRESSED;
			}
			TMR3H = timer.bt[1] | ((channel << 4)&0xf3); // copy high byte/channel data [4..7] bits
			TMR3L = timer.bt[0]; // copy low byte and write to timer counter
			i = 0;
			touch_peak = 1024;
			CTMU_ADC_UPDATED = TRUE; // New data is in timer3 counter, set to FALSE in main program flow
		}
		CTMU_WORKING = FALSE; // clear working flag, ok to read timer3 counter.
		// config CTMU for next reading
		CTMUCONHbits.CTMUEN = 1; // Enable the CTMU
		CTMUCONLbits.EDG1STAT = 0; // Set Edge status bits to zero
		CTMUCONLbits.EDG2STAT = 0;
		CTMUCONHbits.IDISSEN = 1; // drain charge on the circuit
		WriteTimer1(TIMERDISCHARGE); // set timer to discharge rate
	}

	/* we only get this when the master  wants data, the slave never generates one */
	if (PIR3bits.SSP2IF) { // SPI port #2 SLAVE receiver
		PIR3bits.SSP2IF = LOW;
		spi_stat.slave_int_count++;
		data_in2 = SPI_BUF;
		command = data_in2 & HI_NIBBLE;

		if (command == CMD_ADC_GO) { // Found a GO for a conversion command
			spi_comm.ADC_DATA = FALSE;
			if (data_in2 & ADC_SWAP_MASK) {
				upper = TRUE;
			} else {
				upper = FALSE;
			}
			channel = data_in2 & LO_NIBBLE;
			if (channel >= 5) channel += 6; // skip missing channels
			if (channel == 12) channel = 0; // invalid so set to 0
			if (channel > 19) channel = 0; // invalid to set to 0


			if (!ADCON0bits.GO) {
				ADCON0 = ((channel << 2) & 0b00111100) | (ADCON0 & 0b11000011);
				adc_buffer[channel] = 0xffff; // fill with bits
				ADCON0bits.GO = HIGH; // start a conversion
			} else {
				ADCON0bits.GO = LOW; // stop a conversion
				SPI_BUF = CMD_DUMMY; // Tell master  we are here
			}
			spi_comm.REMOTE_LINK = TRUE;
			link = TRUE;
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
	}

	DLED0 = LOW;
}

unsigned int ctmu_touch(unsigned char channel, unsigned char NULL0)
{
	static unsigned int ctmu_change = 0, last = 0, null = 0;
	static union Timers timer;

	if (CTMU_ADC_UPDATED) {
		timer.bt[0] = TMR3L; // read low byte and read 16bits from timer counter into TMR3 16bit buffer
		timer.bt[1] = TMR3H; // read high byte
		if (!NULL0) {
			return(timer.lt & 0x03ff);
		}
		if (((timer.lt & 0x03ff))< (touch_base[channel]&0x03ff)) {
			ctmu_change = (touch_base[channel]&0x03ff)-(timer.lt & 0x03ff); // read diff 
			if (NULL0) {
				if (ctmu_change > 255) ctmu_change = 1;
			}
		}
		if ((null == 0) && NULL0) null = ctmu_change;
		last = ctmu_change;
		return(unsigned int) ctmu_change;
	} else {
		return(unsigned int) last;
	}
}

unsigned int touch_base_calc(unsigned char channel)
{
	long t_avg = 0, i;
	touch_channel(channel);
	CTMU_ADC_UPDATED = FALSE;
	while (!CTMU_ADC_UPDATED) ClrWdt(); // wait for touch update cycle
	for (i = 0; i < 8; i++) {
		CTMU_ADC_UPDATED = FALSE;
		while (!CTMU_ADC_UPDATED) ClrWdt(); // wait for touch update cycle
		t_avg += (ctmu_touch(channel, FALSE)&0x03ff);
	}
	touch_base[channel] = (unsigned int) (t_avg >> 3);
	return touch_base[channel];
}

void touch_channel(unsigned char channel)
{
	CTMU_ADC_UPDATED = FALSE;
	while (!CTMU_ADC_UPDATED); // wait for touch update cycle
	TMR3H = channel << 4; // set channel
	TMR3L = 0; // write to timer3 counter
	CTMU_ADC_UPDATED = FALSE;
	while (!CTMU_ADC_UPDATED); // wait for touch update cycle
}

int ctmu_setup(unsigned char current, unsigned char channel)
{
	//CTMUCONH/1 - CTMU Control registers
	CTMUCONH = 0x00; //make sure CTMU is disabled
	CTMUCONL = 0x90;
	//CTMU continues to run when emulator is stopped,CTMU continues
	//to run in idle mode,Time Generation mode disabled, Edges are blocked
	//No edge sequence order, Analog current source not grounded, trigger
	//output disabled, Edge2 polarity = positive level, Edge2 source =
	//source 0, Edge1 polarity = positive level, Edge1 source = source 0,
	//CTMUICON - CTMU Current Control Register
	CTMUICON = 0x01; //.55uA, Nominal - No Adjustment default

	switch (current) {
	case 11:
		charge_time[channel] = TIMERCHARGE_BASE_1;
		break;
	case 12:
		charge_time[channel] = TIMERCHARGE_BASE_2;
		break;
	case 13:
		charge_time[channel] = TIMERCHARGE_BASE_3;
		break;
	case 14:
		charge_time[channel] = TIMERCHARGE_BASE_4;
		break;
	default:
		charge_time[channel] = TIMERCHARGE_BASE_3; // slower
		break;
	}
	if (current == 0x02) {
		CTMUICON = 0x02; //5.5uA, Nominal - No Adjustment
		charge_time[channel] = TIMERCHARGE_BASE_X10; // faster
	}

	// timer3 register used for atomic data transfer
	T3CONbits.TMR3ON = 0; // Timer is off
	T3CONbits.T3RD16 = 1; // enable 16 bit reads/writes
	TMR3H = 0;
	TMR3L = 0;
	return 0;
}

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

	OSCCON = 0x70; // internal osc 16mhz, CONFIG OPTION 4XPLL for 64MHZ
	OSCTUNE = 0xC0; // 4x pll
	TRISAbits.TRISA6 = 0; // CPU clock out

	TRISCbits.TRISC3 = 0; // clock out Master
	TRISCbits.TRISC4 = 1; // SDI
	TRISCbits.TRISC5 = 0; // SDO
	TRISCbits.TRISC2 = 0; // CS0

	/* ADC channels setup */
	TRISAbits.TRISA0 = HIGH; // an0
	TRISAbits.TRISA1 = HIGH; // an1
	TRISAbits.TRISA2 = HIGH; // an2
	TRISAbits.TRISA3 = HIGH; // an3

	TRISBbits.TRISB4 = 1; // QEI encoder inputs
	TRISBbits.TRISB5 = 1;
	TRISBbits.TRISB6 = LOW; /* outputs */
	TRISBbits.TRISB7 = LOW;

	ANSELA = 0b00001111; // analog bit enables
	ANSELB = 0b00000000; // analog bit enables
	ANSELC = 0b00000000; // analog bit enables
	VREFCON0 = 0b11100000; // ADC voltage ref 2.048 volts
	OpenADC(ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_12_TAD, ADC_CH0 & ADC_INT_ON, ADC_REF_FVR_BUF & ADC_REF_VDD_VSS); // open ADC channel

	PIE1bits.ADIE = HIGH; // the ADC interrupt enable bit
	IPR1bits.ADIP = HIGH; // ADC use high pri

	/* SPI Master */
	OpenSPI1(SPI_FOSC_16, MODE_00, SMPEND); // 1MHz
	SSPCON1 |= SPI_FOSC_16; // set clock to low speed

	/* System activity timer, can reset the processor */
	OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);
	WriteTimer0(TIMEROFFSET); //      start timer0 at 1 second ticks
	OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_FOSC_4 & T1_PS_1_8 &
		T1_OSC1EN_OFF & T1_SYNC_EXT_OFF, 255); // for CTMU scanner

	/* clear SPI module possible flag and enable interrupts*/
	PIR1bits.SSP1IF = LOW;
	PIE1bits.SSP1IE = HIGH;

	/*
	 * CTMU
	 */

	//CTMUCONH/1 - CTMU Control registers
	CTMUCONH = 0x04; //make sure CTMU is disabled and ready for edge 1 before 2
	CTMUICON = 0x03; //.55uA*100, Nominal - No Adjustment default
	CTMUCONLbits.EDG1SEL = 3; // Set Edge CTED1
	CTMUCONLbits.EDG2SEL = 2; // CTED2
	CTMUCONLbits.EDG1POL = HIGH; // Set Edge
	CTMUCONLbits.EDG2POL = HIGH; // positive edges
	CTMUCONHbits.CTMUEN = HIGH; //Enable the CTMU
	CTMUCONHbits.IDISSEN = HIGH; // drain the circuit
	CTMUCONHbits.CTTRIG = LOW; // disable trigger

	/* Enable interrupt priority */
	RCONbits.IPEN = 1;
	/* Enable all high priority interrupts */
	INTCONbits.GIEH = 1;
	/* clear any SSP error bits */
	SSP2CON1bits.WCOL = SSP2CON1bits.SSPOV = LOW;
}

void main(void) /* SPI Master/Slave loopback */
{
	int16_t i, j, k = 0, num_ai_chan = 0;

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
