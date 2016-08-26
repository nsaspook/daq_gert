
/* Parts of this code were modified from
 *  http://www.d.umn.edu/~cprince/PubRes/Hardware/SPI/
 * examples
 *
 * Fully interrupt drived SPI slave ADC for RPi via the daq_gert linux module
 * Port E is the main led diag port
 * PORT H is the LCD port
 * SPI 1 has been config'd as the slave with no select, with SPI 2 as Master.
 * The I/O and clock pins
 * have been interconnected in the standard way for a PIC18F8722 chip
 *
 * Version	0.01 The testing hardware is mainly a pic18f8722 with a
 *		LCD display and PORTE bit leds.
 *		The target hardware for field use will be the pic18f25k22
 *		due to its 28 pin dip format.
 *		define the CPU type below.
 *
 * nsaspook@nsaspook.com    Oct 2012
 */

//#define P25K22
#define P8722
//#define P8722_SLAVE

#ifdef P8722
#include "xlcd.h"
#include <p18f8722.h>
#pragma	config OSC = HSPLL
#pragma config WDT = ON, WDTPS = 1024
#pragma config BOREN = OFF
#pragma config STVREN = ON
#pragma config LVP=OFF
#endif

#ifdef P25K22
#include <p18f25k22.h>
#pragma	config	FOSC = INTIO7
#pragma config	PLLCFG=ON
#pragma config	WDTEN = OFF
#pragma config	CCP2MX = PORTC1, PBADEN = OFF, T3CMX = PORTC0
#pragma config	BOREN = OFF
#pragma config	STVREN = ON
#pragma config	LVP=OFF
#endif

#include <spi.h>
#include <timers.h>
#include <adc.h>
#include <delays.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <GenericTypeDefs.h>

/* bit 7 high for command
 * bit 7 low  for config data in CMD_DUMMY per uC type
 * bits 6 config bit code always 1
 * bit	5 0=ADC ref VDD 1=ADC rec FVR=2.048
 * bit  4 0=10bit adc, 1=12bit adc
 * bits 3..0 port address
 *
 */

#define	TIMEROFFSET		26474           // timer0 16bit counter value for 1 second to overflow

#define CS_SLAVE	LATDbits.LATD0
#define CMD_ADC_GO	0b10000000
#define CMD_ADC_GO_0	0b10000000
#define CMD_ADC_GO_1	0b10000001
#define CMD_ADC_DONE	0b11110000
#define CMD_ADC_DATAL	0b10010000
#define CMD_ADC_DATAH	0b10100000

#define CMD_DUMMY_CFG	0b01000000
#ifdef P8722
#define CMD_DUMMY	0b01001100	/* 12 channels VDD */
#define NUM_AI_CHAN	12
#endif
#ifdef P25K22
#define CMD_DUMMY	0b01101001	/* 9 channels 2.048 but only 8 are ADC */
#define NUM_AI_CHAN     9
#endif

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
#define	HIGH            (unsigned char)1        // digital output state levels, source
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

const rom int8_t *build_date = __DATE__, *build_time = __TIME__;
volatile uint8_t data_in1, data_in2, adc_buffer_ptr = 0,
	adc_channel = 0, SPI_DATA = FALSE, ADC_DATA = FALSE,
	REMOTE_LINK = FALSE, REMOTE_DATA_DONE = FALSE, LOW_BITS = FALSE;
volatile uint8_t dsi = 0; // LCD virtual console number
volatile uint32_t adc_count = 0, adc_error_count = 0, master_int_count = 0,
	slave_int_count = 0, last_slave_int_count = 0;
volatile uint16_t adc_buffer[64] = {0}, adc_data_in = 0;
#pragma udata gpr13
far int8_t bootstr2[MESG_W + 1];
uint8_t lcd18 = 200;
#pragma udata gpr2
far struct lcdb ds[VS_SLOTS];
#pragma udata gpr9

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
    static uint8_t channel = 0, dummy_read = 0;
    static union Timers timer;

    if (INTCONbits.TMR0IF) { // check timer0 irq 1 second timer int handler
	INTCONbits.TMR0IF = LOW; //clear interrupt flag
	//check for TMR0 overflow

	timer.lt = TIMEROFFSET; // Copy timer value into union
	TMR0H = timer.bt[HIGH]; // Write high byte to Timer0
	TMR0L = timer.bt[LOW]; // Write low byte to Timer0
	if ((slave_int_count - last_slave_int_count) < 10) {
	    ClrWdt(); // reset the WDT timer
	}
    }

    if (PIR1bits.ADIF) { // ADC conversion complete flag
	PIR1bits.ADIF = LOW;
	adc_count++; // just keep count
	adc_buffer[channel] = ADRES;
	SSP1BUF = (uint8_t) adc_buffer[channel]; // stuff with lower 8 bits
	ADC_DATA = TRUE;
    }

    if (PIR1bits.SSP1IF) { // SPI port #1 SLAVE receiver
	PIR1bits.SSP1IF = LOW;
	slave_int_count++;
	data_in1 = SSP1BUF;
	if ((data_in1 & 0xf0) == CMD_ADC_GO) { // Found a Master command
	    channel = data_in1 & 0x0f;
#ifdef P25K22
	    if (channel > 4) channel += 7; // skip missing channels
	    if (channel > 14) channel = 0; // invalid to set to 0
#endif
#ifdef P8722
	    if (channel > 11) channel = 0; // invalid so set to 0
#endif
	    if (!ADCON0bits.GO) {
		ADCON0 = ((channel << 2) & 0b00111100) | (ADCON0 & 0b11000011);
		ADC_DATA = FALSE;
		ADCON0bits.GO = 1; // start a conversion
#ifdef P8722
		LATEbits.LATE1 = !LATEbits.LATE1;
#endif
	    } else {
		ADCON0bits.GO = 0; // stop a conversion
		SSP1BUF = CMD_DUMMY; // Tell master  we are here
		ADC_DATA = FALSE;
#ifdef P8722
		LATEbits.LATE2 = !LATEbits.LATE2;
#endif
	    }
	}
	if (data_in1 == CMD_DUMMY_CFG) {
	    SSP1BUF = CMD_DUMMY; // Tell master  we are here
#ifdef P8722
	    LATEbits.LATE3 = !LATEbits.LATE3;
#endif
	}
	if (data_in1 == CMD_ADC_DATAL) {
	    if (!ADCON0bits.GO) {
		SSP1BUF = (uint8_t) (adc_buffer[channel] >> 8); // stuff with upper 8 bits
#ifdef P8722
		LATEbits.LATE4 = !LATEbits.LATE4;
#endif
		last_slave_int_count = slave_int_count;
		ClrWdt(); // reset the WDT timer
	    } else {
#ifdef P8722
		LATEbits.LATE5 = !LATEbits.LATE5;
#endif
		SSP1BUF = CMD_DUMMY;
	    }
	}
	if (data_in1 == CMD_ADC_DATAH) {
	    if (!ADCON0bits.GO) {
		SSP1BUF = (uint8_t) adc_buffer[channel]; // stuff with lower 8 bits
	    } else {
		SSP1BUF = CMD_DUMMY;
	    }
	}
    }





#ifdef P8722
    if (PIR3bits.SSP2IF) { // SPI port #2 MASTER receiver
	PIR3bits.SSP2IF = LOW;
	master_int_count++;
	data_in2 = SSP2BUF;
	if (REMOTE_DATA_DONE) {
	    if ((data_in2 & 0b11000000) == CMD_DUMMY_CFG) { // Crack that whip
	    }
	} else {
	    if (LOW_BITS) {
		adc_data_in = (uint8_t) data_in2;
		LOW_BITS = FALSE;
		LATEbits.LATE6 = !LATEbits.LATE6;
	    } else {
		adc_data_in += (uint16_t) ((uint16_t) data_in2 << 8);
		LATEbits.LATE7 = !LATEbits.LATE7;

		REMOTE_DATA_DONE;
	    }
	}
	SPI_DATA = TRUE;
    }
#endif

}
#pragma code

void wdtdelay(unsigned long delay)
{
    static uint32_t dcount;
    for (dcount = 0; dcount <= delay; dcount++) { // delay a bit
	Nop();
	ClrWdt(); // reset the WDT timer
    };
}

#ifdef P8722

void DelayFor18TCY(void)
{
    static uint8_t n;
    _asm nop _endasm // asm code to disable compiler optimizations
    for (n = 0; n < lcd18; n++) Nop(); // works at 200 (slow white) or 24 (fast blue)
}

//------------------------------------------

void DelayPORXLCD(void) // works with 15
{
    Delay10KTCYx(15); // Delay of 15ms
    return;
}

//------------------------------------------

void DelayXLCD(void) // works with 5
{
    Delay10KTCYx(5); // Delay of 5ms
    return;
}

void LCD_VC_puts(unsigned char console, unsigned char line, unsigned char COPY) // VCx,DSx, [TRUE..FALSE} copy data from bootstr2 string
{ // into the LCD display buffer
    static uint8_t ib = 0;

    if (COPY) {
	ib = console + line; // set to string index to store data in LCD message array ds[x].b
	strncpypgm2ram(ds[ib].b, "                        ", LCD_W); // write 20 space chars
	strncpy(ds[ib].b, bootstr2, LCD_W); // move data from static buffer in lcd message array
	ds[ib].b[LCD_W] = 0; // make sure we have a string terminator
    }
    switch (line) {
    case DS0:
	SetDDRamAddr(LL1); // move to  line
	break;
    case DS1:
	SetDDRamAddr(LL2); // move to  line
	break;
    case DS2:
	SetDDRamAddr(LL3); // move to  line
	break;
    case DS3:
	SetDDRamAddr(LL4); // move to  line
	break;
    default:
	SetDDRamAddr(LL1); // move to  line 1 of out of range
	break;
    }
    ib = dsi + line; // set to string index to display on LCD, dsi GLOBAL is the current VC being displayed

    while (BusyXLCD());
    putsXLCD(ds[ib].b);
    while (BusyXLCD());
}

void init_lcd(void)
{
    lcd18 = 200;
    wdtdelay(10000); // delay for power related LCD setup glitch
    if (BusyXLCD()) {
	OpenXLCD(FOUR_BIT & LINES_5X7);
	while (BusyXLCD());
	wdtdelay(10000); // delay for power related LCD setup glitch
	OpenXLCD(FOUR_BIT & LINES_5X7);
	while (BusyXLCD());
	WriteCmdXLCD(0xc); // blink, cursor off
	while (BusyXLCD());
	WriteCmdXLCD(0x1); // clear screen
	wdtdelay(10000);
	lcd18 = 24;
    }
}
#endif

void adc_conv_delay(void)
{
    int16_t i, j;

    for (i = 0; i < 1; i++) {
	for (j = 0; j < 20; j++) {
	}
    }
}

void slave_cs_delay(void)
{
    int16_t i, j;

    for (i = 0; i < 1; i++) {
	for (j = 0; j < 10; j++) {
	}
    }
}

void clear_spi_data_flag(void)
{
    SPI_DATA = FALSE;
    CS_SLAVE = LOW;
    slave_cs_delay();
}

uint8_t spi_data_recd(void)
{
    uint32_t delay = 0;
    while (!SPI_DATA) {
	if (delay++ > 50000) {
	    CS_SLAVE = HIGH;
	    return FALSE;
	}
    }
    slave_cs_delay();
    CS_SLAVE = HIGH;
    return TRUE;
}

void main(void) /* SPI Master/Slave loopback */
{
    int16_t i, j, k = 0, num_ai_chan = 0, adc_data_recv[16] = {0};
    uint8_t junk, stuff;

#ifdef P8722
    TRISB = 0xff; // external interrupts for later
    TRISE = 0;
    TRISF = 0;
    TRISJ = 0;
    TRISH = LOW; // mpuled and LCD
    LATE = 0xff;
    LATJ = 0xff;
    LATH = 0xff;

    TRISDbits.TRISD6 = 0; // SSP2 pins clk out MASTER
    TRISDbits.TRISD5 = 1; // SDI
    TRISDbits.TRISD4 = 0; // SDO
    TRISDbits.TRISD0 = 0; // CS

    TRISCbits.TRISC3 = 1; // SSP1 pins clk in SLAVE
    TRISCbits.TRISC4 = 1; // SDI
    TRISCbits.TRISC5 = 0; // SDO

    ADCON1 = 0x03; // adc [0..11] enable
#endif
#ifdef P25K22
    TRISCbits.TRISC6 = 0; // Diag led
    TRISCbits.TRISC7 = 0; // Diag led
    TRISAbits.TRISA6 = 0; // clock out

    TRISBbits.TRISB1 = 0; // SSP2 pins MASTER
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB3 = 0;

    TRISCbits.TRISC3 = 1; // SSP1 pins SLAVE  
    TRISCbits.TRISC4 = 1;
    TRISCbits.TRISC5 = 0;


    /* ADC channels setup */
    TRISAbits.TRISA0 = HIGH; // an0
    TRISAbits.TRISA1 = HIGH; // an1
    TRISAbits.TRISA2 = HIGH; // an2
    TRISAbits.TRISA3 = HIGH; // an3
    TRISAbits.TRISA5 = HIGH; // an4
    TRISBbits.TRISB4 = HIGH; // an11
    TRISBbits.TRISB0 = HIGH; // SS2, don't use for analog
    TRISBbits.TRISB5 = HIGH; // an13 
    TRISCbits.TRISC2 = HIGH; // an14

    ANSELA = 0b00101111; // analog bit enables
    ANSELB = 0b00110000; // analog bit enables
    ANSELC = 0b00000100; // analog bit enables
    VREFCON0 = 0b11100000; // ADC voltage ref 2.048 volts
    OpenADC(ADC_FOSC_RC & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH0 & ADC_INT_ON, ADC_REF_FVR_BUF & ADC_REF_VDD_VSS); // open ADC channel for current and voltage readings
#endif

#ifdef P8722
    TRISAbits.TRISA0 = HIGH; // an0
    TRISAbits.TRISA1 = HIGH; // an1
    TRISAbits.TRISA2 = HIGH; // an2
    TRISAbits.TRISA3 = HIGH; // an3
    TRISAbits.TRISA5 = HIGH; // an4
    TRISFbits.TRISF0 = HIGH; // an5
    TRISFbits.TRISF1 = HIGH; // an6
    TRISFbits.TRISF2 = HIGH; // an7
    TRISFbits.TRISF3 = HIGH; // an8
    TRISFbits.TRISF4 = HIGH; // an9
    TRISFbits.TRISF5 = HIGH; // an10
    TRISFbits.TRISF6 = HIGH; // an11
    TRISFbits.TRISF7 = LOW; // SS1
    LATFbits.LATF7 = 0; // enable slave

    OpenADC(ADC_FOSC_RC & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH0 & ADC_REF_VDD_VSS & ADC_INT_ON, ADC_12ANA); // open ADC channel for current and voltage readings
#endif

    PIE1bits.ADIE = HIGH; // the ADC interrupt enable bit
    IPR1bits.ADIP = HIGH; // ADC use high pri

    OpenSPI1(SLV_SSON, MODE_11, SMPMID); // Must be SMPMID in slave mode, Port C 4,5,3
#ifdef P8722
    OpenSPI2(SPI_FOSC_64, MODE_11, SMPMID); // Must be SMPMID in slave mode, Port D 4,5,6
#endif

    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);
    WriteTimer0(TIMEROFFSET); //      start timer0 at 1 second ticks

    SSP1BUF = CMD_DUMMY_CFG;
    SSP2BUF = CMD_DUMMY_CFG;

    PIR1bits.SSP1IF = 0;
    PIE1bits.SSP1IE = 1;
#ifdef P8722
#ifndef P8722_SLAVE
    PIR3bits.SSP2IF = 0;
    PIE3bits.SSP2IE = 1;
#endif
#endif
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;

#ifdef P8722
    init_lcd();
    strncpypgm2ram(bootstr2, build_time, LCD_W);
    LCD_VC_puts(VC0, DS0, YES);
    strncpypgm2ram(bootstr2, build_date, LCD_W);
    LCD_VC_puts(VC0, DS1, YES);

    for (i = 0; i < 1000; i++) {
	for (j = 0; j < 1000; j++) {
	    ClrWdt(); // reset the WDT timer
	}
    }
#endif

    SSP1CON1bits.WCOL = SSP2CON1bits.WCOL = SSP1CON1bits.SSPOV = SSP2CON1bits.SSPOV = 0;

    while (1) { // just loop asking for ADC 0 and 1 and output results on LCD
#ifdef P8722
	LATEbits.LATE0 = !LATEbits.LATE0;
#endif
	junk++;

#ifndef P8722_SLAVE
	LOW_BITS = TRUE;
	REMOTE_DATA_DONE = TRUE;
	clear_spi_data_flag();
	SSP2BUF = (CMD_ADC_GO & 0b11111110) | (junk & 0b00000001); // Master sends data
	if (spi_data_recd()) {
	    stuff = data_in2;
	    if (((data_in2 & 0b11000000) == CMD_DUMMY_CFG)) {
		num_ai_chan = data_in2 & 0x0f;

		REMOTE_DATA_DONE = FALSE;
		if (num_ai_chan > 0) {
		    REMOTE_LINK = TRUE;
		}
		adc_conv_delay();
		clear_spi_data_flag();
		SSP2BUF = CMD_ADC_DATAL; // Master sends cmd;
		spi_data_recd();
		clear_spi_data_flag();
		SSP2BUF = CMD_DUMMY_CFG; // Master sends DUMMY data;
		spi_data_recd();
		adc_data_recv[(junk & 0b00000001)] = adc_data_in;
#ifdef P8722_SLAVE
		adc_data_recv[(junk & 0b00000001)] = 0;
#endif
		REMOTE_DATA_DONE = TRUE;
	    } else {
		REMOTE_LINK = FALSE;
		clear_spi_data_flag();
		SSP2BUF = CMD_DUMMY_CFG; // Master sends DUMMY data;
		spi_data_recd();
		num_ai_chan = 0;
	    }
	} else {
	    REMOTE_LINK = FALSE;
	    clear_spi_data_flag();
	    SSP2BUF = CMD_DUMMY_CFG; // Master sends DUMMY data;
	    spi_data_recd();
	    num_ai_chan = 0;
	}

	if (SSP1CON1bits.WCOL || SSP2CON1bits.WCOL || SSP1CON1bits.SSPOV || SSP2CON1bits.SSPOV) { // check for overruns/collisions
#ifdef P8722
	    LATHbits.LATH0 = !LATHbits.LATH0;
#endif
	    SSP1CON1bits.WCOL = SSP2CON1bits.WCOL = SSP1CON1bits.SSPOV = SSP2CON1bits.SSPOV = 0;
	    adc_error_count = adc_count - adc_error_count;
	}
#endif


	for (i = 0; i < 1; i++) {
	    for (j = 0; j < 1; j++) {
#ifdef P8722
		if ((((k++) % 5000) == 0) || !REMOTE_LINK) {
		    if (REMOTE_LINK) {
			sprintf(bootstr2,
				"SPI U%i %b          ",
				num_ai_chan, stuff);
			LCD_VC_puts(VC0, DS0, YES);
		    } else {
			sprintf(bootstr2,
				"SPI D%lu %lu         ",
				last_slave_int_count, slave_int_count);
			LCD_VC_puts(VC0, DS0, YES);
		    }
		    if (ADC_DATA) {
			sprintf(bootstr2,
				"The ADC is Done         "
				);
			LCD_VC_puts(VC0, DS3, YES);
		    } else {
			sprintf(bootstr2,
				"The ADC is Working             "
				);
			LCD_VC_puts(VC0, DS3, YES);
		    }
		    sprintf(bootstr2,
			    "Err %lu, #%lu  I%lu    ",
			    adc_error_count, adc_count, master_int_count);
		    LCD_VC_puts(VC0, DS2, YES);
		    sprintf(bootstr2,
			    "D %u,%u A %u,%u     ",
			    adc_data_recv[0], adc_data_recv[1], adc_buffer[0], adc_buffer[1]);
		    LCD_VC_puts(VC0, DS1, YES);
		}
#endif



	    }
	}

    };

}
