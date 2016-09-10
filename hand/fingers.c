
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
 *
 * nsaspook@sma2.rain..com    Sept 2016
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
#include <limits.h> 
#include <ctmu.h>
#include <usart.h>
#include "finger.h"

/*
 * Hand controlled light toy
 */

volatile struct spi_stat_type spi_stat = {0};

const rom int8_t *build_date = __DATE__, *build_time = __TIME__;
const rom int8_t Version[] = "\r\n Version 0.1 PIC fingers ";

volatile uint8_t ctmu_button; // selects ADC/CTMU combo to monitor
volatile uint16_t adc_buffer[5][9] = {0}, charge_time[16];
volatile uint8_t CTMU_ADC_UPDATED = FALSE, TIME_CHARGE = FALSE, CTMU_WORKING = FALSE;
struct finger_move_type finger[5] = {0};
int8_t mesg[MESG_W], rs232_debug = RS232_DEBUG;

/* function prototype */
void InterruptHandlerHigh(void);

//High priority interrupt vector
#pragma code InterruptVectorHigh = 0x08

void InterruptVectorHigh(void)
{
	_asm
		goto InterruptHandlerHigh //jump to interrupt routine
		_endasm
}
#pragma code

// High priority interrupt routine
#pragma interrupt InterruptHandlerHigh

void InterruptHandlerHigh(void)
{
	static union Timers timer;
	static uint8_t i_adc = 0;

	if (INTCONbits.TMR0IF) { // check timer0 irq 
		if (!CTMUCONHbits.IDISSEN) { // charge cycle timer0 int, because not shorting the CTMU voltage.
			DLED1 = HIGH;
			CTMUCONLbits.EDG1STAT = 0; // Stop charging touch circuit
			TIME_CHARGE = FALSE; // clear charging flag
			CTMU_WORKING = TRUE; // set working flag, doing touch ADC conversion
			// configure ADC for next reading
			ADCON0bits.CHS = ctmu_button; // Select ADC channel
			ADCON0bits.ADON = 1; // Turn on ADC
			ADCON0bits.GO = 1; // and begin A/D conv, will set adc int flag when done.
		} else { // discharge cycle timer0 int, because CTMU voltage is shorted 
			DLED1 = LOW;
			ADCON0bits.CHS = ctmu_button; // Select ADC channel for charging
			CTMUCONHbits.IDISSEN = 0; // end drain of touch circuit
			TIME_CHARGE = TRUE; // set charging flag
			CTMU_WORKING = TRUE; // set working flag, doing 
			timer.lt = charge_time[ctmu_button]; // set timer to charge rate time
			TMR0H = timer.bt[HIGH]; // Write high byte to Timer0
			TMR0L = timer.bt[LOW]; // Write low byte to Timer0
			CTMUCONLbits.EDG1STAT = 1; // Begin charging the touch circuit
		}
		// clr  TMR0 int flag
		INTCONbits.TMR0IF = 0; //clear interrupt flag
	}

	if (PIR1bits.ADIF) { // check ADC irq
		PIR1bits.ADIF = 0; // clear ADC int flag
		spi_stat.adc_count++;
		PIR1bits.SSPIF = LOW; // clear SPI flags
		PIE1bits.SSP1IE = HIGH; // enable to send second byte
		SSP1BUF = ADRESH | ((ctmu_button << 4)&0xf3);
		adc_buffer[ctmu_button][i_adc] = ADRES;
		timer.lt = TIMERDISCHARGE; // set timer to discharge rate
		if (++i_adc >= ADC_READS) {
			TMR3H = ADRESH | ((ctmu_button << 4)&0xf3); // copy high byte/channel data [4..7] bits
			TMR3L = ADRESL; // copy low byte and write to timer counter
			i_adc = 0; // reset adc buffer position
			CTMU_ADC_UPDATED = TRUE; // New data is in timer3 counter, set to FALSE in main program flow
			timer.lt = TIMERPROCESS; // set timer to data processing rate
		}
		CTMU_WORKING = FALSE; // clear working flag, ok to read timer3 counter.
		// config CTMU for next reading
		CTMUCONHbits.CTMUEN = 1; // Enable the CTMU
		CTMUCONLbits.EDG1STAT = 0; // Set Edge status bits to zero
		CTMUCONLbits.EDG2STAT = 0;
		CTMUCONHbits.IDISSEN = 1; // drain charge on the circuit
		TMR0H = timer.bt[HIGH]; // Write high byte to Timer0
		TMR0L = timer.bt[LOW]; // Write low byte to Timer0
	}

	if (PIE1bits.SSP1IE && PIR1bits.SSPIF) { // SPI port #1 receiver
		PIR1bits.SSPIF = LOW;
		spi_stat.int_count++;
		spi_stat.data_in = SSP1BUF;
		PIE1bits.SSP1IE = LOW; // disable so we don't send again
		SSP1BUF = ADRESL;
	}

	if (PIR1bits.TMR1IF) {
		PIR1bits.TMR1IF = 0; // clear TMR2 int flag
		timer.lt = PDELAY;
		TMR1H = timer.bt[HIGH]; // Write high byte to Timer1
		TMR1L = timer.bt[LOW]; // Write low byte to Timer1
		spi_stat.time_tick++;

		/* simple software timer */
		if (spi_stat.bit_timer_set) {
			spi_stat.bit_timer_start = spi_stat.time_tick;
			spi_stat.bit_timer_clear = FALSE;
			spi_stat.bit_timer_set = FALSE;
		}

		if ((spi_stat.time_tick >= (spi_stat.bit_timer_value + spi_stat.bit_timer_start))) {
			spi_stat.bit_timer_clear = TRUE;
			spi_stat.bit_timer_start = USLONG_MAX - spi_stat.bit_timer_value;
		}
	}
}

int16_t ctmu_touch(uint8_t channel, uint8_t diff_val)
{
	int16_t ctmu_change;
	uint8_t i;

	if (CTMU_ADC_UPDATED) {
		finger[channel].moving_val = finger[channel].avg_val;
		finger[channel].avg_val = 0;
		for (i = 0; i < 8; i++) {
			finger[channel].avg_val += adc_buffer[channel][i]&0x03ff;
		}
		finger[channel].avg_val = finger[channel].avg_val >> (uint16_t) 3;
		finger[channel].moving_avg = (finger[channel].moving_val + finger[channel].avg_val) >> (uint16_t) 1;

		if (!diff_val) {
			return finger[channel].avg_val;
		}
		ctmu_change = finger[channel].zero_ref - finger[channel].moving_avg; // read diff 
		return ctmu_change;
	} else {
		return 0;
	}
}

/*
 * compute the gesture zero
 */
uint16_t touch_base_calc(uint8_t channel)
{
	uint8_t i;

	touch_channel(channel);
	CTMU_ADC_UPDATED = FALSE;
	while (!CTMU_ADC_UPDATED) ClrWdt(); // wait for touch update cycle
	finger[channel].avg_val = 0;
	finger[channel].zero_noise = 0;
	finger[channel].zero_max = adc_buffer[channel][0]&0x03ff;
	finger[channel].zero_min = adc_buffer[channel][0]&0x03ff;
	for (i = 0; i < 8; i++) {
		finger[channel].avg_val += adc_buffer[channel][i]&0x03ff;
		if (adc_buffer[channel][i]&0x03ff > finger[channel].zero_max) // look at the noise spreads
			finger[channel].zero_max = adc_buffer[channel][i]&0x03ff;
		if (adc_buffer[channel][i]&0x03ff < finger[channel].zero_min)
			finger[channel].zero_min = adc_buffer[channel][i]&0x03ff;
	}
	finger[channel].avg_val = finger[channel].avg_val >> (uint16_t) 3;
	finger[channel].zero_ref = finger[channel].avg_val;
	if ((finger[channel].zero_max - finger[channel].zero_min) > ZERO_NOISE)
		finger[channel].zero_noise = 1;
	return finger[channel].zero_ref;
}

void touch_channel(uint8_t channel)
{
	CTMU_ADC_UPDATED = FALSE;
	while (!CTMU_ADC_UPDATED); // wait for touch update cycle
	ctmu_button = channel;
	CTMU_ADC_UPDATED = FALSE;
	while (!CTMU_ADC_UPDATED); // wait for touch update cycle
}

int16_t ctmu_setup(uint8_t current, uint8_t channel)
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
	case 2:
		CTMUICON = 0x02; //5.5uA, Nominal - No Adjustment
		charge_time[channel] = TIMERCHARGE_BASE_X10; // faster
		break;
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

	// timer3 register used for atomic data transfer
	T3CONbits.TMR3ON = 0; // Timer is off
	T3CONbits.T3RD16 = 1; // enable 16 bit reads/writes
	TMR3H = 0;
	TMR3L = 0;
	return 0;
}

void ctmu_zero_set(void)
{
	uint8_t i, max_count;

	for (i = 0; i < 4; i++) {
		max_count = 0;
		do {
			touch_base_calc(i);
			if (finger[i].zero_noise)
				if (++max_count > 64)
					break;
		} while (finger[i].zero_noise);
	}
}

int16_t finger_diff(int16_t finger1, int16_t finger2)
{
	return abs(finger1 - finger2);
}

/* bit rotations for 32 bit led motion control */
uint32_t rotl32(uint32_t value, unsigned int count)
{
	const unsigned int mask = (CHAR_BIT * sizeof(value) - 1);
	count &= mask;
	return(value << count) | (value >> ((-count) & mask)); // unary minus warning cheated with -nw=2059
}

uint32_t rotr32(uint32_t value, unsigned int count)
{
	const unsigned int mask = (CHAR_BIT * sizeof(value) - 1);
	count &= mask;
	return(value >> count) | (value << ((-count) & mask));
}

void led_motion(uint8_t mode, uint8_t bit0, uint8_t bit1, uint8_t bit2, uint8_t bit3)
{
	FLED0 = bit0;
	FLED1 = bit1;
	FLED2 = bit2;
	FLED3 = bit3;

	if (mode)
		return;

	spi_stat.bit_timer_clear = FALSE;
	spi_stat.bit_timer_set = TRUE;
	while (!spi_stat.bit_timer_clear) ClrWdt();

}

int16_t finger_trigger(uint8_t channel_count)
{
	static uint32_t roller0 = ROLL_PATTERN0, roller1 = ROLL_PATTERN1, roller2 = ROLL_PATTERN2, roller3 = ROLL_PATTERN3;
	/* check finger trigger conditions */
	if (((finger[0].moving_diff > TRIP) && (finger[1].moving_diff > TRIP)) && (finger_diff(finger[0].moving_diff, finger[1].moving_diff) < TRIP_DIFF)) {
		led_motion(0, roller0 & 0x1, roller1 & 0x1, roller2 & 0x1, roller3 & 0x1);
		if (rs232_debug) {
			sprintf(mesg, " %u:%d:%d:%d:%d %d:%d diff %d: rotr %lu\r\n", channel_count, finger[channel_count].zero_ref, (int16_t) finger[channel_count].avg_val,
				finger[channel_count].moving_avg, finger[channel_count].moving_val,
				finger[0].moving_diff, finger[1].moving_diff, finger_diff(finger[0].moving_diff, finger[1].moving_diff), roller0);
			puts1USART(mesg);
		}
		roller0 = rotr32(roller0, 1);
		roller1 = rotr32(roller1, 1);
		roller2 = rotr32(roller2, 1);
		roller3 = rotr32(roller3, 1);
	} else {
		led_motion(1, 1, 1, 1, 1);
	}
	return 0;
}

void config_pic(void)
{
	OSCCON = 0x70; // internal osc 16mhz, CONFIG OPTION 4XPLL for 64MHZ
	OSCTUNE = 0xC0; // 4x pll
	WPUB = 0xff; // pull ups ON
	SLRCON = 0x00; // slew rate to max for high speed SPI

	INTCON2bits.RBPU = LOW; // turn on weak pullups
	TRISAbits.TRISA6 = 0; // CPU clock out

	TRISCbits.TRISC3 = 0; // clock out Master
	TRISCbits.TRISC4 = 1; // SDI
	TRISCbits.TRISC5 = 0; // SDO
	TRISCbits.TRISC2 = 0; // CS0

	TRISCbits.TRISC6 = 0; // tx rs232
	TRISCbits.TRISC7 = 1; // rx

	/* ADC channels setup */
	TRISAbits.TRISA0 = HIGH; // an0
	TRISAbits.TRISA1 = HIGH; // an1
	TRISAbits.TRISA2 = HIGH; // an2
	TRISAbits.TRISA3 = HIGH; // an3

	TRISB = 0x00; //finger led outputs 
	LATB = 0xff; // all leds off

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

	/* System activity timer */
	OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_1);
	WriteTimer0(TIMERDISCHARGE); //	start timer0 

	/* tick timer */
	OpenTimer1(T1_SOURCE_FOSC_4 & T1_16BIT_RW & T1_PS_1_8 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF, 0);
	WriteTimer1(PDELAY);
	spi_stat.bit_timer_value = BIT_TIMER_VALUE; // led motion timer
	spi_stat.bit_timer_start = USLONG_MAX - spi_stat.bit_timer_value;

	/* clear SPI module possible flag and enable interrupts*/
	PIR1bits.SSP1IF = LOW;
	PIE1bits.SSP1IE = HIGH;

	/*
	 * CTMU
	 */

	//CTMUCONH/1 - CTMU Control registers
	CTMUCONH = 0x04; //make sure CTMU is disabled
	CTMUICON = 0x03; //.55uA*100, Nominal - No Adjustment default
	CTMUCONL = 0x90;
	CTMUCONHbits.CTMUEN = HIGH; //Enable the CTMU
	CTMUCONHbits.IDISSEN = HIGH; // drain the circuit
	CTMUCONHbits.CTTRIG = LOW; // disable trigger

	/* diag output only */
	Open1USART(USART_TX_INT_OFF &
		USART_RX_INT_OFF &
		USART_ASYNCH_MODE &
		USART_EIGHT_BIT &
		USART_CONT_RX &
		USART_BRGH_LOW, 51); // 64mhz Fosc 38400 baud
	SPBRGH1 = 0x00;
	SPBRG1 = 25;

	/* DIAG ports */
	TRISAbits.TRISA7 = 0; // out
	TRISCbits.TRISC0 = 0;
	DLED0 = HIGH;
	DLED1 = HIGH;

	/* Enable interrupt priority */
	RCONbits.IPEN = 1;
	/* Enable all high priority interrupts */
	INTCONbits.GIEH = 1;
	/* clear any SSP error bits */
	SSP2CON1bits.WCOL = SSP2CON1bits.SSPOV = LOW;
}

void main(void) /* SPI Master/Slave loopback */
{
	uint8_t i, channel_count, recal_count = 0;

	config_pic(); // setup the slave for work
	putrs1USART(Version);
	putrs1USART(build_date);
	putrs1USART(" ");
	putrs1USART(build_time);
	putrs1USART("\r\n");

	//		CTMU setups
	ctmu_button = 0; // select start touch input
	for (i = 0; i < 4; i++) {
		ctmu_setup(2, i); // config the CTMU for touch response 
		ctmu_zero_set();
	}
	ctmu_button = 0; // select start touch input
	channel_count = 0;

	while (1) { // just loop 

		/* update error stats */
		if (SSP2CON1bits.WCOL || SSP2CON1bits.SSPOV) { // check for overruns/collisions
			SSP2CON1bits.WCOL = SSP2CON1bits.SSPOV = 0;
			spi_stat.adc_error_count = spi_stat.adc_count - spi_stat.adc_error_count;
			spi_stat.last_int_count = spi_stat.int_count;
		}

		_asm clrwdt _endasm // reset the WDT timer

		/* update detector data */
		CTMU_ADC_UPDATED = FALSE;
		while (!CTMU_ADC_UPDATED); // wait for complete channel touch update cycle

		if (ctmu_button == SCAN_MAX_CHAN) DLED0 = !DLED0;
		/* clean up some detector noise */
		if (finger[channel_count].avg_val > finger[channel_count].zero_ref) {
			finger[channel_count].zero_ref = finger[channel_count].avg_val;
		}

		/* check for finger trips */
		finger[0].moving_diff = ctmu_touch(0, 1);
		finger[1].moving_diff = ctmu_touch(1, 1);

		/* check finger trigger conditions */
		finger_trigger(channel_count);

		/* reset the finger zeros on a schedule */
		if (++channel_count > SCAN_MAX_CHAN) {
			channel_count = 0;
			if (!recal_count++)
				ctmu_zero_set();
		}
		ctmu_button = channel_count;
	}

}
